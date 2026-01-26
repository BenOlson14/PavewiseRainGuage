/************************************************************************************
Pavewise Mobile Rain Gauge — GPS/STRING TEST BUILD (TestSerial_fixed_gps_NEW.ino)

SUMMARY (what this build does every wake):
  1) Boots, downclocks CPU, disables WiFi/BT, mounts SD, and purges old logs
     if SD usage is above the threshold.
  2) Reads rainfall + battery voltage, then powers the modem and registers on
     the cellular network.
  3) Determines the current epoch: uses stored RTC estimate and re-anchors
     from GPS every 6 hours (or on first boot), with adaptive GPS timeout.
  4) Builds a compact payload (IMEI|batt_mv|rain_x100|epoch[|unit][|lat|lon]) and:
       - writes it to a daily CSV log
       - writes it to a queue file
       - attempts to POST all queued files with adaptive HTTP timeout
  5) Gracefully powers down the modem and deep-sleeps for 15 minutes.

PURPOSE OF THIS FILE:
  This build is specifically for verifying that the device is:
    - reading rainfall correctly (15-minute delta)
    - reading battery voltage correctly
    - building the compact payload correctly (NO commas, NO decimals)
    - only including lat/lon when GPS is refreshed (every 6 hours)
    - writing daily logs + queue files to SD
    - attempting modem power-down in software before deep sleep

KEY DIFFERENCE VS PRODUCTION BUILDS:
  - HTTP POST is enabled, but we log the payload and HTTP timing to Serial.
  - Queue files are still written so you can inspect /queue content on SD.

DATA STORAGE STRATEGY:
  - /logs/log_YYYYMMDD.csv        (daily permanent history, human readable)
  - /queue/q_epoch_wake.txt       (one per interval; deleted after successful upload)
  - /state/rain_prev_total_mm.txt (previous cumulative rainfall total)
  - /state/gps_last.txt           (last known epoch, lat, lon)
  - /state/gps_fix_ms.txt         (last successful GPS fix duration in ms)
  - /state/gps_retry_epoch.txt    (next epoch to retry GPS after a failure)
  - /state/http_last_ms.txt       (last HTTP send duration in ms)
  - /state/identity.txt           (cached ICCID, IMEI so identity survives power loss)
  - /state/queue_retry_*.txt      (retry counters for invalid payloads)

TIME STRATEGY:
  - g_epochEstimate stored in RTC memory survives deep sleep.
  - On each wake, if epoch exists, add 900 seconds (15 min).
  - Every 6 hours (or first boot), attempt GPS to re-anchor time and update location.
  - GPS timeout is adaptive: last fix time * 2 (default 10 minutes).
  - If GPS fails, schedule a retry on the next 15-minute wake.

POWER STRATEGY (SOFTWARE-ONLY):
  - WiFi/BT off, CPU 80MHz.
  - GNSS OFF when not needed.
  - Best-effort modem shutdown: +CFUN=0, +CPOF, PWRKEY pulse, FLIGHT HIGH, UART end.
  - True hardware power removal requires VBAT gating (not done here).

************************************************************************************/

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#include <Arduino.h>
#include <cstring>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include <WiFi.h>
#include <esp_bt.h>

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Rain sensor library (DFRobot tipping bucket)
#include "DFRobot_RainfallSensor.h"

#define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)
#define PAVEWISE_ENABLE_HTTP true
#include "utilities.h"

#if PAVEWISE_ENABLE_DEBUG
  #define DBG_BEGIN()       do{ Serial.begin(DEBUG_BAUD); delay(200);}while(0)
  #define DBG_PRINT(x)      Serial.print(x)
  #define DBG_PRINTLN(x)    Serial.println(x)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
  #define DBG_FLUSH()       Serial.flush()
#else
  #define DBG_BEGIN()       do{}while(0)
  #define DBG_PRINT(x)      do{}while(0)
  #define DBG_PRINTLN(x)    do{}while(0)
  #define DBG_PRINTF(...)   do{}while(0)
  #define DBG_FLUSH()       do{}while(0)
#endif

// ============================= RTC STATE =============================
// Survives deep sleep, NOT battery removal.

RTC_DATA_ATTR uint32_t g_wakeCounter   = 0;
RTC_DATA_ATTR uint32_t g_epochEstimate = 0;
RTC_DATA_ATTR uint32_t g_bootCounter   = 0;

// ============================= MODEM + SENSOR OBJECTS =============================

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsm(modem);
HttpClient http(gsm, SERVER_HOST, SERVER_PORT);

DFRobot_RainfallSensor_I2C RainSensor(&Wire);

// ============================= SMALL HELPERS =============================
// Small delay between queued HTTP sends to avoid back-to-back modem churn.
static const uint32_t HTTP_QUEUE_SEND_DELAY_MS = 1000;
// Maximum number of per-cycle retries for invalid payloads before deleting queue entry.
static const uint8_t HTTP_QUEUE_INVALID_RETRY_CYCLES = 10;

// Safe elapsed millis calculator (handles wrap naturally).
static uint32_t elapsedMs(uint32_t startMs) {
  return (uint32_t)(millis() - startMs);
}

static bool isValidImei(const String &imei) {
  if (imei.length() != 15) return false;
  for (size_t i = 0; i < imei.length(); i++) {
    if (!isDigit(imei[i])) return false;
  }
  return true;
}

static void drainModemSerial(uint32_t durationMs = 150) {
  uint32_t start = millis();
  while (elapsedMs(start) < durationMs) {
    while (SerialAT.available()) {
      SerialAT.read();
    }
    delay(10);
  }
}

static String readModemImei(uint8_t attempts = 3) {
  for (uint8_t i = 0; i < attempts; i++) {
    drainModemSerial();
    String imei = modem.getIMEI();
    imei.trim();
    if (isValidImei(imei)) {
      return imei;
    }
    if (imei.length()) {
      DBG_PRINTF("[MODEM] IMEI read invalid (attempt %u): %s\n", (unsigned)(i + 1), imei.c_str());
    }
    delay(200);
  }
  return String();
}

// Poll AT until timeout; returns true on first successful response.
static bool waitForAtResponsive(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (elapsedMs(start) < timeoutMs) {
    if (modem.testAT()) return true;
    delay(1000);
  }
  return false;
}

// Turn off radios we do not use (WiFi/BT).
static void disableRadiosForPower() {
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();
}

// Downclock CPU to reduce power.
static void setLowPowerCpu() {
  setCpuFrequencyMhz(80);
}

// Pulse the SIM7600 PWRKEY pin to toggle power state.
// Configure the SIM7600 control pins to match the proven power-up sequence.
static void ts7600PowerPinsSetup() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_FLIGHT, OUTPUT);
  pinMode(MODEM_DTR, OUTPUT);
  pinMode(MODEM_STATUS, INPUT);

  digitalWrite(MODEM_FLIGHT, HIGH);
  digitalWrite(MODEM_DTR, LOW);

  digitalWrite(MODEM_PWRKEY, LOW);
  delay(MODEM_PWRKEY_PREP_MS);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(MODEM_PWRKEY_PULSE_MS);
  digitalWrite(MODEM_PWRKEY, LOW);
}

// Helper to pulse PWRKEY when forcing power-down.
static void pulsePwrKey(uint32_t holdMs = MODEM_POWEROFF_PULSE_MS) {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(holdMs);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
}

// ============================= SD HELPERS =============================

// Mount SD and ensure directory structure exists.
// Mount SD and ensure /logs, /queue, /state exist.
static bool initSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) return false;

  if (!SD.exists(DIR_LOGS))  SD.mkdir(DIR_LOGS);
  if (!SD.exists(DIR_QUEUE)) SD.mkdir(DIR_QUEUE);
  if (!SD.exists(DIR_STATE)) SD.mkdir(DIR_STATE);

  return true;
}

// Compute % used for log purging.
static float sdUsedPercent() {
  uint64_t total = SD.totalBytes();
  uint64_t used  = SD.usedBytes();
  if (total == 0) return 0.0f;
  return 100.0f * (float)used / (float)total;
}

static bool isDailyLogName(const String &base) {
  if (base.length() != 16) return false;
  if (!base.startsWith("log_")) return false;
  if (!base.endsWith(".csv")) return false;
  for (int i = 4; i < 12; i++) if (base[i] < '0' || base[i] > '9') return false;
  return true;
}

// Find the oldest daily log by filename ordering.
static bool findOldestDailyLog(String &oldestPath) {
  File dir = SD.open(DIR_LOGS);
  if (!dir) return false;
  String best = "";
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      String full = String(f.name());
      String base = full.substring(full.lastIndexOf('/') + 1);
      if (isDailyLogName(base)) {
        if (best == "" || base < best) best = base;
      }
    }
    f.close();
  }
  dir.close();
  if (best == "") return false;
  oldestPath = String(DIR_LOGS) + "/" + best;
  return true;
}

// Purge logs when SD exceeds the high watermark, delete oldest until target met.
static void purgeLogsIfNeeded() {
  if (sdUsedPercent() < SD_PURGE_START_PCT) return;
  DBG_PRINTF("[SD] %.1f%% used -> purging...\n", sdUsedPercent());
  while (sdUsedPercent() > SD_PURGE_TARGET_PCT) {
    String oldest;
    if (!findOldestDailyLog(oldest)) break;
    DBG_PRINTF("[SD] Deleting: %s\n", oldest.c_str());
    SD.remove(oldest.c_str());
  }
  DBG_PRINTF("[SD] After purge: %.1f%% used\n", sdUsedPercent());
}

// Read a one-line text file and trim.
// Read one-line text file from SD (trimmed).
static bool readTextFile(const char *path, String &out) {
  if (!SD.exists(path)) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  out = f.readStringUntil('\n');
  f.close();
  out.trim();
  return out.length() > 0;
}

// Write a one-line text file (overwrite).
// Write one-line text file to SD (overwrite).
static bool writeTextFile(const char *path, const String &txt) {
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  f.seek(0);
  f.print(txt);
  f.print("\n");
  f.close();
  return true;
}

// Read float from file.
static bool readFloatFile(const char *path, float &out) {
  String s;
  if (!readTextFile(path, s)) return false;
  out = s.toFloat();
  return true;
}

// Write float to file (state storage only).
static bool writeFloatFile(const char *path, float v) {
  return writeTextFile(path, String(v, 3));
}

// Read uint32 from SD (stored as string).
static bool readUInt32File(const char *path, uint32_t &out) {
  String s;
  if (!readTextFile(path, s)) return false;
  out = (uint32_t)s.toInt();
  return true;
}

// Write uint32 to SD (stored as string).
static bool writeUInt32File(const char *path, uint32_t v) {
  return writeTextFile(path, String((unsigned long)v));
}

static bool readBoolFile(const char *path, bool &out) {
  String s;
  if (!readTextFile(path, s)) return false;
  s.trim();
  out = (s == "1" || s.equalsIgnoreCase("true"));
  return true;
}

static bool writeBoolFile(const char *path, bool v) {
  return writeTextFile(path, v ? "1" : "0");
}

static bool payloadHasUnitToken(const String &payload) {
  int count = 0;
  for (size_t i = 0; i < payload.length(); i++) {
    if (payload[i] == '|') count++;
  }
  return (count == 4 || count == 6);
}

// Load cached identity: "iccid,imei"
static bool loadIdentity(String &iccid, String &imei) {
  String line;
  if (!readTextFile(FILE_IDENTITY, line)) return false;
  int c = line.indexOf(',');
  if (c < 0) return false;

  iccid = line.substring(0, c); iccid.trim();
  imei  = line.substring(c + 1); imei.trim();
  return (iccid.length() > 0 && isValidImei(imei));
}

// Save identity cache.
static void saveIdentity(const String &iccid, const String &imei) {
  if (iccid.length() == 0 || !isValidImei(imei)) return;
  writeTextFile(FILE_IDENTITY, iccid + "," + imei);
}

// Load last GPS state: "epoch,lat,lon"
static bool loadLastGps(uint32_t &epoch, double &lat, double &lon) {
  String line;
  if (!readTextFile(FILE_GPS_LAST, line)) return false;

  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  if (p1 < 0 || p2 < 0) return false;

  epoch = (uint32_t)line.substring(0, p1).toInt();
  lat   = line.substring(p1 + 1, p2).toDouble();
  lon   = line.substring(p2 + 1).toDouble();

  return epoch > 0;
}

// Save last GPS state.
static void saveLastGps(uint32_t epoch, double lat, double lon) {
  if (epoch == 0) return;
  char buf[96];
  snprintf(buf, sizeof(buf), "%lu,%.7f,%.7f",
           (unsigned long)epoch, lat, lon);
  writeTextFile(FILE_GPS_LAST, String(buf));
}

// ============================= BATTERY MEASUREMENT =============================
// Battery voltage calibration is board-specific.
// This is a starting point; you should calibrate VBAT_DIVIDER and CAL_MULT against multimeter.

static const float VBAT_DIVIDER    = 2.0f;    // adjust if schematic divider ratio differs
static const float VBAT_CAL_MULT   = 1.00f;   // multiply correction
static const float VBAT_CAL_OFFSET = 0.00f;   // offset correction (volts)

static float readBatteryVoltage(uint32_t &rawMvAtAdcPin) {
  // 11 dB attenuation allows measuring higher voltages at ADC input.
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  // This returns millivolts at the ADC pin (already calibrated by ESP32 ADC code).
  rawMvAtAdcPin = analogReadMilliVolts(BAT_ADC_PIN);

  // Convert to VBAT by applying divider ratio, then calibration adjustments.
  float v = (rawMvAtAdcPin / 1000.0f) * VBAT_DIVIDER;
  v = v * VBAT_CAL_MULT + VBAT_CAL_OFFSET;
  return v;
}

// ============================= DATE/TIME HELPERS =============================
// Used only for naming daily logs based on epoch date.

static bool isLeap(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

static uint8_t dim(int y, int m) {
  static const uint8_t d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  return (m == 2 && isLeap(y)) ? 29 : d[m - 1];
}

static uint32_t ymdhmsToEpoch(int y, int mo, int da, int h, int mi, int s) {
  uint32_t days = 0;
  for (int yr = 1970; yr < y; yr++) days += isLeap(yr) ? 366 : 365;
  for (int m = 1; m < mo; m++) days += dim(y, m);
  days += (da - 1);
  return days * 86400UL + (uint32_t)h * 3600UL + (uint32_t)mi * 60UL + (uint32_t)s;
}

static void epochToYMD(uint32_t e, int &y, int &m, int &d) {
  uint32_t days = e / 86400UL;
  y = 1970;

  while (true) {
    uint32_t dy = isLeap(y) ? 366 : 365;
    if (days >= dy) { days -= dy; y++; } else break;
  }

  m = 1;
  while (true) {
    uint8_t dm = dim(y, m);
    if (days >= dm) { days -= dm; m++; } else break;
  }

  d = (int)days + 1;
}

static String dailyLogPath(uint32_t epoch) {
  int y, m, d;
  epochToYMD(epoch, y, m, d);
  char buf[48];
  snprintf(buf, sizeof(buf), "%s/log_%04d%02d%02d.csv", DIR_LOGS, y, m, d);
  return String(buf);
}

// ============================= PAYLOAD COMPRESSION =============================
//
// Compact payload uses '|' delimiter and scaled integers (no commas, no decimals).
//
// Base record (always):
//   I|B|R|E
//
// Optional one-time unit token (first successful upload only):
//   |U
//
// If GPS refreshed this wake, append:
//   |LA|LO
//
// Scale rules (server decode):
//   R  = round(rain_mm * 100)             => rain_mm = R / 100.0
//   B  = battery millivolts (integer)     => batt_v  = B / 1000.0
//   LA = round(lat_deg * 1e7)             => lat     = LA / 1e7
//   LO = round(lon_deg * 1e7)             => lon     = LO / 1e7
//
static const char *rainUnitToken() { return RAIN_UNIT; }
static int32_t scaleRain100(float value) {
  return (int32_t)lroundf(value * 100.0f);
}

static int32_t scaleDeg1e7(double deg) {
  return (int32_t)llround(deg * 10000000.0);
}

static String buildUploadPayload(const String &imei,
                                 uint32_t battMv,
                                 float rainDeltaMm,
                                 uint32_t epochNow,
                                 bool includeUnit,
                                 bool includeGps,
                                 double lat,
                                 double lon) {
  int32_t R = scaleRain100(rainDeltaMm);

  String s;
  s.reserve(includeGps ? 120 : 90);

  s += imei;
  s += "|";
  s += String((unsigned long)battMv);
  s += "|";
  s += String(R);
  s += "|";
  s += String((unsigned long)epochNow);
  if (includeUnit) {
    s += "|";
    s += rainUnitToken();
  }

  if (includeGps) {
    s += "|";
    s += String(scaleDeg1e7(lat));
    s += "|";
    s += String(scaleDeg1e7(lon));
  }

  return s;
}

// ============================= MODEM CONTROL =============================

// Power modem on and wait for AT response.
static bool modemPowerOn() {
  DBG_PRINTLN("[MODEM] Powering ON sequence...");
  ts7600PowerPinsSetup();

  int st = digitalRead(MODEM_STATUS);
  DBG_PRINTF("[MODEM] MODEM_STATUS after PWRKEY kick: %d (may be unwired)\n", st);

  DBG_PRINTLN("[MODEM] Waiting ~15s for modem boot...");
  delay(MODEM_BOOT_WAIT_MS);

  DBG_PRINTLN("[MODEM] Starting UART...");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(200);

  DBG_PRINTLN("[MODEM] Testing AT responsiveness (up to ~10s)...");
  if (waitForAtResponsive(10000)) {
    DBG_PRINTLN("[MODEM] AT OK");
    return true;
  }

  DBG_PRINTLN("[MODEM] AT timeout, retrying power cycle...");

  // Force modem OFF, then ON again, to avoid a stuck power state.
  pulsePwrKey();
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(MODEM_PWRKEY_PREP_MS);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(MODEM_PWRKEY_PULSE_MS);
  digitalWrite(MODEM_PWRKEY, LOW);

  DBG_PRINTLN("[MODEM] Waiting ~15s for modem boot (retry)...");
  delay(MODEM_BOOT_WAIT_MS);

  DBG_PRINTLN("[MODEM] Testing AT responsiveness (retry, up to ~10s)...");
  if (waitForAtResponsive(10000)) {
    DBG_PRINTLN("[MODEM] AT OK (retry)");
    return true;
  }

  DBG_PRINTLN("[MODEM] AT timeout");
  return false;
}

// Best-effort modem shutdown (software-only).
static void modemBestEffortPowerDown() {
  DBG_PRINTLN("[MODEM] power down (software-only) ...");

  // Ensure GNSS engine is off.
  modem.sendAT("+CGPS=0");
  modem.waitResponse(1000);

  // Minimum functionality (often disables RF stack).
  modem.sendAT("+CFUN=0");
  modem.waitResponse(2000);

  // Request power off.
  modem.sendAT("+CPOF");
  modem.waitResponse(3000);

  // Configure sleep and assert DTR high in case modem ignores CPOF.
  modem.sendAT("+CSCLK=2");
  modem.waitResponse(1000);
  digitalWrite(MODEM_DTR, HIGH);

  // Extra PWRKEY pulse to force shutdown.
  pulsePwrKey();

  // Stop UART to avoid phantom powering.
  SerialAT.end();

  // Force FLIGHT HIGH (RF disabled) if modem core still partially alive.
  digitalWrite(MODEM_FLIGHT, HIGH);

  DBG_PRINTLN("[MODEM] power down done");
}

// Connect network and PDP context; also read ICCID/IMEI.
static bool connectNetwork(String &iccid, String &imei, int &rssi) {
  rssi = -1;

  if (!modem.init()) {
    DBG_PRINTLN("[MODEM] modem.init failed");
    return false;
  }

#if defined(TINY_GSM_MODEM_HAS_GPS)
  DBG_PRINTLN("[GPS] Enabling GNSS...");
  modem.setGPSMode(1);
  delay(200);
#endif

  String tIccid = modem.getSimCCID();
  String tImei  = readModemImei();

  if (tIccid.length()) iccid = tIccid;
  if (tImei.length() && isValidImei(tImei)) {
    imei = tImei;
  } else if (tImei.length()) {
    DBG_PRINTF("[MODEM] IMEI read invalid: %s\n", tImei.c_str());
  }

  DBG_PRINTF("[MODEM] ICCID: %s\n", iccid.c_str());
  DBG_PRINTF("[MODEM] IMEI : %s\n", imei.c_str());

  DBG_PRINT("[MODEM] Modem Info: ");
  DBG_PRINTLN(modem.getModemInfo());

  DBG_PRINTLN("[NET] Waiting for network registration (up to 60s)...");
  uint32_t start = millis();
  if (!modem.waitForNetwork(60000L)) {
    uint32_t elapsed = elapsedMs(start);
    DBG_PRINTF("[NET] waitForNetwork timeout (%.1f s)\n", elapsed / 1000.0f);
    return false;
  }
  uint32_t netMs = elapsedMs(start);
  DBG_PRINTF("[NET] Registered in %lu ms (%.1f s)\n",
                (unsigned long)netMs, netMs / 1000.0f);

  int16_t csq = modem.getSignalQuality();
  DBG_PRINTF("[NET] RSSI (CSQ): %d\n", (int)csq);

  String op = modem.getOperator();
  DBG_PRINT("[NET] Operator : ");
  DBG_PRINTLN(op.length() ? op : "(unknown)");

  DBG_PRINT("[NET] Local IP : ");
  DBG_PRINTLN(modem.localIP());

  DBG_PRINTLN("[NET] Connecting GPRS...");
  start = millis();
  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    uint32_t elapsed = elapsedMs(start);
    DBG_PRINTF("[NET] gprsConnect failed (%.1f s)\n", elapsed / 1000.0f);
    return false;
  }
  uint32_t gprsMs = elapsedMs(start);
  DBG_PRINTF("[NET] GPRS OK in %lu ms (%.1f s)\n",
                (unsigned long)gprsMs, gprsMs / 1000.0f);

  rssi = modem.getSignalQuality();
  DBG_PRINTF("[NET] RSSI (CSQ): %d\n", rssi);

  DBG_PRINT("[NET] Local IP : ");
  DBG_PRINTLN(modem.localIP());

  return true;
}

// ============================= GPS PARSING =============================
//
// IMPORTANT FIX:
//   We DO NOT use a lambda here (auto nmeaToDeg=...)
//   because that can sometimes break Arduino parsing/highlighting.
//   Instead we use a normal helper function.

// Convert NMEA ddmm.mmmm or dddmm.mmmm into decimal degrees.
static double nmeaDdmmToDeg(const String &ddmm) {
  int dot = ddmm.indexOf('.');
  int degLen = (dot >= 0 && dot > 4) ? 3 : 2;     // lon often has 3 deg digits
  double deg = ddmm.substring(0, degLen).toDouble();
  double min = ddmm.substring(degLen).toDouble();
  return deg + (min / 60.0);
}

// Parse a +CGPSINFO response into lat/lon and possibly epoch.
static bool parseCgpsInfo(const String &resp,
                          double &latOut, double &lonOut,
                          bool &hasEpoch, uint32_t &epochOut) {
  // Extract payload after ":" if present.
  int idx = resp.indexOf(":");
  String p = (idx >= 0) ? resp.substring(idx + 1) : resp;
  p.trim();

  // When no fix, SIMCOM often returns ",,,,,,"
  if (p.startsWith(",")) return false;

  // Split by commas.
  std::vector<String> parts;
  parts.reserve(10);

  int start = 0;
  for (int i = 0; i <= (int)p.length(); i++) {
    if (i == (int)p.length() || p[i] == ',') {
      parts.push_back(p.substring(start, i));
      start = i + 1;
    }
  }

  // Need at least: lat, N/S, lon, E/W
  if (parts.size() < 4) return false;

  String latStr = parts[0]; latStr.trim();
  String ns     = parts[1]; ns.trim();
  String lonStr = parts[2]; lonStr.trim();
  String ew     = parts[3]; ew.trim();

  if (latStr.length() < 3 || lonStr.length() < 4) return false;

  // Convert NMEA ddmm.mmmm -> decimal degrees.
  double lat = nmeaDdmmToDeg(latStr);
  double lon = nmeaDdmmToDeg(lonStr);

  if (ns == "S") lat = -lat;
  if (ew == "W") lon = -lon;

  latOut = lat;
  lonOut = lon;

  // Default to no epoch unless we parse date/time.
  hasEpoch = false;
  epochOut = 0;

  // If available: ddmmyy and hhmmss.s
  if (parts.size() >= 6) {
    String date = parts[4]; date.trim();
    String time = parts[5]; time.trim();

    if (date.length() >= 6 && time.length() >= 6) {
      int dd = date.substring(0, 2).toInt();
      int mo = date.substring(2, 4).toInt();
      int yy = date.substring(4, 6).toInt();
      int year = 2000 + yy;

      int hh = time.substring(0, 2).toInt();
      int mm = time.substring(2, 4).toInt();
      int ss = time.substring(4, 6).toInt();

      if (year >= 2020 && mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31 &&
          hh <= 23 && mm <= 59 && ss <= 59) {
        epochOut = ymdhmsToEpoch(year, mo, dd, hh, mm, ss);
        hasEpoch = (epochOut > 0);
      }
    }
  }

  return true;
}

// Attempt GPS fix; returns true when fix obtained (lat/lon).
static bool gpsAcquire(uint32_t timeoutMs,
                       double &latOut, double &lonOut,
                       bool &hasEpoch, uint32_t &epochOut,
                       uint32_t &fixTimeMsOut) {
  hasEpoch = false;
  epochOut = 0;

  // Turn GNSS ON.
  modem.sendAT("+CGPS=1");
  uint32_t start = millis();
  if (modem.waitResponse(2000) != 1) {
    fixTimeMsOut = elapsedMs(start);
    return false;
  }

  uint32_t lastProgress = 0;
  while (elapsedMs(start) < timeoutMs) {
    // Ask for GPS info.
    modem.sendAT("+CGPSINFO");

    String out;
    int r = modem.waitResponse(1200, out);

    // If we got a response containing +CGPSINFO, attempt parse.
    if (r == 1 && out.indexOf("+CGPSINFO") >= 0) {
      if (parseCgpsInfo(out, latOut, lonOut, hasEpoch, epochOut)) {
        // Turn GNSS OFF immediately after fix to save power.
        modem.sendAT("+CGPS=0");
        modem.waitResponse(1000);
        fixTimeMsOut = elapsedMs(start);
        return true;
      }
    }

    if (elapsedMs(start) - lastProgress >= 5000) {
      lastProgress = elapsedMs(start);
      DBG_PRINTF("[GPS] searching... t=%.1f s\n", lastProgress / 1000.0f);
    }

    delay(2000);
  }

  // Timeout: ensure GNSS OFF.
  modem.sendAT("+CGPS=0");
  modem.waitResponse(1000);
  fixTimeMsOut = elapsedMs(start);
  return false;
}

// ============================= LOGGING / QUEUE =============================

// Append line to daily CSV log; create header if file new.
static void appendDaily(const String &path, const String &line) {
  bool isNew = !SD.exists(path);
  File f = SD.open(path, FILE_APPEND);
  if (!f) return;

  if (isNew) {
    f.println("payload");
  }

  f.println(line);
  f.close();
}

// Queue file name: q_epoch_wake.txt
static String makeQueueFilename(uint32_t epoch, uint32_t wakeCounter) {
  char buf[80];
  snprintf(buf, sizeof(buf), "%s/q_%lu_%lu.txt",
           DIR_QUEUE, (unsigned long)epoch, (unsigned long)wakeCounter);
  return String(buf);
}

// Write a queue file (one payload line).
static void writeQueueLine(const String &path, const String &line) {
  File f = SD.open(path, FILE_WRITE);
  if (!f) return;
  f.seek(0);
  f.println(line);
  f.close();
}

static uint32_t computeHttpTimeoutMs(uint32_t lastHttpMs) {
  if (lastHttpMs == 0) return HTTP_TIMEOUT_DEFAULT_MS;
  uint64_t timeout = (uint64_t)lastHttpMs * HTTP_TIMEOUT_MULTIPLIER;
  if (timeout > HTTP_TIMEOUT_MAX_MS) timeout = HTTP_TIMEOUT_MAX_MS;
  return (uint32_t)timeout;
}

static String buildUnitPayload(const String &imei) {
  String s;
  s.reserve(32);
  s += imei;
  s += "|";
  s += rainUnitToken();
  return s;
}

static String extractImeiFromPayload(const String &payload) {
  int sep = payload.indexOf('|');
  if (sep <= 0) return String();
  return payload.substring(0, sep);
}

static String queueRetryStatePath(const String &queuePath) {
  String base = queuePath;
  int slash = base.lastIndexOf('/');
  if (slash >= 0) {
    base = base.substring(slash + 1);
  }
  return String(DIR_STATE) + "/queue_retry_" + base;
}

static uint32_t readQueueRetryCount(const String &queuePath) {
  String statePath = queueRetryStatePath(queuePath);
  uint32_t count = 0;
  readUInt32File(statePath.c_str(), count);
  return count;
}

static uint32_t incrementQueueRetryCount(const String &queuePath) {
  String statePath = queueRetryStatePath(queuePath);
  uint32_t count = readQueueRetryCount(queuePath);
  count += 1;
  writeUInt32File(statePath.c_str(), count);
  return count;
}

static void clearQueueRetryState(const String &queuePath) {
  String statePath = queueRetryStatePath(queuePath);
  if (SD.exists(statePath.c_str())) {
    SD.remove(statePath.c_str());
  }
}

// Normalize queue paths so files always resolve under DIR_QUEUE.
static String ensureQueuePath(const String &name) {
  if (name.startsWith("/")) return name;
  return String(DIR_QUEUE) + "/" + name;
}

// Serial-only queue preload test guard (one-time per SD card).
static const char *FILE_QUEUE_PRELOAD_DONE = "/state/queue_preload_done.txt";
static const char *FILE_BOOT_COUNTER = PAVEWISE_FILE_BOOT_COUNTER;

// Optionally preload synthetic payloads into /queue to validate queue sends.
static void preloadQueueTestPayloads(uint32_t epochNow, const String &imei, uint32_t battMvVBAT) {
  if (!QUEUE_PRELOAD_TEST_ENABLED) return;
  if (SD.exists(FILE_QUEUE_PRELOAD_DONE)) return;
  if (epochNow == 0) epochNow = g_wakeCounter;

  DBG_PRINTF("[QUEUE] preload test enabled: adding %u payloads\n",
                (unsigned)QUEUE_PRELOAD_COUNT);
  for (uint8_t i = 0; i < QUEUE_PRELOAD_COUNT; i++) {
    uint32_t fakeEpoch = epochNow - (uint32_t)(i + 1) * 60UL;
    String payload = buildUploadPayload(
      imei,
      battMvVBAT,
      0.0f,
      fakeEpoch,
      false,
      false,
      0.0,
      0.0
    );
    String qPath = makeQueueFilename(fakeEpoch, g_wakeCounter + i + 1);
    writeQueueLine(qPath, payload);
  }
  writeTextFile(FILE_QUEUE_PRELOAD_DONE, "1");
}

// Send HTTP payload and measure time spent (used to adapt next timeout).
static bool httpPostPayload(const String &line, uint32_t timeoutMs, uint32_t &durationMs, int &statusOut, String &responseOut) {
  http.setHttpResponseTimeout(timeoutMs);
  DBG_PRINTF("[HTTP] Attempting POST http://%s:%d%s\n", SERVER_HOST, SERVER_PORT, SERVER_PATH);
  DBG_PRINTF("[HTTP] Payload length: %u bytes\n", (unsigned)line.length());
  uint32_t start = millis();
  http.beginRequest();
  http.post(SERVER_PATH);
  http.sendHeader("Content-Type", "text/plain");
  http.sendHeader("Content-Length", line.length());
  http.beginBody();
  http.print(line);
  DBG_PRINTLN("[HTTP] Payload sent, awaiting response...");
  http.endRequest();
  statusOut = http.responseStatusCode();
  responseOut = http.responseBody();
  durationMs = elapsedMs(start);
  http.stop();
  if (durationMs > timeoutMs) return false;
  return (statusOut >= 200 && statusOut < 300);
}

static bool responseIndicatesInvalidPayload(int status, const String &response) {
  return status == 400 && response.indexOf("\"code\":\"invalid_payload\"") >= 0;
}

// Send a single queued file (delete only on success).
static bool sendQueueFile(const String &path, uint32_t timeoutMs, uint32_t &durationMs, bool &unitSent) {
  File f = SD.open(path, FILE_READ);
  if (!f) {
    DBG_PRINTF("[QUEUE] FAILED to open %s\n", path.c_str());
    return false;
  }
  String line = f.readStringUntil('\n');
  size_t fileSize = f.size();
  f.close();
  DBG_PRINTF("[QUEUE] sending %s (%u bytes on disk)\n", path.c_str(), (unsigned)fileSize);
  int status = 0;
  String response;
  bool ok = httpPostPayload(line, timeoutMs, durationMs, status, response);
  bool stored = response.indexOf("stored") >= 0;
  bool unitRequired = response.indexOf("unit_required") >= 0;
  bool invalidPayload = responseIndicatesInvalidPayload(status, response);
  DBG_PRINTF("[HTTP] %s status=%d duration=%lu ms timeout=%lu ms\n",
                ok ? "OK" : "FAIL",
                status,
                (unsigned long)durationMs,
                (unsigned long)timeoutMs);
  if (response.length() > 0) {
    DBG_PRINTF("[HTTP] response=%s\n", response.c_str());
    if (stored) {
      DBG_PRINTLN("[HTTP] Server response indicates: stored");
    }
  } else {
    DBG_PRINTLN("[HTTP] response empty");
  }
  if (!stored && invalidPayload) {
    DBG_PRINTLN("[QUEUE] invalid payload response; retrying once");
    uint32_t retryDuration = 0;
    int retryStatus = 0;
    String retryResponse;
    bool retryOk = httpPostPayload(line, timeoutMs, retryDuration, retryStatus, retryResponse);
    bool retryStored = retryResponse.indexOf("stored") >= 0;
    bool retryInvalidPayload = responseIndicatesInvalidPayload(retryStatus, retryResponse);
    durationMs = retryDuration;
    status = retryStatus;
    response = retryResponse;
    ok = retryOk;
    stored = retryStored;
    invalidPayload = retryInvalidPayload;
  }
  if (!stored && unitRequired && !invalidPayload) {
    String imei = extractImeiFromPayload(line);
    if (imei.length()) {
      String unitPayload = buildUnitPayload(imei);
      uint32_t unitDuration = 0;
      int unitStatus = 0;
      String unitResponse;
      bool unitOk = httpPostPayload(unitPayload, timeoutMs, unitDuration, unitStatus, unitResponse);
      bool unitStored = unitResponse.indexOf("stored") >= 0;
      if (unitOk && unitStored) {
        unitSent = true;
        writeBoolFile(FILE_RAIN_UNIT_SENT, true);
        uint32_t retryDuration = 0;
        int retryStatus = 0;
        String retryResponse;
        bool retryOk = httpPostPayload(line, timeoutMs, retryDuration, retryStatus, retryResponse);
        bool retryStored = retryResponse.indexOf("stored") >= 0;
        durationMs = retryDuration;
        if (retryOk && retryStored) {
          SD.remove(path.c_str());
          clearQueueRetryState(path);
          return true;
        }
      }
    }
  }
  if (ok && stored) {
    if (!unitSent && payloadHasUnitToken(line)) {
      unitSent = true;
      writeBoolFile(FILE_RAIN_UNIT_SENT, true);
    }
    SD.remove(path.c_str());
    clearQueueRetryState(path);
  } else if (invalidPayload) {
    uint32_t retryCount = incrementQueueRetryCount(path);
    DBG_PRINTF("[QUEUE] invalid payload retry %lu/%u\n",
               (unsigned long)retryCount,
               HTTP_QUEUE_INVALID_RETRY_CYCLES);
    if (retryCount >= HTTP_QUEUE_INVALID_RETRY_CYCLES) {
      DBG_PRINTF("[QUEUE] dropping invalid payload after %lu cycles: %s\n",
                 (unsigned long)retryCount, path.c_str());
      SD.remove(path.c_str());
      clearQueueRetryState(path);
      return true;
    }
  } else if (ok && !stored) {
    DBG_PRINTLN("[QUEUE] skipping delete: missing stored ack");
  }
  return ok;
}

// Walk the queue folder in order and attempt to send each payload.
static void sendAllQueuedFiles(uint32_t &lastHttpMs, bool &unitSent) {
  std::vector<String> files;
  File dir = SD.open(DIR_QUEUE);
  if (!dir) return;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      files.push_back(ensureQueuePath(String(f.name())));
    }
    f.close();
  }
  dir.close();
  DBG_PRINTF("[QUEUE] %u payload(s) queued\n", (unsigned)files.size());
  std::sort(files.begin(), files.end(), [](const String &a, const String &b) { return a < b; });
  for (auto &p : files) {
    uint32_t durationMs = 0;
    uint32_t timeoutMs = computeHttpTimeoutMs(lastHttpMs);
    if (!sendQueueFile(p, timeoutMs, durationMs, unitSent)) {
      lastHttpMs = HTTP_TIMEOUT_MAX_MS;
      writeUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
      break;
    }
    lastHttpMs = durationMs;
    writeUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
    delay(HTTP_QUEUE_SEND_DELAY_MS);
  }
}

// ============================= RAINFALL =============================

static float readRainTotalMm() {
  return RainSensor.getRainfall();
}

static float rainDeltaInterval(float totalMm) {
  float prev = totalMm;
  // If file missing (first boot), prev defaults to total (delta=0).
  readFloatFile(FILE_RAIN_PREV_TOTAL, prev);

  float d = totalMm - prev;
  if (d < 0) d = 0;

  writeFloatFile(FILE_RAIN_PREV_TOTAL, totalMm);
  return d;
}

// ============================= MAIN =============================

void setup() {
  DBG_BEGIN();

  // Power reductions immediately.
  setLowPowerCpu();
  disableRadiosForPower();

  // Wake counter in RTC memory.
  g_wakeCounter++;

  DBG_PRINTLN("");
  DBG_PRINTLN("==================================================");
  DBG_PRINTF("[BOOT] Wake #%lu start\n", (unsigned long)g_wakeCounter);
  DBG_PRINTLN("==================================================");

  // If we already have epoch estimate, step forward by 15 minutes each wake.
  if (g_epochEstimate > 0) {
    g_epochEstimate += WAKE_INTERVAL_SECONDS;
  }

  // Mount SD (critical for logging/queue). If fails, sleep.
  bool sdOk = initSD();
  DBG_PRINTF("[SD] init: %s\n", sdOk ? "OK" : "FAIL");
  if (!sdOk) {
    DBG_PRINTLN("[FATAL] SD init failed -> sleep");
    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS * 1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }
  purgeLogsIfNeeded();

  // Detect cold boot (power loss) by comparing RTC boot counter with SD state.
  uint32_t bootCounterStored = 0;
  readUInt32File(FILE_BOOT_COUNTER, bootCounterStored);
  bool coldBoot = (g_bootCounter == 0 || g_bootCounter != bootCounterStored);
  if (coldBoot) {
    bootCounterStored = bootCounterStored + 1;
    writeUInt32File(FILE_BOOT_COUNTER, bootCounterStored);
    g_bootCounter = bootCounterStored;
  }
  DBG_PRINTF("[BOOT] cold boot detected: %s\n", coldBoot ? "YES" : "NO");

  // Load cached identity so we can still build payload even if modem fails.
  String iccid = "unknown";
  String imei  = "unknown";
  loadIdentity(iccid, imei);

  // Load last GPS state (survives power loss).
  uint32_t lastGpsEpoch = 0;
  double lastLat = 0.0, lastLon = 0.0;
  loadLastGps(lastGpsEpoch, lastLat, lastLon);
  uint32_t lastGpsFixMs = 0;
  readUInt32File(FILE_GPS_FIX_MS, lastGpsFixMs);
  uint32_t gpsRetryEpoch = 0;
  readUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);
  uint32_t lastHttpMs = 0;
  readUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
  bool unitSent = false;
  readBoolFile(FILE_RAIN_UNIT_SENT, unitSent);

  // If RTC epoch estimate was lost but SD has GPS epoch, restore it.
  if (g_epochEstimate == 0 && lastGpsEpoch > 0) {
    g_epochEstimate = lastGpsEpoch;
    DBG_PRINTF("[STATE] restored epoch from SD: %lu\n", (unsigned long)g_epochEstimate);
  }

  // Rain sensor init (I2C).
  Wire.begin();
  bool rainOk = RainSensor.begin();
  DBG_PRINTF("[RAIN] begin(): %s\n", rainOk ? "OK" : "FAIL");

  // Read rainfall totals and compute delta for this wake interval.
  float totalMm = readRainTotalMm();
  float deltaMm = rainDeltaInterval(totalMm);

  DBG_PRINTF("[RAIN] total=%.3f mm  delta15m=%.3f mm\n", totalMm, deltaMm);

  // Read battery voltage (scaled to VBAT).
  uint32_t battMvRaw = 0;
  float battV = readBatteryVoltage(battMvRaw);
  uint32_t battMvVBAT = (uint32_t)lroundf(battV * 1000.0f);

  DBG_PRINTF("[BATT] raw=%lu mV, scaled=%.3f V (~%lu mV)\n",
             (unsigned long)battMvRaw, battV, (unsigned long)battMvVBAT);

  // Power modem on (SIM7600 PWRKEY + UART + AT test loop).
  bool modemOk = modemPowerOn();
  DBG_PRINTF("[MODEM] Overall modem connect: %s\n", modemOk ? "OK" : "FAIL");

  // Network connect (only if modemOk), and cache ICCID/IMEI to SD.
  int rssi = -1;
  bool netOk = false;
  if (modemOk) {
    netOk = connectNetwork(iccid, imei, rssi);
    DBG_PRINTF("[NET] Registered: %s (RSSI=%d)\n", netOk ? "YES" : "NO", rssi);

    // Save identity so power loss still has ID next boot.
    saveIdentity(iccid, imei);
  }

  // Determine our best epoch for this wake (RTC estimate or last GPS).
  uint32_t epochNow = g_epochEstimate;
  if (epochNow == 0 && lastGpsEpoch > 0) epochNow = lastGpsEpoch;

  bool haveValidEpoch = (epochNow > 0);

  // Decide if GPS refresh is due:
  //  - if no valid epoch
  //  - or no last gps epoch
  //  - or older than 6 hours
  //  - or a retry interval was scheduled
  bool gpsRetryDue = (gpsRetryEpoch > 0) && (epochNow >= gpsRetryEpoch);
  bool needGps = coldBoot ||
                 (!haveValidEpoch) ||
                 (lastGpsEpoch == 0) ||
                 (epochNow >= lastGpsEpoch && (epochNow - lastGpsEpoch) >= GPS_REFRESH_SECONDS) ||
                 gpsRetryDue ||
                 (g_wakeCounter == 1);

  bool gpsIncludedInUpload = false;

  // GPS fix attempt only if modem is up AND GPS is due.
  if (modemOk && needGps) {
    // Adaptive GPS timeout with a minimum floor so we always search >= 3 minutes.
    uint32_t gpsTimeoutMs = lastGpsFixMs > 0 ? (lastGpsFixMs * 2UL) : GPS_TIMEOUT_DEFAULT_MS;
    if (gpsTimeoutMs < GPS_TIMEOUT_MIN_MS) gpsTimeoutMs = GPS_TIMEOUT_MIN_MS;
    DBG_PRINTF("[GPS] refresh due; attempting fix (timeout %.1f min)...\n",
                  gpsTimeoutMs / 60000.0f);

    double lat = 0.0, lon = 0.0;
    bool hasGpsEpoch = false;
    uint32_t gpsEpoch = 0;
    uint32_t fixTimeMs = 0;

    bool gpsOk = gpsAcquire(gpsTimeoutMs, lat, lon, hasGpsEpoch, gpsEpoch, fixTimeMs);

    if (gpsOk) {
      lastLat = lat;
      lastLon = lon;
      lastGpsFixMs = fixTimeMs;
      writeUInt32File(FILE_GPS_FIX_MS, lastGpsFixMs);
      gpsRetryEpoch = 0;
      writeUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);

      if (hasGpsEpoch) {
        lastGpsEpoch = gpsEpoch;
        g_epochEstimate = gpsEpoch;
        epochNow = gpsEpoch;

        DBG_PRINTF("[GPS] FIX OK: lat=%.7f lon=%.7f epoch=%lu (%.1f s)\n",
                      lastLat, lastLon, (unsigned long)gpsEpoch, fixTimeMs / 1000.0f);
      } else {
        // If GPS time not parseable, still store location and keep existing epoch estimate.
        if (epochNow == 0) epochNow = (uint32_t)(millis() / 1000UL);
        lastGpsEpoch = epochNow;
        if (g_epochEstimate == 0) g_epochEstimate = epochNow;

        DBG_PRINTF("[GPS] FIX OK: lat=%.7f lon=%.7f (no epoch, %.1f s)\n",
                      lastLat, lastLon, fixTimeMs / 1000.0f);
      }

      saveLastGps(lastGpsEpoch, lastLat, lastLon);
      gpsIncludedInUpload = true;  // ONLY TRUE when we refreshed GPS this wake
    } else {
      uint32_t baseEpoch = epochNow;
      if (baseEpoch == 0 && g_epochEstimate > 0) baseEpoch = g_epochEstimate;
      if (baseEpoch == 0) baseEpoch = (uint32_t)(millis() / 1000UL);
      lastGpsFixMs = 0;
      writeUInt32File(FILE_GPS_FIX_MS, lastGpsFixMs);
      gpsRetryEpoch = baseEpoch + GPS_RETRY_SECONDS;
      writeUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);
      DBG_PRINTF("[GPS] FIX FAIL (retry scheduled at epoch %lu)\n",
                    (unsigned long)gpsRetryEpoch);
    }
  } else if (needGps && !modemOk) {
    uint32_t baseEpoch = epochNow;
    if (baseEpoch == 0 && g_epochEstimate > 0) baseEpoch = g_epochEstimate;
    if (baseEpoch == 0) baseEpoch = (uint32_t)(millis() / 1000UL);
    lastGpsFixMs = 0;
    writeUInt32File(FILE_GPS_FIX_MS, lastGpsFixMs);
    gpsRetryEpoch = baseEpoch + GPS_RETRY_SECONDS;
    writeUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);
    DBG_PRINTF("[GPS] retry scheduled at epoch %lu (modem not OK)\n",
                  (unsigned long)gpsRetryEpoch);
  } else {
    DBG_PRINTLN("[GPS] not due");
  }

  // If epoch still zero, fall back to lastGpsEpoch.
  if (epochNow == 0) epochNow = lastGpsEpoch;

  // COMPACT PAYLOAD (no commas, no decimals).
  String payload = buildUploadPayload(
    imei,
    battMvVBAT,
    deltaMm,
    epochNow,
    !unitSent,
    gpsIncludedInUpload,   // only include lat/lon when GPS refreshed
    lastLat,
    lastLon
  );

  String logPath = dailyLogPath(epochNow ? epochNow : 0);
  appendDaily(logPath, payload);
  DBG_PRINTF("[LOG] %s\n", logPath.c_str());
  DBG_PRINTF("[LOG] %s\n", payload.c_str());

  // Print the payload instead of sending it.
  DBG_PRINTLN("[PAYLOAD] built:");
  DBG_PRINTLN(payload);

  // Queue record still written so you can inspect /queue output.
  String qPath = makeQueueFilename(epochNow ? epochNow : g_wakeCounter, g_wakeCounter);
  writeQueueLine(qPath, payload);
  DBG_PRINTF("[QUEUE] wrote %s\n", qPath.c_str());

  // Optional queue preload test before sending.
  preloadQueueTestPayloads(epochNow, imei, battMvVBAT);

  // Attempt HTTP sending for queued files (if enabled and network OK).
  if (ENABLE_HTTP) {
    if (netOk) {
      uint32_t timeoutMs = computeHttpTimeoutMs(lastHttpMs);
      DBG_PRINTF("[HTTP] sending queued payloads (timeout %lu ms)...\n",
                    (unsigned long)timeoutMs);
      sendAllQueuedFiles(lastHttpMs, unitSent);
    } else {
      DBG_PRINTLN("[HTTP] skipped (network not OK)");
    }
  } else {
    DBG_PRINTLN("[HTTP] disabled");
  }

  // Modem best-effort power down.
  if (modemOk) {
    modemBestEffortPowerDown();
  }

  // Deep sleep until next interval.
  DBG_PRINTLN("[SLEEP] Sleeping for 15 minutes...");
  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS * 1000000ULL);
  DBG_FLUSH();
  esp_deep_sleep_start();
}

void loop() {
  // Not used; we deep sleep in setup().
}
