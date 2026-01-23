/************************************************************************************
Pavewise Mobile Rain Gauge — GPS/STRING TEST BUILD (TestSerial_fixed_gps_NEW.ino)

PURPOSE OF THIS FILE:
  This build is specifically for verifying that the device is:
    - reading rainfall correctly (and computing 15-minute delta)
    - reading battery voltage correctly
    - building the compact payload correctly (NO commas, NO decimals)
    - only including lat/lon in the payload when GPS is refreshed (every 6 hours)
    - writing daily logs + queue files to SD
    - attempting modem power-down in software before deep sleep

KEY DIFFERENCE VS PRODUCTION BUILDS:
  - HTTP POST is intentionally DISABLED here.
  - Instead of POSTing, we print the payload string that would be sent.
  - We still write queue files so you can inspect /queue content on SD.

DATA STORAGE STRATEGY:
  - /logs/log_YYYYMMDD.csv        (daily permanent history, human readable)
  - /queue/q_epoch_wake.txt       (one per interval; would be deleted after successful upload in production)
  - /state/rain_prev_total_mm.txt (previous cumulative rainfall total for delta computation)
  - /state/gps_last.txt           (last known epoch, lat, lon)
  - /state/identity.txt           (cached ICCID, IMEI so identity survives power loss)

TIME STRATEGY:
  - g_epochEstimate stored in RTC memory survives deep sleep.
  - On each wake, if epoch exists, add 900 seconds (15 min).
  - Every 6 hours (or first boot), attempt GPS to re-anchor time and update location.

POWER STRATEGY (SOFTWARE-ONLY):
  - WiFi/BT off, CPU 80MHz.
  - GNSS OFF when not needed.
  - Best-effort modem shutdown: +CFUN=0, +CPOF, PWRKEY pulse, FLIGHT HIGH, UART end.
  - True hardware power removal requires VBAT gating (not done here).

************************************************************************************/

#define ENABLE_DEBUG 1

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include <WiFi.h>
#include <esp_bt.h>

#include <TinyGsmClient.h>

// Rain sensor library (DFRobot tipping bucket)
#include "DFRobot_RainfallSensor.h"

// ============================= USER SETTINGS =============================

// Wake every 15 minutes
static const uint32_t WAKE_INTERVAL_SECONDS = 15UL * 60UL;

// GPS refresh every 6 hours
static const uint32_t GPS_REFRESH_SECONDS = 6UL * 3600UL;

// APN (Hologram)
static const char APN[]       = "hologram";
static const char GPRS_USER[] = "";
static const char GPRS_PASS[] = "";

// ============================= BOARD PINS =============================

// UART settings
#define UART_BAUD     115200

// ESP32 UART -> SIM7600
#define MODEM_TX      27
#define MODEM_RX      26

// SIM7600 control pins (typical LilyGO mapping)
#define MODEM_PWRKEY  4
#define MODEM_DTR     32
#define MODEM_FLIGHT  25

// Battery ADC pin (typical on LilyGO)
#define BAT_ADC_PIN   35

// SD SPI pins
#define SD_MISO       2
#define SD_MOSI       15
#define SD_SCLK       14
#define SD_CS         13

// ============================= SD FILE STRUCTURE =============================

// Permanent daily logs (one file per day)
static const char *DIR_LOGS  = "/logs";

// Queue records (one file per interval; production would upload and then delete)
static const char *DIR_QUEUE = "/queue";

// Persistent state for power-loss recovery
static const char *DIR_STATE = "/state";

static const char *FILE_RAIN_PREV_TOTAL = "/state/rain_prev_total_mm.txt";
static const char *FILE_GPS_LAST        = "/state/gps_last.txt";
static const char *FILE_IDENTITY        = "/state/identity.txt";

// ============================= DEBUG MACROS =============================

#if ENABLE_DEBUG
  #define DBG_BEGIN()       do{ Serial.begin(115200); delay(200);}while(0)
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

// ============================= MODEM + SENSOR OBJECTS =============================

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);

DFRobot_RainfallSensor_I2C RainSensor(&Wire);

// ============================= SMALL HELPERS =============================

// Safe elapsed millis calculator (handles wrap naturally).
static uint32_t elapsedMs(uint32_t startMs) {
  return (uint32_t)(millis() - startMs);
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
static void pulsePwrKey(uint32_t holdMs = 1500) {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(holdMs);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
}

// ============================= SD HELPERS =============================

// Mount SD and ensure directory structure exists.
static bool initSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) return false;

  if (!SD.exists(DIR_LOGS))  SD.mkdir(DIR_LOGS);
  if (!SD.exists(DIR_QUEUE)) SD.mkdir(DIR_QUEUE);
  if (!SD.exists(DIR_STATE)) SD.mkdir(DIR_STATE);

  return true;
}

// Read a one-line text file and trim.
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

// Load cached identity: "iccid,imei"
static bool loadIdentity(String &iccid, String &imei) {
  String line;
  if (!readTextFile(FILE_IDENTITY, line)) return false;
  int c = line.indexOf(',');
  if (c < 0) return false;

  iccid = line.substring(0, c); iccid.trim();
  imei  = line.substring(c + 1); imei.trim();
  return (iccid.length() > 0 && imei.length() > 0);
}

// Save identity cache.
static void saveIdentity(const String &iccid, const String &imei) {
  if (iccid.length() == 0 || imei.length() == 0) return;
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
//   E|W|R|B|S|I
//
// If GPS refreshed this wake, append:
//   |LA|LO
//
// Scale rules:
//   R  = round(rain_mm * 100)             => server: rain_mm = R / 100.0
//   B  = battery millivolts (integer)     => server: batt_v  = B / 1000.0
//   LA = round(lat_deg * 1e7)             => server: lat     = LA / 1e7
//   LO = round(lon_deg * 1e7)             => server: lon     = LO / 1e7
//
static int32_t scaleRain100(float mm) {
  return (int32_t)lroundf(mm * 100.0f);
}

static int32_t scaleDeg1e7(double deg) {
  return (int32_t)llround(deg * 10000000.0);
}

static String buildUploadPayload(uint32_t epochNow,
                                 uint32_t wakeCounter,
                                 float rainDeltaMm,
                                 uint32_t battMv,
                                 int rssi,
                                 const String &imei,
                                 bool includeGps,
                                 double lat,
                                 double lon) {
  int32_t R = scaleRain100(rainDeltaMm);

  String s;
  s.reserve(includeGps ? 120 : 90);

  s += String(epochNow);
  s += "|";
  s += String(wakeCounter);
  s += "|";
  s += String(R);
  s += "|";
  s += String((unsigned long)battMv);
  s += "|";
  s += String(rssi);
  s += "|";
  s += imei;

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
  // FLIGHT LOW enables RF functions; HIGH often disables RF.
  pinMode(MODEM_FLIGHT, OUTPUT);
  digitalWrite(MODEM_FLIGHT, LOW);

  // DTR low while actively talking.
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);

  DBG_PRINTLN("[MODEM] PWRKEY pulse (boot)...");
  pulsePwrKey(1500);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  uint32_t start = millis();
  while (elapsedMs(start) < 12000) {
    modem.sendAT("AT");
    if (modem.waitResponse(250) == 1) {
      DBG_PRINTLN("[MODEM] AT OK");
      return true;
    }
    delay(250);
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
  pulsePwrKey(1500);

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
    DBG_PRINTLN("[NET] modem.init failed");
    return false;
  }

  String tIccid = modem.getSimCCID();
  String tImei  = modem.getIMEI();

  if (tIccid.length()) iccid = tIccid;
  if (tImei.length())  imei  = tImei;

  DBG_PRINTF("[NET] ICCID=%s\n", iccid.c_str());
  DBG_PRINTF("[NET] IMEI =%s\n", imei.c_str());

  if (!modem.waitForNetwork(60000L)) {
    DBG_PRINTLN("[NET] waitForNetwork timeout");
    return false;
  }

  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    DBG_PRINTLN("[NET] gprsConnect failed");
    return false;
  }

  rssi = modem.getSignalQuality();
  DBG_PRINTF("[NET] RSSI/CSQ=%d\n", rssi);

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
                       bool &hasEpoch, uint32_t &epochOut) {
  hasEpoch = false;
  epochOut = 0;

  // Turn GNSS ON.
  modem.sendAT("+CGPS=1");
  if (modem.waitResponse(2000) != 1) return false;

  uint32_t start = millis();
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
        return true;
      }
    }

    delay(2000);
  }

  // Timeout: ensure GNSS OFF.
  modem.sendAT("+CGPS=0");
  modem.waitResponse(1000);
  return false;
}

// ============================= LOGGING / QUEUE =============================

// Append line to daily CSV log; create header if file new.
static void appendDaily(const String &path, const String &line) {
  bool isNew = !SD.exists(path);
  File f = SD.open(path, FILE_APPEND);
  if (!f) return;

  if (isNew) {
    f.println("epoch_utc,lat,lon,rain_mm_15m,batt_v,iccid,imei,rssi,gps_included_upload");
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

// ============================= RAINFALL =============================

static float readRainTotalMm() {
  return RainSensor.getRainfall();
}

static float rainDelta15m(float totalMm) {
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

  DBG_PRINTF("\n\n===== WAKE #%lu =====\n", (unsigned long)g_wakeCounter);

  // If we already have epoch estimate, step forward by 15 minutes each wake.
  if (g_epochEstimate > 0) {
    g_epochEstimate += WAKE_INTERVAL_SECONDS;
  }

  // Mount SD (critical for logging/queue). If fails, sleep.
  if (!initSD()) {
    DBG_PRINTLN("[FATAL] SD init failed -> sleep");
    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS * 1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }

  // Load cached identity so we can still build payload even if modem fails.
  String iccid = "unknown";
  String imei  = "unknown";
  loadIdentity(iccid, imei);

  // Load last GPS state (survives power loss).
  uint32_t lastGpsEpoch = 0;
  double lastLat = 0.0, lastLon = 0.0;
  loadLastGps(lastGpsEpoch, lastLat, lastLon);

  // If RTC epoch estimate was lost but SD has GPS epoch, restore it.
  if (g_epochEstimate == 0 && lastGpsEpoch > 0) {
    g_epochEstimate = lastGpsEpoch;
    DBG_PRINTF("[STATE] restored epoch from SD: %lu\n", (unsigned long)g_epochEstimate);
  }

  // Rain sensor init (I2C).
  Wire.begin();
  bool rainOk = RainSensor.begin();
  DBG_PRINTF("[RAIN] begin=%s\n", rainOk ? "OK" : "FAIL");

  // Read rainfall totals and compute delta for this wake interval.
  float totalMm = readRainTotalMm();
  float deltaMm = rainDelta15m(totalMm);

  DBG_PRINTF("[RAIN] total=%.3f mm  delta15m=%.3f mm\n", totalMm, deltaMm);

  // Read battery.
  uint32_t battMvRaw = 0;
  float battV = readBatteryVoltage(battMvRaw);
  uint32_t battMvVBAT = (uint32_t)lroundf(battV * 1000.0f);

  DBG_PRINTF("[BATT] ADC=%lu mV  VBAT=%.3f V (~%lu mV)\n",
             (unsigned long)battMvRaw, battV, (unsigned long)battMvVBAT);

  // Power modem on.
  bool modemOk = modemPowerOn();
  DBG_PRINTLN(modemOk ? "[MODEM] OK" : "[MODEM] FAIL");

  // Network connect (only if modemOk).
  int rssi = -1;
  bool netOk = false;
  if (modemOk) {
    netOk = connectNetwork(iccid, imei, rssi);
    DBG_PRINTF("[NET] %s RSSI=%d\n", netOk ? "OK" : "FAIL", rssi);

    // Save identity so power loss still has ID next boot.
    saveIdentity(iccid, imei);
  }

  // Determine our best epoch for this wake.
  uint32_t epochNow = g_epochEstimate;
  if (epochNow == 0 && lastGpsEpoch > 0) epochNow = lastGpsEpoch;

  bool haveValidEpoch = (epochNow > 0);

  // Decide if GPS refresh is due:
  //  - if no valid epoch
  //  - or no last gps epoch
  //  - or older than 6 hours
  bool needGps = (!haveValidEpoch) ||
                 (lastGpsEpoch == 0) ||
                 (epochNow >= lastGpsEpoch && (epochNow - lastGpsEpoch) >= GPS_REFRESH_SECONDS);

  bool gpsIncludedInUpload = false;

  // GPS fix attempt only if network is up AND GPS is due.
  if (netOk && needGps) {
    DBG_PRINTLN("[GPS] refresh due; attempting fix...");
    double lat = 0.0, lon = 0.0;
    bool hasGpsEpoch = false;
    uint32_t gpsEpoch = 0;

    // Up to 8 minutes for a fix.
    bool gpsOk = gpsAcquire(8UL * 60UL * 1000UL, lat, lon, hasGpsEpoch, gpsEpoch);

    if (gpsOk) {
      lastLat = lat;
      lastLon = lon;

      if (hasGpsEpoch) {
        lastGpsEpoch = gpsEpoch;
        g_epochEstimate = gpsEpoch;
        epochNow = gpsEpoch;

        DBG_PRINTF("[GPS] FIX OK lat=%.7f lon=%.7f epoch=%lu\n",
                   lastLat, lastLon, (unsigned long)gpsEpoch);
      } else {
        // If GPS time not parseable, still store location and keep existing epoch estimate.
        if (epochNow == 0) epochNow = (uint32_t)(millis() / 1000UL);
        lastGpsEpoch = epochNow;
        if (g_epochEstimate == 0) g_epochEstimate = epochNow;

        DBG_PRINTF("[GPS] FIX OK lat=%.7f lon=%.7f (no epoch)\n",
                   lastLat, lastLon);
      }

      saveLastGps(lastGpsEpoch, lastLat, lastLon);
      gpsIncludedInUpload = true;  // ONLY TRUE when we refreshed GPS this wake
    } else {
      DBG_PRINTLN("[GPS] FIX FAIL (using last known)");
    }
  } else {
    DBG_PRINTLN("[GPS] not due or network not OK");
  }

  // If epoch still zero, fall back to lastGpsEpoch.
  if (epochNow == 0) epochNow = lastGpsEpoch;

  // DAILY LOG (readable CSV, local only)
  {
    char dailyLine[256];
    snprintf(dailyLine, sizeof(dailyLine),
             "%lu,%.7f,%.7f,%.3f,%.3f,%s,%s,%d,%d",
             (unsigned long)epochNow,
             lastLat, lastLon,
             deltaMm, battV,
             iccid.c_str(), imei.c_str(),
             rssi,
             gpsIncludedInUpload ? 1 : 0);

    String logPath = dailyLogPath(epochNow ? epochNow : 0);
    appendDaily(logPath, String(dailyLine));

    DBG_PRINTF("[LOG] %s\n", logPath.c_str());
    DBG_PRINTF("[LOG] %s\n", dailyLine);
  }

  // COMPACT PAYLOAD (no commas, no decimals)
  String payload = buildUploadPayload(
    epochNow,
    g_wakeCounter,
    deltaMm,
    battMvVBAT,
    rssi,
    imei,
    gpsIncludedInUpload,   // only include lat/lon when GPS refreshed
    lastLat,
    lastLon
  );

  // Print the payload instead of sending it.
  DBG_PRINTLN("[PAYLOAD] would POST:");
  DBG_PRINTLN(payload);

  // Queue record still written so you can inspect /queue output.
  String qPath = makeQueueFilename(epochNow ? epochNow : g_wakeCounter, g_wakeCounter);
  writeQueueLine(qPath, payload);
  DBG_PRINTF("[QUEUE] wrote %s\n", qPath.c_str());

  // HTTP POST intentionally disabled in this build:
  //
  //   int status = 0;
  //   bool ok = httpPostPayload(payload, status);
  //   Serial.printf("[HTTP] status=%d ok=%d\n", status, ok);
  //

  // Modem best-effort power down.
  if (modemOk) {
    modemBestEffortPowerDown();
  }

  // Deep sleep until next interval.
  DBG_PRINTLN("[SLEEP] deep sleep 15 min...");
  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS * 1000000ULL);
  DBG_FLUSH();
  esp_deep_sleep_start();
}

void loop() {
  // Not used; we deep sleep in setup().
}


