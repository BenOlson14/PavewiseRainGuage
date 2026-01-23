/************************************************************************************
  LILYGO T-SIM7600 + ESP32 + SD (SPI) + DFRobot Rainfall Sensor (I2C)
  ==================================================================================
  SERIAL-ONLY TESTER (HTTP DISABLED) — MAX DEBUG + TIMING METRICS + ADAPTIVE GPS TIMEOUT
  ==================================================================================

  What this tester prints each wake:
  ---------------------------------
  MODEM / CELLULAR:
    - Modem power-on sequence (PWRKEY/FLIGHT/DTR)
    - Time to become AT-responsive
    - Time for modem.init()
    - IMEI + ICCID
    - Time to register on cellular network (waitForNetwork)
    - Operator name + RSSI (CSQ)
    - Time to connect GPRS + Local IP (optional)

  GPS:
    - Whether GPS refresh is needed (first run / time unknown / every 6 hours)
    - Uses an adaptive timeout:
        * Starts at 10 minutes
        * After a successful fix: next timeout = fix_time * 1.20 (min 60s)
        * If a fix times out: reset timeout back to 10 minutes
    - Prints GPS attempt duration and if it succeeded

  DATA:
    - Battery: analogReadMilliVolts(pin35) and scaled V = (mV/1000)*2
    - Rainfall delta: unchanged (cumulative difference stored on SD)
    - Prints JSON_LINE for quick copy/paste

************************************************************************************/

// MUST be before TinyGSM include
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>  // included, NOT USED in serial tester
#include "DFRobot_RainfallSensor.h"

// ======================== SETTINGS =========================
#define uS_TO_S_FACTOR      1000000ULL
#define TIME_TO_SLEEP       300                 // 5 minutes

// T-SIM7600 pins (from your header)
#define UART_BAUD           115200
#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

#define BOARD_BAT_ADC_PIN   35
#define LED_PIN             12

// SD purge thresholds (used if you later re-enable daily logs)
static const float SD_PURGE_START_PCT  = 80.0f;
static const float SD_PURGE_TARGET_PCT = 75.0f;

// GPS anchor rules
static const uint32_t GPS_REFRESH_SECONDS     = 6UL * 3600UL;

// Adaptive timeout rules
static const uint32_t GPS_TIMEOUT_DEFAULT_MS  = 10UL * 60UL * 1000UL; // 10 min
static const uint32_t GPS_TIMEOUT_FLOOR_MS    = 60UL * 1000UL;        // 60 sec min
static const float    GPS_TIMEOUT_MARGIN      = 1.20f;                // +20%

// APN (only needed if you want to see GPRS/IP; GPS can work without it)
static const char APN[]       = "hologram";
static const char GPRS_USER[] = "";
static const char GPRS_PASS[] = "";

// SD paths
static const char *DATA_DIR             = "/data";
static const char *RAIN_PREV_TOTAL_FILE = "/data/rain_prev_total_mm.txt";

// ======================== RTC STATE ========================
// Survive deep sleep
RTC_DATA_ATTR uint32_t lastGpsEpoch          = 0;
RTC_DATA_ATTR uint32_t lastSendEpoch         = 0;
RTC_DATA_ATTR uint32_t gpsTimeoutMsAdaptive  = GPS_TIMEOUT_DEFAULT_MS;
RTC_DATA_ATTR uint32_t wakeCounter           = 0;

// ======================== OBJECTS ==========================
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
DFRobot_RainfallSensor_I2C RainSensor(&Wire);

// ======================== STRUCTS (MUST BE ABOVE USE) ======
struct CellTimings {
  uint32_t atMs   = 0;   // time until AT responds
  uint32_t initMs = 0;   // time for modem.init()
  uint32_t netMs  = 0;   // time for waitForNetwork()
  uint32_t gprsMs = 0;   // time for gprsConnect()
};

// ======================== SMALL HELPERS ====================
static uint32_t elapsedMs(unsigned long startMs) {
  return (uint32_t)(millis() - startMs);
}
static void printDuration(const char *label, bool ok, uint32_t ms) {
  Serial.printf("%s: %s in %lu ms (%.2f s)\n",
                label, ok ? "OK" : "FAIL", (unsigned long)ms, ms / 1000.0f);
}

// ======================== PURE UTC MATH (NO TZ) =============
static bool isLeapYear(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}
static uint16_t daysBeforeMonth(int y, int m) {
  static const uint16_t cumDays[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  uint16_t d = cumDays[m - 1];
  if (m > 2 && isLeapYear(y)) d += 1;
  return d;
}

// ======================== SD HELPERS =======================
static bool ensureDir(const char *path) {
  if (SD.exists(path)) return true;
  return SD.mkdir(path);
}
static bool initSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) return false;
  return ensureDir(DATA_DIR);
}

// (Purging functions left in place; not critical for this tester, but harmless)
static float sdUsedPercent() {
  uint64_t total = SD.totalBytes();
  uint64_t used  = SD.usedBytes();
  if (total == 0) return 0.0f;
  return 100.0f * (float)used / (float)total;
}
static bool isDailyLogName(const char *name) {
  String s(name);
  if (s.length() != 16) return false;
  if (!s.startsWith("log_")) return false;
  if (!s.endsWith(".csv")) return false;
  for (int i = 4; i < 12; i++) if (s[i] < '0' || s[i] > '9') return false;
  return true;
}
static bool findOldestDailyLog(String &oldestPath) {
  File dir = SD.open(DATA_DIR);
  if (!dir) return false;
  String bestName = "";
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      String base = String(f.name());
      int slash = base.lastIndexOf('/');
      if (slash >= 0) base = base.substring(slash + 1);
      if (isDailyLogName(base.c_str())) {
        if (bestName == "" || base < bestName) bestName = base;
      }
    }
    f.close();
  }
  dir.close();
  if (bestName == "") return false;
  oldestPath = String(DATA_DIR) + "/" + bestName;
  return true;
}
static void purgeLogsIfNeeded() {
  float usedPct = sdUsedPercent();
  if (usedPct < SD_PURGE_START_PCT) return;
  Serial.printf("[SD] %.1f%% used -> purging...\n", usedPct);
  while (sdUsedPercent() > SD_PURGE_TARGET_PCT) {
    String oldest;
    if (!findOldestDailyLog(oldest)) break;
    Serial.printf("[SD] Deleting: %s\n", oldest.c_str());
    SD.remove(oldest.c_str());
  }
  Serial.printf("[SD] After purge: %.1f%% used\n", sdUsedPercent());
}

// ======================== BATTERY ==========================
static float readBatteryVoltage(uint32_t &rawOut) {
  // 1. Ensure the power rail for the divider is HIGH (often shared with LED/Peripherals)
  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, HIGH); 
  delay(50); // Let voltage stabilize

  // 2. Standardize ADC settings
  analogReadResolution(12); // 0-4095
  analogSetAttenuation(ADC_11db); // 0-3.3V range

  // 3. Take multiple samples to average out modem noise
  uint32_t sum = 0;
  for(int i=0; i<10; i++) {
      sum += analogRead(BOARD_BAT_ADC_PIN);
      delay(5);
  }
  rawOut = sum / 10;
  
  // 4. Calculation:
  // (Raw / 4095) * 3.3V (Ref) * 2.0 (Divider) * 1.1 (ESP32 Calibration Factor)
  // 1.1 is the common scaling factor for ESP32's internal 1100mV reference
  return (rawOut / 4095.0f) * 3.3f * 2.0f * 1.1f;
}

// ======================== RAIN (UNCHANGED) =================
static bool readFloatFromFile(const char *path, float &out) {
  if (!SD.exists(path)) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  s.trim();
  if (!s.length()) return false;
  out = s.toFloat();
  return true;
}
static bool writeFloatToFile(const char *path, float v) {
  if (SD.exists(path)) SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  f.print(String(v, 4));
  f.print("\n");
  f.close();
  return true;
}
static float getRainIntervalInches() {
  float totalMm = RainSensor.getRainfall();
  float prevMm = 0.0f;
  bool havePrev = readFloatFromFile(RAIN_PREV_TOTAL_FILE, prevMm);

  float deltaMm = 0.0f;
  if (!havePrev) {
    deltaMm = 0.0f;
  } else {
    deltaMm = totalMm - prevMm;
    if (deltaMm < 0.0f) deltaMm = (totalMm < 50.0f) ? totalMm : 0.0f;
    if (deltaMm > 100.0f) deltaMm = 0.0f;
  }

  writeFloatToFile(RAIN_PREV_TOTAL_FILE, totalMm);
  return deltaMm / 25.4f;
}

// ======================== MODEM POWER (T-SIM7600) ==========
static void ts7600_powerPinsSetup() {
  // Match the proven LILYGO T-SIM7600 bring-up sequence (same logic as AllFunctions.ino):
  //  - FLIGHT must be HIGH to enable the modem (not "airplane mode")
  //  - PWRKEY must be driven HIGH briefly, then LOW (and can be left LOW)
  //  - DTR LOW keeps the modem awake (HIGH can allow sleep depending on module config)
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_FLIGHT, OUTPUT);
  pinMode(MODEM_DTR, OUTPUT);
  pinMode(MODEM_STATUS, INPUT);  // optional; some boards don’t wire this meaningfully

  digitalWrite(MODEM_FLIGHT, HIGH);  // enable RF (flight mode OFF)
  digitalWrite(MODEM_DTR, LOW);      // keep awake

  // PWRKEY "kick" (as in the working sketch)
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);                        // need a short high pulse
  digitalWrite(MODEM_PWRKEY, LOW);   // leave LOW
}

static bool powerOnModem_TSIM7600() {
  ts7600_powerPinsSetup();

  // MODEM_STATUS is not always reliable on every board revision, so treat it as advisory.
  int st = digitalRead(MODEM_STATUS);
  Serial.printf("[MODEM] MODEM_STATUS after PWRKEY kick: %d (may be unwired)\n", st);

  // Give the modem time to boot before we start sending AT commands.
  Serial.println("[MODEM] Waiting ~5s for modem boot...");
  delay(5000);

  return true;
}


// ======================== NETWORK DETAIL PRINTS =============
static void printNetworkDetails() {
  int16_t rssi = modem.getSignalQuality();  // CSQ-like
  Serial.printf("[NET] RSSI (CSQ): %d\n", (int)rssi);

  String op = modem.getOperator();
  Serial.print("[NET] Operator : ");
  Serial.println(op.length() ? op : "(unknown)");

  IPAddress ip = modem.localIP();
  Serial.print("[NET] Local IP : ");
  Serial.println(ip);
}

// ======================== CONNECT MODEM + TIMINGS ===========
static bool connectModemAndMeasure(CellTimings &t, bool &netOkOut, bool &gprsOkOut) {
  netOkOut = false;
  gprsOkOut = false;

  Serial.println("[MODEM] Powering ON sequence...");
  powerOnModem_TSIM7600();

  Serial.println("[MODEM] Starting UART...");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(200);

  // AT responsiveness timing
  Serial.println("[MODEM] Testing AT responsiveness (up to ~10s)...");
  unsigned long s = millis();
  bool atOk = false;
  for (int i = 0; i < 10; i++) {
    if (modem.testAT()) { atOk = true; break; }
    delay(1000);
  }
  t.atMs = elapsedMs(s);
  printDuration("[MODEM] AT responsive", atOk, t.atMs);
  if (!atOk) return false;

  // modem.init timing
  Serial.println("[MODEM] modem.init()...");
  s = millis();
  bool initOk = modem.init();
  t.initMs = elapsedMs(s);
  printDuration("[MODEM] init()", initOk, t.initMs);
  if (!initOk) return false;

#if defined(TINY_GSM_MODEM_HAS_GPS)
  // Match AllFunctions.ino: configure GNSS mode (BEIDOU + DPO enabled)
  // (Safe even if GPS is not used every wake)
  uint8_t gnssMode = modem.getGNSSMode();
  Serial.print("[GPS] GNSS Mode (before): ");
  Serial.println(gnssMode);
  modem.setGNSSMode(1, 1);
  delay(200);
  Serial.print("[GPS] GNSS Mode (after) : ");
  Serial.println(modem.getGNSSMode());
#endif

  // Identifiers
  Serial.print("[MODEM] Modem Info: ");
  Serial.println(modem.getModemInfo());

  String imei = modem.getIMEI();
  if (imei.length()) { Serial.print("[MODEM] IMEI : "); Serial.println(imei); }

  String iccid = modem.getSimCCID();
  if (iccid.length()) { Serial.print("[MODEM] ICCID: "); Serial.println(iccid); }

  // Network registration timing
  Serial.println("[NET] Waiting for network registration (up to 60s)...");
  s = millis();
  netOkOut = modem.waitForNetwork(60000L);
  t.netMs = elapsedMs(s);
  printDuration("[NET] Registered", netOkOut, t.netMs);

  // Operator/RSSI print (works sometimes even if not registered)
  printNetworkDetails();

  // GPRS timing (optional)
  Serial.println("[NET] Connecting GPRS (optional)...");
  s = millis();
  gprsOkOut = modem.gprsConnect(APN, GPRS_USER, GPRS_PASS);
  t.gprsMs = elapsedMs(s);
  printDuration("[NET] GPRS", gprsOkOut, t.gprsMs);

  if (gprsOkOut) printNetworkDetails();

  return true;
}

static void disconnectModem() {
  Serial.println("[NET] Disconnecting modem services...");
  gsmClient.stop();
  modem.gprsDisconnect();
}

// ======================== GPS + TIMING ======================
static bool tryGetGpsUtcWithTiming(uint32_t timeoutMs,
                                  float &lat, float &lon,
                                  uint32_t &epochOut,
                                  uint32_t &fixTimeMsOut) {
#if defined(TINY_GSM_MODEM_HAS_GPS)
  Serial.println("[GPS] Enabling GPS/GNSS...");
  modem.enableGPS();
  delay(200);
#endif

  bool fixOk = false;

  unsigned long start = millis();
  unsigned long lastProgress = 0;

  while (millis() - start < timeoutMs) {
    float speed = 0.0f;
    float alt   = 0.0f;
    int vsat = 0;
    int usat = 0;
    float accuracy = 0.0f;
    int year = 0, month = 0, day = 0;
    int hour = 0, minute = 0, second = 0;

    bool ok = modem.getGPS(&lat, &lon, &speed, &alt,
                           &vsat, &usat, &accuracy,
                           &year, &month, &day, &hour, &minute, &second);

    if (millis() - lastProgress > 5000) {
      lastProgress = millis();
      Serial.printf("[GPS] searching... ok=%d vsat=%d usat=%d lat=%.6f lon=%.6f\n",
                    ok ? 1 : 0, vsat, usat, lat, lon);
    }

    if (ok && year >= 1970 && month >= 1 && month <= 12 && day >= 1 && day <= 31) {
      uint32_t days = 0;
      for (int y = 1970; y < year; y++) days += isLeapYear(y) ? 366U : 365U;
      days += daysBeforeMonth(year, month);
      days += (uint32_t)(day - 1);

      epochOut = days * 86400UL
               + (uint32_t)hour * 3600UL
               + (uint32_t)minute * 60UL
               + (uint32_t)second;

      fixTimeMsOut = elapsedMs(start);

      Serial.printf("[GPS] FIX OK: UTC=%04d-%02d-%02d %02d:%02d:%02d  acc=%.2f vsat=%d usat=%d\n",
                    year, month, day, hour, minute, second, accuracy, vsat, usat);
      fixOk = true;
      goto gps_cleanup;
    }

    delay(1000);
  }

  fixTimeMsOut = elapsedMs(start);
  goto gps_cleanup;

gps_cleanup:
#if defined(TINY_GSM_MODEM_HAS_GPS)
  Serial.println("[GPS] Disabling GPS/GNSS...");
  modem.disableGPS();
#endif
  return fixOk;
}


// ======================== SUMMARY PRINT ====================
static void printSummary(uint32_t epochNow,
                         uint32_t rawBatMv, float batV,
                         float lat, float lon,
                         float rainIn,
                         bool gpsFresh,
                         uint32_t gpsTimeoutUsed,
                         uint32_t gpsAttemptMs,
                         const CellTimings &cellT,
                         bool netOk, bool gprsOk) {
  Serial.println();
  Serial.println("===================== SUMMARY =====================");
  Serial.printf("Wake #              : %lu\n", (unsigned long)wakeCounter);
  Serial.printf("Epoch (UTC)         : %lu\n", (unsigned long)epochNow);

  Serial.printf("Battery raw (mV)    : %lu\n", (unsigned long)rawBatMv);
  Serial.printf("Battery scaled (V)  : %.3f\n", batV);

  Serial.printf("Rain interval (in)  : %.4f\n", rainIn);
  Serial.printf("Lat/Lon             : %.6f, %.6f\n", lat, lon);

  Serial.printf("AT time             : %lu ms\n", (unsigned long)cellT.atMs);
  Serial.printf("init() time         : %lu ms\n", (unsigned long)cellT.initMs);
  Serial.printf("Network reg         : %s in %lu ms\n", netOk ? "YES" : "NO", (unsigned long)cellT.netMs);
  Serial.printf("GPRS                : %s in %lu ms\n", gprsOk ? "YES" : "NO", (unsigned long)cellT.gprsMs);

  Serial.printf("GPS fresh           : %d\n", gpsFresh ? 1 : 0);
  Serial.printf("GPS timeout used    : %lu ms (%.1f min)\n",
                (unsigned long)gpsTimeoutUsed, gpsTimeoutUsed / 60000.0f);
  Serial.printf("GPS attempt time    : %lu ms (%.1f min)\n",
                (unsigned long)gpsAttemptMs, gpsAttemptMs / 60000.0f);

  Serial.print("JSON_LINE: ");
  Serial.print("{\"ts_utc\":"); Serial.print((unsigned long)epochNow);
  Serial.print(",\"battery_v\":"); Serial.print(batV, 3);
  Serial.print(",\"lat\":"); Serial.print(lat, 6);
  Serial.print(",\"lon\":"); Serial.print(lon, 6);
  Serial.print(",\"rain_5min_in\":"); Serial.print(rainIn, 4);
  Serial.print(",\"gps_fresh\":"); Serial.print(gpsFresh ? 1 : 0);
  Serial.println("}");
  Serial.println("==================================================");
}

// ======================== ARDUINO MAIN ======================
void setup() {
  Serial.begin(115200);
  delay(300);

  wakeCounter++;

  Serial.println();
  Serial.println("==================================================");
  Serial.printf("[BOOT] Wake #%lu start\n", (unsigned long)wakeCounter);
  Serial.println("==================================================");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ADC config for analogReadMilliVolts()
  pinMode(BOARD_BAT_ADC_PIN, INPUT);
  analogSetPinAttenuation(BOARD_BAT_ADC_PIN, ADC_11db);

  // I2C + rainfall sensor
  Wire.begin();
  bool rainOk = RainSensor.begin();
  Serial.printf("[RAIN] begin(): %s\n", rainOk ? "OK" : "FAIL");

  // SD
  bool sdOk = initSD();
  Serial.printf("[SD] init: %s\n", sdOk ? "OK" : "FAIL");
  if (sdOk) purgeLogsIfNeeded();

  // Battery
  uint32_t rawBatMv = 0;
  float batV = readBatteryVoltage(rawBatMv);
  Serial.printf("[BAT] raw=%lu mV, scaled=%.3f V\n", (unsigned long)rawBatMv, batV);

  // Rain (unchanged)
  float rainIn = 0.0f;
  if (rainOk && sdOk) rainIn = getRainIntervalInches();
  else Serial.println("[RAIN] Skipping rainfall delta (sensor or SD not ready).");

  // Modem/network (timed)
  CellTimings cellT;
  bool netOk = false;
  bool gprsOk = false;

  Serial.println("[MODEM] Connecting modem (timed)...");
  bool modemOk = connectModemAndMeasure(cellT, netOk, gprsOk);
  Serial.printf("[MODEM] Overall modem connect: %s\n", modemOk ? "OK" : "FAIL");

  // Predict time between anchors
  uint32_t epochNow = 0;
  if (lastSendEpoch > 0) {
    epochNow = lastSendEpoch + TIME_TO_SLEEP;
    Serial.printf("[TIME] Predicted epoch = %lu\n", (unsigned long)epochNow);
  } else {
    Serial.println("[TIME] No previous epoch stored yet.");
  }

  // Determine if GPS anchor is needed
  bool needGps = (lastGpsEpoch == 0) || (epochNow == 0) ||
                 ((epochNow > lastGpsEpoch) && ((epochNow - lastGpsEpoch) >= GPS_REFRESH_SECONDS));
  Serial.printf("[GPS] needGps=%d (lastGpsEpoch=%lu)\n", needGps ? 1 : 0, (unsigned long)lastGpsEpoch);

  float lat = 0.0f, lon = 0.0f;
  bool gpsFresh = false;
  uint32_t gpsAttemptMs = 0;

  // Use adaptive timeout (sanity checked)
  uint32_t gpsTimeoutUsed = gpsTimeoutMsAdaptive;
  if (gpsTimeoutUsed < GPS_TIMEOUT_FLOOR_MS || gpsTimeoutUsed > (30UL * 60UL * 1000UL)) {
    gpsTimeoutUsed = GPS_TIMEOUT_DEFAULT_MS;
    gpsTimeoutMsAdaptive = GPS_TIMEOUT_DEFAULT_MS;
  }

  if (needGps && modemOk) {
    Serial.printf("[GPS] Attempting fix with timeout %lu ms (%.1f min)\n",
                  (unsigned long)gpsTimeoutUsed, gpsTimeoutUsed / 60000.0f);

    uint32_t gpsEpoch = 0;
    bool ok = tryGetGpsUtcWithTiming(gpsTimeoutUsed, lat, lon, gpsEpoch, gpsAttemptMs);

    if (ok) {
      gpsFresh = true;
      epochNow = gpsEpoch;
      lastGpsEpoch = gpsEpoch;

      // Learn next timeout = fix_time * 1.2, but never below floor
      uint32_t learned = (uint32_t)((double)gpsAttemptMs * GPS_TIMEOUT_MARGIN);
      if (learned < GPS_TIMEOUT_FLOOR_MS) learned = GPS_TIMEOUT_FLOOR_MS;
      gpsTimeoutMsAdaptive = learned;

      Serial.printf("[GPS] FIX SUCCESS in %lu ms (%.2f s). Next timeout=%lu ms (%.2f min)\n",
                    (unsigned long)gpsAttemptMs, gpsAttemptMs / 1000.0f,
                    (unsigned long)gpsTimeoutMsAdaptive, gpsTimeoutMsAdaptive / 60000.0f);
    } else {
      Serial.printf("[GPS] FIX TIMEOUT after %lu ms (%.2f s)\n",
                    (unsigned long)gpsAttemptMs, gpsAttemptMs / 1000.0f);

      // Reset adaptive timeout to 10 minutes on timeout (your rule)
      gpsTimeoutMsAdaptive = GPS_TIMEOUT_DEFAULT_MS;
      Serial.printf("[GPS] Adaptive timeout reset to default: %lu ms (10 min)\n",
                    (unsigned long)gpsTimeoutMsAdaptive);

      if (epochNow == 0) {
        Serial.println("[TIME] WARNING: epochNow still 0 (no previous time + no GPS fix yet).");
      }
    }
  } else {
    if (!needGps) Serial.println("[GPS] Not refreshing this wake (within 6-hour window).");
    if (!modemOk) Serial.println("[GPS] Skipping GPS (modem not OK).");
  }

  // Save timestamp used this wake
  lastSendEpoch = epochNow;

  // Summary includes cellular + gps attempt durations
  printSummary(epochNow, rawBatMv, batV, lat, lon, rainIn, gpsFresh,
               gpsTimeoutUsed, gpsAttemptMs, cellT, netOk, gprsOk);

  if (modemOk) disconnectModem();

  Serial.println("[SLEEP] Sleeping for 5 minutes...");
  esp_sleep_enable_timer_wakeup((uint64_t)TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(50);
  esp_deep_sleep_start();
}

void loop() {}

