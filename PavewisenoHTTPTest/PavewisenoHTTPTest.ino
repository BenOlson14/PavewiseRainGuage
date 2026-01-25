/************************************************************************************
Pavewise Mobile Rain Gauge — NO-HTTP TEST BUILD (15-minute cadence)

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
       - DOES NOT send HTTP (ENABLE_HTTP=false)
  5) Gracefully powers down the modem and deep-sleeps for 15 minutes.

This file is the SAME logic as the serial tester build but:
  - HTTP is disabled.
  - Wake interval matches the serial build for consistent behavior.
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
#include "DFRobot_RainfallSensor.h"

// Keep the same cadence as the serial/release builds for consistent behavior.
#define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)
#define PAVEWISE_ENABLE_HTTP false
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

RTC_DATA_ATTR uint32_t g_wakeCounter   = 0;
RTC_DATA_ATTR uint32_t g_epochEstimate = 0;

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsm(modem);
HttpClient http(gsm, SERVER_HOST, SERVER_PORT);

DFRobot_RainfallSensor_I2C RainSensor(&Wire);

// Delay between queued HTTP sends to avoid back-to-back modem churn.
static const uint32_t HTTP_QUEUE_SEND_DELAY_MS = 1000;

static uint32_t elapsedMs(uint32_t startMs){ return (uint32_t)(millis() - startMs); }

// Poll AT until timeout; returns true on first successful response.
static bool waitForAtResponsive(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (elapsedMs(start) < timeoutMs) {
    if (modem.testAT()) return true;
    delay(1000);
  }
  return false;
}

static void disableRadiosForPower() {
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();
}

static void setLowPowerCpu() { setCpuFrequencyMhz(80); }

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
  float used = sdUsedPercent();
  if (used < SD_PURGE_START_PCT) return;
  DBG_PRINTF("[SD] %.1f%% used -> purging...\n", used);
  while (sdUsedPercent() > SD_PURGE_TARGET_PCT) {
    String oldest;
    if (!findOldestDailyLog(oldest)) break;
    DBG_PRINTF("[SD] Deleting: %s\n", oldest.c_str());
    SD.remove(oldest.c_str());
  }
  DBG_PRINTF("[SD] After purge: %.1f%% used\n", sdUsedPercent());
}

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

static bool readFloatFile(const char *path, float &out) {
  String s;
  if (!readTextFile(path, s)) return false;
  out = s.toFloat();
  return true;
}

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

static bool loadIdentity(String &iccid, String &imei) {
  String line;
  if (!readTextFile(FILE_IDENTITY, line)) return false;
  int c = line.indexOf(',');
  if (c < 0) return false;
  iccid = line.substring(0, c); iccid.trim();
  imei  = line.substring(c + 1); imei.trim();
  return (iccid.length() > 0 && imei.length() > 0);
}

static void saveIdentity(const String &iccid, const String &imei) {
  if (iccid.length() == 0 || imei.length() == 0) return;
  writeTextFile(FILE_IDENTITY, iccid + "," + imei);
}

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

static void saveLastGps(uint32_t epoch, double lat, double lon) {
  if (epoch == 0) return;
  char buf[96];
  snprintf(buf, sizeof(buf), "%lu,%.7f,%.7f", (unsigned long)epoch, lat, lon);
  writeTextFile(FILE_GPS_LAST, String(buf));
}

static const float VBAT_DIVIDER    = 2.0f;
static const float VBAT_CAL_MULT   = 1.00f;
static const float VBAT_CAL_OFFSET = 0.00f;

static float readBatteryVoltage(uint32_t &rawMv) {
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
  rawMv = analogReadMilliVolts(BAT_ADC_PIN);
  float v = (rawMv / 1000.0f) * VBAT_DIVIDER;
  v = v * VBAT_CAL_MULT + VBAT_CAL_OFFSET;
  return v;
}

static bool isLeap(int y){ return ((y%4==0)&&(y%100!=0))||(y%400==0); }
static uint8_t dim(int y,int m){
  static const uint8_t d[12]={31,28,31,30,31,30,31,31,30,31,30,31};
  return (m==2 && isLeap(y)) ? 29 : d[m-1];
}
static uint32_t ymdhmsToEpoch(int y,int mo,int da,int h,int mi,int s){
  uint32_t days=0;
  for(int yr=1970; yr<y; yr++) days+=isLeap(yr)?366:365;
  for(int m=1; m<mo; m++) days+=dim(y,m);
  days+=(da-1);
  return days*86400UL+(uint32_t)h*3600UL+(uint32_t)mi*60UL+(uint32_t)s;
}
static void epochToYMD(uint32_t e,int &y,int &m,int &d){
  uint32_t days=e/86400UL; y=1970;
  while(true){ uint32_t dy=isLeap(y)?366:365; if(days>=dy){days-=dy;y++;} else break; }
  m=1; while(true){ uint8_t dm=dim(y,m); if(days>=dm){days-=dm;m++;} else break; }
  d=(int)days+1;
}
static String dailyLogPath(uint32_t epoch){
  int y,m,d; epochToYMD(epoch,y,m,d);
  char buf[48];
  snprintf(buf,sizeof(buf),"%s/log_%04d%02d%02d.csv",DIR_LOGS,y,m,d);
  return String(buf);
}

// Payload compression helpers (same as SerialTest)
static const char *rainUnitToken() { return RAIN_UNIT; }
static int32_t scaleRain100(float value){ return (int32_t)lroundf(value*100.0f); }
static int32_t scaleDeg1e7(double deg){ return (int32_t)llround(deg*10000000.0); }

// Build compact payload used for logging/queue (HTTP disabled in this file).
static String buildUploadPayload(const String &imei,
                                 uint32_t battMv,
                                 float rainDeltaMm,
                                 uint32_t epochNow,
                                 bool includeUnit,
                                 bool includeGps,
                                 double lat,
                                 double lon) {
  int32_t R = scaleRain100(rainDeltaMm);
  String s; s.reserve(includeGps?120:90);
  s += imei; s += "|";
  s += String((unsigned long)battMv); s += "|";
  s += String(R); s += "|";
  s += String((unsigned long)epochNow);
  if (includeUnit) {
    s += "|"; s += rainUnitToken();
  }
  if(includeGps){
    s += "|"; s += String(scaleDeg1e7(lat));
    s += "|"; s += String(scaleDeg1e7(lon));
  }
  return s;
}

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

static void modemBestEffortPowerDown() {
  DBG_PRINTLN("[MODEM] power down (software-only) ...");
  modem.sendAT("+CGPS=0"); modem.waitResponse(1000);
  modem.gprsDisconnect();
  modem.sendAT("+CFUN=0"); modem.waitResponse(2000);
  modem.sendAT("+CPOF");   modem.waitResponse(3000);
  modem.sendAT("+CSCLK=2"); modem.waitResponse(1000);
  digitalWrite(MODEM_DTR, HIGH);
  pulsePwrKey();
  SerialAT.end();
  digitalWrite(MODEM_FLIGHT, HIGH);
  DBG_PRINTLN("[MODEM] power down done");
}

static bool connectNetwork(String &iccid,String &imei,int &rssi) {
  rssi=-1;
  if(!modem.init()) {
    DBG_PRINTLN("[MODEM] modem.init failed");
    return false;
  }
#if defined(TINY_GSM_MODEM_HAS_GPS)
  DBG_PRINTLN("[GPS] Enabling GNSS...");
  modem.setGPSMode(1);
  delay(200);
#endif
  String tIccid=modem.getSimCCID(); if(tIccid.length()) iccid=tIccid;
  String tImei =modem.getIMEI();    if(tImei.length())  imei=tImei;
  DBG_PRINTF("[MODEM] ICCID: %s\n", iccid.c_str());
  DBG_PRINTF("[MODEM] IMEI : %s\n", imei.c_str());
  DBG_PRINT("[MODEM] Modem Info: ");
  DBG_PRINTLN(modem.getModemInfo());

  DBG_PRINTLN("[NET] Waiting for network registration (up to 60s)...");
  uint32_t start = millis();
  if(!modem.waitForNetwork(60000L)) {
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
  if(!modem.gprsConnect(APN,GPRS_USER,GPRS_PASS)) {
    uint32_t elapsed = elapsedMs(start);
    DBG_PRINTF("[NET] gprsConnect failed (%.1f s)\n", elapsed / 1000.0f);
    return false;
  }
  uint32_t gprsMs = elapsedMs(start);
  DBG_PRINTF("[NET] GPRS OK in %lu ms (%.1f s)\n",
             (unsigned long)gprsMs, gprsMs / 1000.0f);
  rssi=modem.getSignalQuality();
  DBG_PRINTF("[NET] RSSI (CSQ): %d\n", rssi);
  DBG_PRINT("[NET] Local IP : ");
  DBG_PRINTLN(modem.localIP());
  return true;
}

// Convert NMEA ddmm.mmmm or dddmm.mmmm into decimal degrees.
static double nmeaDdmmToDeg(const String &ddmm) {
  int dot = ddmm.indexOf('.');
  int degLen = (dot >= 0 && dot > 4) ? 3 : 2;     // lon often has 3 deg digits
  double deg = ddmm.substring(0, degLen).toDouble();
  double min = ddmm.substring(degLen).toDouble();
  return deg + (min / 60.0);
}

static bool parseCgpsInfo(const String &resp,double &latOut,double &lonOut,bool &hasEpoch,uint32_t &epochOut){
  int idx=resp.indexOf(":");
  String p=(idx>=0)?resp.substring(idx+1):resp; p.trim();
  if(p.startsWith(",")) return false;

  std::vector<String> parts; parts.reserve(10);
  int start=0;
  for(int i=0;i<=(int)p.length();i++){
    if(i==(int)p.length()||p[i]==','){
      parts.push_back(p.substring(start,i));
      start=i+1;
    }
  }
  if(parts.size()<4) return false;
  String latStr=parts[0]; latStr.trim();
  String ns=parts[1]; ns.trim();
  String lonStr=parts[2]; lonStr.trim();
  String ew=parts[3]; ew.trim();

  double lat=nmeaDdmmToDeg(latStr), lon=nmeaDdmmToDeg(lonStr);
  if(ns=="S") lat=-lat;
  if(ew=="W") lon=-lon;
  latOut=lat; lonOut=lon;

  hasEpoch=false; epochOut=0;
  if(parts.size()>=6){
    String date=parts[4]; date.trim();
    String time=parts[5]; time.trim();
    if(date.length()>=6 && time.length()>=6){
      int dd=date.substring(0,2).toInt();
      int mo=date.substring(2,4).toInt();
      int yy=date.substring(4,6).toInt();
      int year=2000+yy;
      int hh=time.substring(0,2).toInt();
      int mm=time.substring(2,4).toInt();
      int ss=time.substring(4,6).toInt();
      if(year>=2020 && mo>=1 && mo<=12 && dd>=1 && dd<=31 && hh<=23 && mm<=59 && ss<=59){
        epochOut=ymdhmsToEpoch(year,mo,dd,hh,mm,ss);
        hasEpoch=epochOut>0;
      }
    }
  }
  return true;
}

// Attempt GPS fix; returns lat/lon and (if available) UTC epoch.
static bool gpsAcquire(uint32_t timeoutMs,
                       double &latOut,
                       double &lonOut,
                       bool &hasEpoch,
                       uint32_t &epochOut,
                       uint32_t &fixTimeMsOut){
  hasEpoch=false; epochOut=0;
  modem.sendAT("+CGPS=1");
  uint32_t start=millis();
  if(modem.waitResponse(2000)!=1) {
    fixTimeMsOut = elapsedMs(start);
    return false;
  }
  uint32_t lastProgress = 0;
  while(elapsedMs(start)<timeoutMs){
    modem.sendAT("+CGPSINFO");
    String out;
    int r=modem.waitResponse(1200,out);
    if(r==1 && out.indexOf("+CGPSINFO")>=0){
      if(parseCgpsInfo(out,latOut,lonOut,hasEpoch,epochOut)){
        modem.sendAT("+CGPS=0"); modem.waitResponse(1000);
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
  modem.sendAT("+CGPS=0"); modem.waitResponse(1000);
  fixTimeMsOut = elapsedMs(start);
  return false;
}

static void appendDaily(const String &path,const String &line){
  bool isNew=!SD.exists(path);
  File f=SD.open(path, FILE_APPEND);
  if(!f) return;
  if(isNew) {
    f.println("payload");
  }
  f.println(line);
  f.close();
}

static String makeQueueFilename(uint32_t epoch,uint32_t wakeCounter){
  char buf[80];
  snprintf(buf,sizeof(buf),"%s/q_%lu_%lu.txt",DIR_QUEUE,(unsigned long)epoch,(unsigned long)wakeCounter);
  return String(buf);
}

// Normalize queue paths so files always resolve under DIR_QUEUE.
static String ensureQueuePath(const String &name){
  if(name.startsWith("/")) return name;
  return String(DIR_QUEUE)+"/"+name;
}

static void writeQueueLine(const String &path,const String &line){
  File f=SD.open(path, FILE_WRITE);
  if(!f) return;
  f.seek(0);
  f.println(line);
  f.close();
}

// Send HTTP payload and measure time spent (unused when ENABLE_HTTP=false).
static bool httpPostPayload(const String &line, uint32_t timeoutMs, uint32_t &durationMs, int &statusOut, String &responseOut){
  http.setHttpResponseTimeout(timeoutMs);
  uint32_t start = millis();
  http.beginRequest();
  http.post(SERVER_PATH);
  http.sendHeader("Content-Type","text/plain");
  http.sendHeader("Content-Length", line.length());
  http.beginBody();
  http.print(line);
  http.endRequest();
  statusOut=http.responseStatusCode();
  responseOut=http.responseBody();
  durationMs = elapsedMs(start);
  http.stop();
  if (durationMs > timeoutMs) return false;
  return (statusOut>=200 && statusOut<300);
}

static uint32_t computeHttpTimeoutMs(uint32_t lastHttpMs) {
  if (lastHttpMs == 0) return HTTP_TIMEOUT_DEFAULT_MS;
  uint64_t timeout = (uint64_t)lastHttpMs * HTTP_TIMEOUT_MULTIPLIER;
  if (timeout > HTTP_TIMEOUT_MAX_MS) timeout = HTTP_TIMEOUT_MAX_MS;
  return (uint32_t)timeout;
}

// Send a single queued file (unused when ENABLE_HTTP=false).
static bool sendQueueFile(const String &path, uint32_t timeoutMs, uint32_t &durationMs, bool &unitSent){
  File f=SD.open(path, FILE_READ);
  if(!f) return false;
  String line=f.readStringUntil('\n');
  f.close();
  int status=0;
  String response;
  bool ok=httpPostPayload(line, timeoutMs, durationMs, status, response);
  bool stored = response.indexOf("stored") >= 0;
  if(ok && stored) {
    if (!unitSent && payloadHasUnitToken(line)) {
      unitSent = true;
      writeBoolFile(FILE_RAIN_UNIT_SENT, true);
    }
    SD.remove(path.c_str());
  }
  return ok;
}

// Walk the queue folder in order and attempt to send each payload (unused here).
static void sendAllQueuedFiles(uint32_t &lastHttpMs, bool &unitSent){
  std::vector<String> files;
  File dir=SD.open(DIR_QUEUE);
  if(!dir) return;
  while(true){
    File f=dir.openNextFile();
    if(!f) break;
    if(!f.isDirectory()) files.push_back(ensureQueuePath(String(f.name())));
    f.close();
  }
  dir.close();
  std::sort(files.begin(),files.end(),[](const String&a,const String&b){return a<b;});
  for(auto &p:files){
    uint32_t durationMs = 0;
    uint32_t timeoutMs = computeHttpTimeoutMs(lastHttpMs);
    if(!sendQueueFile(p, timeoutMs, durationMs, unitSent)) {
      lastHttpMs = HTTP_TIMEOUT_MAX_MS;
      writeUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
      break;
    }
    lastHttpMs = durationMs;
    writeUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
    delay(HTTP_QUEUE_SEND_DELAY_MS);
  }
}

static float readRainTotalMm(){ return RainSensor.getRainfall(); }

static float rainDeltaInterval(float totalMm){
  float prev=totalMm;
  readFloatFile(FILE_RAIN_PREV_TOTAL, prev);
  float d=totalMm-prev;
  if(d<0) d=0;
  writeFloatFile(FILE_RAIN_PREV_TOTAL, totalMm);
  return d;
}

void setup(){
  DBG_BEGIN();
  setLowPowerCpu();
  disableRadiosForPower();

  // Wake counter and epoch estimate are stored in RTC memory.
  g_wakeCounter++;
  if(g_epochEstimate>0) g_epochEstimate += WAKE_INTERVAL_SECONDS;

  // Mount SD (critical for logs and state). If it fails, sleep.
  bool sdOk = initSD();
  DBG_PRINTF("[SD] init: %s\n", sdOk ? "OK" : "FAIL");
  if(!sdOk){
    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }

  // Load cached identity so we can build payload even if modem fails.
  String iccid="unknown", imei="unknown";
  loadIdentity(iccid, imei);

  // Load last known GPS (epoch + location) and timing state from SD.
  uint32_t lastGpsEpoch=0;
  double lastLat=0.0, lastLon=0.0;
  loadLastGps(lastGpsEpoch, lastLat, lastLon);
  uint32_t lastGpsFixMs = 0;
  readUInt32File(FILE_GPS_FIX_MS, lastGpsFixMs);
  uint32_t gpsRetryEpoch = 0;
  readUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);
  uint32_t lastHttpMs = 0;
  readUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
  bool unitSent = false;
  readBoolFile(FILE_RAIN_UNIT_SENT, unitSent);

  // Restore epoch estimate from SD if RTC was lost.
  if(g_epochEstimate==0 && lastGpsEpoch>0) g_epochEstimate=lastGpsEpoch;

  // Initialize rainfall sensor (I2C).
  Wire.begin();
  bool rainOk = RainSensor.begin();
  DBG_PRINTF("[RAIN] begin(): %s\n", rainOk ? "OK" : "FAIL");

  // Compute rainfall delta for this interval.
  float totalMm=readRainTotalMm();
  float deltaMm=rainDeltaInterval(totalMm);
  DBG_PRINTF("[RAIN] total=%.3f mm  delta5m=%.3f mm\n", totalMm, deltaMm);

  uint32_t battMvRaw=0;
  float battV=readBatteryVoltage(battMvRaw);
  uint32_t battMvVBAT=(uint32_t)lroundf(battV*1000.0f);
  DBG_PRINTF("[BATT] raw=%lu mV, scaled=%.3f V (~%lu mV)\n",
             (unsigned long)battMvRaw, battV, (unsigned long)battMvVBAT);

  // If modem can't power on, still log locally and queue payload.
  bool modemOk = modemPowerOn();
  DBG_PRINTF("[MODEM] Overall modem connect: %s\n", modemOk ? "OK" : "FAIL");
  if(!modemOk){
    uint32_t epochNow=(g_epochEstimate>0)?g_epochEstimate:lastGpsEpoch;
    String payload = buildUploadPayload(imei, battMvVBAT, deltaMm, epochNow,
                                        !unitSent, false, 0, 0);
    appendDaily(dailyLogPath(epochNow ? epochNow : 0), payload);
    String qPath=makeQueueFilename(epochNow?epochNow:g_wakeCounter,g_wakeCounter);
    writeQueueLine(qPath,payload);

    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }

  // Connect cellular network (for GPS and for parity with other builds).
  int rssi=-1;
  bool netOk=connectNetwork(iccid,imei,rssi);
  saveIdentity(iccid,imei);
  DBG_PRINTF("[NET] Registered: %s (RSSI=%d)\n", netOk ? "YES" : "NO", rssi);

  // Determine current epoch for this wake.
  uint32_t epochNow=g_epochEstimate;
  if(epochNow==0 && lastGpsEpoch>0) epochNow=lastGpsEpoch;

  // Decide if GPS refresh is due.
  bool haveValidEpoch = (epochNow > 0);
  bool gpsRetryDue = (gpsRetryEpoch > 0) && (epochNow >= gpsRetryEpoch);
  bool needGps = (!haveValidEpoch) ||
                 (lastGpsEpoch == 0) ||
                 (epochNow >= lastGpsEpoch && (epochNow - lastGpsEpoch) >= GPS_REFRESH_SECONDS) ||
                 gpsRetryDue ||
                 (g_wakeCounter == 1);

  bool gpsIncludedInUpload = false;

  if (modemOk && needGps) {
    double lat = 0.0, lon = 0.0;
    bool hasGpsEpoch = false;
    uint32_t gpsEpoch = 0;
    uint32_t fixTimeMs = 0;
    // Adaptive GPS timeout with a minimum floor so we always search >= 3 minutes.
    uint32_t gpsTimeoutMs = lastGpsFixMs > 0 ? (lastGpsFixMs * 2UL) : GPS_TIMEOUT_DEFAULT_MS;
    if (gpsTimeoutMs < GPS_TIMEOUT_MIN_MS) gpsTimeoutMs = GPS_TIMEOUT_MIN_MS;

    DBG_PRINTF("[GPS] refresh due; attempting fix (timeout %.1f min)...\n",
               gpsTimeoutMs / 60000.0f);
    bool gpsOk = gpsAcquire(gpsTimeoutMs, lat, lon, hasGpsEpoch, gpsEpoch, fixTimeMs);
    if (gpsOk) {
      lastLat = lat; lastLon = lon;
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
        if (epochNow == 0) epochNow = (uint32_t)(millis() / 1000UL);
        lastGpsEpoch = epochNow;
        if (g_epochEstimate == 0) g_epochEstimate = epochNow;
        DBG_PRINTF("[GPS] FIX OK: lat=%.7f lon=%.7f (no epoch, %.1f s)\n",
                   lastLat, lastLon, fixTimeMs / 1000.0f);
      }
      saveLastGps(lastGpsEpoch, lastLat, lastLon);
      gpsIncludedInUpload = true;
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

  // Final fallback if we still have no epoch.
  if(epochNow==0) epochNow=lastGpsEpoch;

  // Build compact payload for local queue storage.
  String payload = buildUploadPayload(imei, battMvVBAT, deltaMm, epochNow,
                                      !unitSent, gpsIncludedInUpload, lastLat, lastLon);
  appendDaily(dailyLogPath(epochNow ? epochNow : 0), payload);
  DBG_PRINTLN("[PAYLOAD] built:");
  DBG_PRINTLN(payload);

  String qPath=makeQueueFilename(epochNow?epochNow:g_wakeCounter,g_wakeCounter);
  writeQueueLine(qPath,payload);

  // HTTP intentionally disabled in this build.
  if(netOk && ENABLE_HTTP) {
    DBG_PRINTF("[HTTP] sending queued payloads (timeout %lu ms)...\n",
               (unsigned long)computeHttpTimeoutMs(lastHttpMs));
    sendAllQueuedFiles(lastHttpMs, unitSent);
  } else if (!ENABLE_HTTP) {
    DBG_PRINTLN("[HTTP] disabled");
  } else {
    DBG_PRINTLN("[HTTP] skipped (network not OK)");
  }

  // Housekeeping and power-down.
  purgeLogsIfNeeded();
  modemBestEffortPowerDown();

  DBG_PRINTLN("[SLEEP] entering deep sleep...");
  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
  DBG_FLUSH();
  esp_deep_sleep_start();
}

void loop(){}
