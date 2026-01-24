/************************************************************************************
Pavewise Mobile Rain Gauge — RELEASE BUILD (Functionally identical to SerialTest)

SUMMARY (what this build does every wake):
  1) Boots, downclocks CPU, disables WiFi/BT, mounts SD, and purges old logs
     if SD usage is above the threshold.
  2) Reads rainfall + battery voltage, then powers the modem and registers on
     the cellular network.
  3) Determines the current epoch: uses stored RTC estimate and re-anchors
     from GPS every 6 hours (or on first boot), with adaptive GPS timeout.
  4) Builds a compact payload (IMEI|batt_mv|rain_x100|epoch[|lat|lon]) and:
       - writes it to a daily CSV log
       - writes it to a queue file
       - attempts to POST all queued files with adaptive HTTP timeout
  5) Gracefully powers down the modem and deep-sleeps for 15 minutes.

This file is the SAME logic as SerialTestPavewise_NEW.
Only difference: ENABLE_DEBUG = 0, meaning no Serial printing occurs.

This reduces CPU time, UART activity, and prevents Serial from keeping the chip awake.
************************************************************************************/

#define ENABLE_DEBUG 0

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
#include <ArduinoHttpClient.h>
#include "DFRobot_RainfallSensor.h"

// ============================= USER SETTINGS =============================
// All timing and backend configuration lives here so cadence and server
// details can be changed without touching the rest of the code.
// Same settings as SerialTest (keep identical behavior)
static const uint32_t WAKE_INTERVAL_SECONDS = 15UL * 60UL;
static const uint32_t GPS_REFRESH_SECONDS   = 6UL * 3600UL;
// GPS fix timeout strategy:
//   - default is 10 minutes
//   - after a successful fix, next timeout = last_fix_time * 2
static const uint32_t GPS_TIMEOUT_DEFAULT_MS = 10UL * 60UL * 1000UL;
// If a fix fails, retry next wake (15 minutes).
static const uint32_t GPS_RETRY_SECONDS      = 15UL * 60UL;

// HTTP send strategy:
//   - timeout adapts based on last send duration (x5, capped).
//   - queue files are retried in order; failures stop the loop.
static const uint32_t HTTP_TIMEOUT_DEFAULT_MS = 30UL * 1000UL;
static const uint32_t HTTP_TIMEOUT_MAX_MS     = 120UL * 1000UL;
static const uint32_t HTTP_TIMEOUT_MULTIPLIER = 5UL;
static const bool ENABLE_HTTP = true;

static const char APN[]       = "hologram";
static const char GPRS_USER[] = "";
static const char GPRS_PASS[] = "";

static const char SERVER_HOST[] = "example.com";   // TODO
static const int  SERVER_PORT   = 80;              // TODO
static const char SERVER_PATH[] = "/ingest";       // TODO

static const float SD_PURGE_START_PCT  = 80.0f;
static const float SD_PURGE_TARGET_PCT = 70.0f;

// Pins identical to SerialTest
#define UART_BAUD     115200
#define MODEM_TX      27
#define MODEM_RX      26
#define MODEM_PWRKEY  4
#define MODEM_DTR     32
#define MODEM_RI      33
#define MODEM_FLIGHT  25
#define MODEM_STATUS  34

// LILYGO SIM7600 reference timings:
// MODEM_POWERON_PULSE_WIDTH_MS = 500, MODEM_START_WAIT_MS = 15000.
static const uint32_t MODEM_PWRKEY_PREP_MS  = 100;
static const uint32_t MODEM_PWRKEY_PULSE_MS = 500;
static const uint32_t MODEM_BOOT_WAIT_MS    = 15000;
static const uint32_t MODEM_POWEROFF_PULSE_MS = 3000;

#define BAT_ADC_PIN   35
#define SD_MISO       2
#define SD_MOSI       15
#define SD_SCLK       14
#define SD_CS         13

// SD layout identical
static const char *DIR_LOGS  = "/logs";
static const char *DIR_QUEUE = "/queue";
static const char *DIR_STATE = "/state";
static const char *FILE_RAIN_PREV_TOTAL = "/state/rain_prev_total_mm.txt";
static const char *FILE_GPS_LAST        = "/state/gps_last.txt";
static const char *FILE_GPS_FIX_MS      = "/state/gps_fix_ms.txt";
static const char *FILE_GPS_RETRY_EPOCH = "/state/gps_retry_epoch.txt";
static const char *FILE_IDENTITY        = "/state/identity.txt";
static const char *FILE_HTTP_LAST_MS    = "/state/http_last_ms.txt";

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

RTC_DATA_ATTR uint32_t g_wakeCounter   = 0;
RTC_DATA_ATTR uint32_t g_epochEstimate = 0;

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsm(modem);
HttpClient http(gsm, SERVER_HOST, SERVER_PORT);

DFRobot_RainfallSensor_I2C RainSensor(&Wire);

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
  if (sdUsedPercent() < SD_PURGE_START_PCT) return;
  while (sdUsedPercent() > SD_PURGE_TARGET_PCT) {
    String oldest;
    if (!findOldestDailyLog(oldest)) break;
    SD.remove(oldest.c_str());
  }
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
static int32_t scaleRain100(float mm){ return (int32_t)lroundf(mm*100.0f); }
static int32_t scaleDeg1e7(double deg){ return (int32_t)llround(deg*10000000.0); }

// Build compact payload used for HTTP and queue files.
static String buildUploadPayload(const String &imei,
                                 uint32_t battMv,
                                 float rainDeltaMm,
                                 uint32_t epochNow,
                                 bool includeGps,
                                 double lat,
                                 double lon) {
  int32_t R = scaleRain100(rainDeltaMm);
  String s; s.reserve(includeGps?120:90);
  s += imei; s += "|";
  s += String((unsigned long)battMv); s += "|";
  s += String(R); s += "|";
  s += String((unsigned long)epochNow);
  if(includeGps){
    s += "|"; s += String(scaleDeg1e7(lat));
    s += "|"; s += String(scaleDeg1e7(lon));
  }
  return s;
}

static bool modemPowerOn() {
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
    return true;
  }

  return false;
}

static void modemBestEffortPowerDown() {
  modem.sendAT("+CGPS=0"); modem.waitResponse(1000);
  modem.gprsDisconnect();
  modem.sendAT("+CFUN=0"); modem.waitResponse(2000);
  modem.sendAT("+CPOF");   modem.waitResponse(3000);
  modem.sendAT("+CSCLK=2"); modem.waitResponse(1000);
  digitalWrite(MODEM_DTR, HIGH);
  pulsePwrKey();
  SerialAT.end();
  digitalWrite(MODEM_FLIGHT, HIGH);
}

static bool connectNetwork(String &iccid,String &imei,int &rssi) {
  rssi=-1;
  if(!modem.init()) return false;
#if defined(TINY_GSM_MODEM_HAS_GPS)
  uint8_t gnssMode = modem.getGNSSMode();
  DBG_PRINTF("[GPS] GNSS Mode (before): %u\n", gnssMode);
  modem.setGNSSMode(1, 1);
  delay(200);
  DBG_PRINTF("[GPS] GNSS Mode (after) : %u\n", modem.getGNSSMode());
#endif
  String tIccid=modem.getSimCCID(); if(tIccid.length()) iccid=tIccid;
  String tImei =modem.getIMEI();    if(tImei.length())  imei=tImei;
  if(!modem.waitForNetwork(60000L)) return false;
  if(!modem.gprsConnect(APN,GPRS_USER,GPRS_PASS)) return false;
  rssi=modem.getSignalQuality();
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
    f.println("imei,batt_mv,rain_mm_x100,epoch_utc,lat_deg_x1e7,lon_deg_x1e7,gps_fresh");
  }
  f.println(line);
  f.close();
}

static String makeQueueFilename(uint32_t epoch,uint32_t wakeCounter){
  char buf[80];
  snprintf(buf,sizeof(buf),"%s/q_%lu_%lu.txt",DIR_QUEUE,(unsigned long)epoch,(unsigned long)wakeCounter);
  return String(buf);
}

static void writeQueueLine(const String &path,const String &line){
  File f=SD.open(path, FILE_WRITE);
  if(!f) return;
  f.seek(0);
  f.println(line);
  f.close();
}

// Send HTTP payload and measure time spent (used to adapt next timeout).
static bool httpPostPayload(const String &line, uint32_t timeoutMs, uint32_t &durationMs, int &statusOut){
  http.setHttpResponseTimeout(timeoutMs);
  uint32_t start = millis();
  http.connectionKeepAlive();
  http.beginRequest();
  http.post(SERVER_PATH);
  http.sendHeader("Content-Type","text/plain");
  http.sendHeader("Content-Length", line.length());
  http.beginBody();
  http.print(line);
  http.endRequest();
  statusOut = http.responseStatusCode();
  (void)http.responseBody();
  durationMs = elapsedMs(start);
  if (durationMs > timeoutMs) return false;
  return (statusOut>=200 && statusOut<300);
}

static uint32_t computeHttpTimeoutMs(uint32_t lastHttpMs) {
  if (lastHttpMs == 0) return HTTP_TIMEOUT_DEFAULT_MS;
  uint64_t timeout = (uint64_t)lastHttpMs * HTTP_TIMEOUT_MULTIPLIER;
  if (timeout > HTTP_TIMEOUT_MAX_MS) timeout = HTTP_TIMEOUT_MAX_MS;
  return (uint32_t)timeout;
}

// Send a single queued file (delete only on success).
static bool sendQueueFile(const String &path, uint32_t timeoutMs, uint32_t &durationMs){
  File f=SD.open(path, FILE_READ);
  if(!f) return false;
  String line=f.readStringUntil('\n');
  f.close();
  int status=0;
  bool ok=httpPostPayload(line, timeoutMs, durationMs, status);
  if(ok) SD.remove(path.c_str());
  return ok;
}

// Walk the queue folder in order and attempt to send each payload.
static void sendAllQueuedFiles(uint32_t &lastHttpMs){
  std::vector<String> files;
  File dir=SD.open(DIR_QUEUE);
  if(!dir) return;
  while(true){
    File f=dir.openNextFile();
    if(!f) break;
    if(!f.isDirectory()) files.push_back(String(f.name()));
    f.close();
  }
  dir.close();
  std::sort(files.begin(),files.end(),[](const String&a,const String&b){return a<b;});
  for(auto &p:files){
    uint32_t timeoutMs = computeHttpTimeoutMs(lastHttpMs);
    uint32_t durationMs = 0;
    if(!sendQueueFile(p, timeoutMs, durationMs)) {
      if (durationMs > 0) {
        lastHttpMs = durationMs;
        writeUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
      }
      break;
    }
    lastHttpMs = durationMs;
    writeUInt32File(FILE_HTTP_LAST_MS, lastHttpMs);
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
  if(!initSD()){
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

  // Restore epoch estimate from SD if RTC was lost.
  if(g_epochEstimate==0 && lastGpsEpoch>0) g_epochEstimate=lastGpsEpoch;

  // Initialize rainfall sensor (I2C).
  Wire.begin();
  (void)RainSensor.begin();

  // Compute rainfall delta for this interval.
  float totalMm=readRainTotalMm();
  float deltaMm=rainDeltaInterval(totalMm);

  uint32_t battMvRaw=0;
  float battV=readBatteryVoltage(battMvRaw);
  uint32_t battMvVBAT=(uint32_t)lroundf(battV*1000.0f);

  // If modem can't power on, still log locally and queue payload.
  if(!modemPowerOn()){
    uint32_t epochNow=(g_epochEstimate>0)?g_epochEstimate:lastGpsEpoch;
    char dailyLine[256];
    snprintf(dailyLine, sizeof(dailyLine),
             "%s,%lu,%ld,%lu,,,%d",
             imei.c_str(),
             (unsigned long)battMvVBAT,
             (long)scaleRain100(deltaMm),
             (unsigned long)epochNow,
             0);
    appendDaily(dailyLogPath(epochNow ? epochNow : 0), String(dailyLine));

    String payload = buildUploadPayload(imei, battMvVBAT, deltaMm, epochNow, false, 0, 0);
    String qPath=makeQueueFilename(epochNow?epochNow:g_wakeCounter,g_wakeCounter);
    writeQueueLine(qPath,payload);

    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }

  // Connect cellular network (for GPS and HTTP).
  int rssi=-1;
  bool netOk=connectNetwork(iccid,imei,rssi);
  saveIdentity(iccid,imei);

  // Determine current epoch for this wake.
  uint32_t epochNow=g_epochEstimate;
  if(epochNow==0 && lastGpsEpoch>0) epochNow=lastGpsEpoch;

  // Decide if GPS refresh is due.
  bool haveValidEpoch = (epochNow > 0);
  bool gpsRetryDue = (gpsRetryEpoch > 0) && (epochNow >= gpsRetryEpoch);
  bool needGps = (!haveValidEpoch) ||
                 (lastGpsEpoch == 0) ||
                 (epochNow >= lastGpsEpoch && (epochNow - lastGpsEpoch) >= GPS_REFRESH_SECONDS) ||
                 gpsRetryDue;

  bool gpsIncludedInUpload = false;

  if (modemOk && needGps) {
    double lat = 0.0, lon = 0.0;
    bool hasGpsEpoch = false;
    uint32_t gpsEpoch = 0;
    uint32_t fixTimeMs = 0;
    uint32_t gpsTimeoutMs = lastGpsFixMs > 0 ? (lastGpsFixMs * 2UL) : GPS_TIMEOUT_DEFAULT_MS;

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
      } else {
        if (epochNow == 0) epochNow = (uint32_t)(millis() / 1000UL);
        lastGpsEpoch = epochNow;
        if (g_epochEstimate == 0) g_epochEstimate = epochNow;
      }

      saveLastGps(lastGpsEpoch, lastLat, lastLon);
      gpsIncludedInUpload = true;
    } else {
      uint32_t baseEpoch = epochNow;
      if (baseEpoch == 0 && g_epochEstimate > 0) baseEpoch = g_epochEstimate;
      if (baseEpoch == 0) baseEpoch = (uint32_t)(millis() / 1000UL);
      gpsRetryEpoch = baseEpoch + GPS_RETRY_SECONDS;
      writeUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);
    }
  } else if (needGps && !modemOk) {
    uint32_t baseEpoch = epochNow;
    if (baseEpoch == 0 && g_epochEstimate > 0) baseEpoch = g_epochEstimate;
    if (baseEpoch == 0) baseEpoch = (uint32_t)(millis() / 1000UL);
    gpsRetryEpoch = baseEpoch + GPS_RETRY_SECONDS;
    writeUInt32File(FILE_GPS_RETRY_EPOCH, gpsRetryEpoch);
  }

  // Final fallback if we still have no epoch.
  if(epochNow==0) epochNow=lastGpsEpoch;

  char dailyLine[256];
  if (gpsIncludedInUpload) {
    snprintf(dailyLine, sizeof(dailyLine),
             "%s,%lu,%ld,%lu,%ld,%ld,%d",
             imei.c_str(),
             (unsigned long)battMvVBAT,
             (long)scaleRain100(deltaMm),
             (unsigned long)epochNow,
             (long)scaleDeg1e7(lastLat),
             (long)scaleDeg1e7(lastLon),
             1);
  } else {
    snprintf(dailyLine, sizeof(dailyLine),
             "%s,%lu,%ld,%lu,,,%d",
             imei.c_str(),
             (unsigned long)battMvVBAT,
             (long)scaleRain100(deltaMm),
             (unsigned long)epochNow,
             0);
  }
  // Daily CSV log (human readable).
  appendDaily(dailyLogPath(epochNow ? epochNow : 0), String(dailyLine));

  // Build compact payload for HTTP and queue.
  String payload = buildUploadPayload(imei, battMvVBAT, deltaMm, epochNow,
                                      gpsIncludedInUpload, lastLat, lastLon);

  String qPath=makeQueueFilename(epochNow?epochNow:g_wakeCounter,g_wakeCounter);
  writeQueueLine(qPath,payload);

  // Send queued payloads if HTTP is enabled and network is OK.
  if(netOk && ENABLE_HTTP) sendAllQueuedFiles(lastHttpMs);

  // Housekeeping and power-down.
  purgeLogsIfNeeded();
  modemBestEffortPowerDown();

  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
  DBG_FLUSH();
  esp_deep_sleep_start();
}

void loop(){}
