/************************************************************************************
Pavewise Mobile Rain Gauge — RELEASE BUILD (Functionally identical to SerialTest)

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

// Same settings as SerialTest (keep identical behavior)
static const uint32_t WAKE_INTERVAL_SECONDS = 15UL * 60UL;
static const uint32_t GPS_REFRESH_SECONDS   = 6UL * 3600UL;

static const char APN[]       = "hologram";
static const char GPRS_USER[] = "";
static const char GPRS_PASS[] = "";

static const char SERVER_HOST[] = "example.com";   // TODO
static const int  SERVER_PORT   = 80;              // TODO
static const char SERVER_PATH[] = "/ingest";       // TODO

static const float SD_PURGE_START_PCT  = 80.0f;
static const float SD_PURGE_TARGET_PCT = 75.0f;

// Pins identical to SerialTest
#define UART_BAUD     115200
#define MODEM_TX      27
#define MODEM_RX      26
#define MODEM_PWRKEY  4
#define MODEM_DTR     32
#define MODEM_FLIGHT  25
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
static const char *FILE_IDENTITY        = "/state/identity.txt";

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

static void disableRadiosForPower() {
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();
}

static void setLowPowerCpu() { setCpuFrequencyMhz(80); }

static void pulsePwrKey(uint32_t holdMs = 1500) {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(holdMs);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
}

static bool initSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) return false;
  if (!SD.exists(DIR_LOGS))  SD.mkdir(DIR_LOGS);
  if (!SD.exists(DIR_QUEUE)) SD.mkdir(DIR_QUEUE);
  if (!SD.exists(DIR_STATE)) SD.mkdir(DIR_STATE);
  return true;
}

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

static void purgeLogsIfNeeded() {
  if (sdUsedPercent() < SD_PURGE_START_PCT) return;
  while (sdUsedPercent() > SD_PURGE_TARGET_PCT) {
    String oldest;
    if (!findOldestDailyLog(oldest)) break;
    SD.remove(oldest.c_str());
  }
}

static bool readTextFile(const char *path, String &out) {
  if (!SD.exists(path)) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  out = f.readStringUntil('\n');
  f.close();
  out.trim();
  return out.length() > 0;
}

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

static String buildUploadPayload(uint32_t epochNow,uint32_t wakeCounter,float rainDeltaMm,
                                 uint32_t battMv,int rssi,const String &imei,
                                 bool includeGps,double lat,double lon) {
  int32_t R = scaleRain100(rainDeltaMm);
  String s; s.reserve(includeGps?120:90);
  s += String(epochNow); s += "|";
  s += String(wakeCounter); s += "|";
  s += String(R); s += "|";
  s += String((unsigned long)battMv); s += "|";
  s += String(rssi); s += "|";
  s += imei;
  if(includeGps){
    s += "|"; s += String(scaleDeg1e7(lat));
    s += "|"; s += String(scaleDeg1e7(lon));
  }
  return s;
}

static bool modemPowerOn() {
  pinMode(MODEM_FLIGHT, OUTPUT); digitalWrite(MODEM_FLIGHT, LOW);
  pinMode(MODEM_DTR, OUTPUT);    digitalWrite(MODEM_DTR, LOW);
  pulsePwrKey(1500);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  uint32_t start=millis();
  while(elapsedMs(start)<12000){
    modem.sendAT("AT");
    if(modem.waitResponse(250)==1) return true;
    delay(250);
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
  pulsePwrKey(1500);
  SerialAT.end();
  digitalWrite(MODEM_FLIGHT, HIGH);
}

static bool connectNetwork(String &iccid,String &imei,int &rssi) {
  rssi=-1;
  if(!modem.init()) return false;
  String tIccid=modem.getSimCCID(); if(tIccid.length()) iccid=tIccid;
  String tImei =modem.getIMEI();    if(tImei.length())  imei=tImei;
  if(!modem.waitForNetwork(60000L)) return false;
  if(!modem.gprsConnect(APN,GPRS_USER,GPRS_PASS)) return false;
  rssi=modem.getSignalQuality();
  return true;
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

  auto nmeaToDeg=[](const String &ddmm)->double{
    int dot=ddmm.indexOf('.');
    int degLen=(dot>=0&&dot>4)?3:2;
    double deg=ddmm.substring(0,degLen).toDouble();
    double min=ddmm.substring(degLen).toDouble();
    return deg+min/60.0;
  };
  double lat=nmeaToDeg(latStr), lon=nmeaToDeg(lonStr);
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

static bool gpsAcquire(uint32_t timeoutMs,double &latOut,double &lonOut,bool &hasEpoch,uint32_t &epochOut){
  hasEpoch=false; epochOut=0;
  modem.sendAT("+CGPS=1");
  if(modem.waitResponse(2000)!=1) return false;
  uint32_t start=millis();
  while(elapsedMs(start)<timeoutMs){
    modem.sendAT("+CGPSINFO");
    String out;
    int r=modem.waitResponse(1200,out);
    if(r==1 && out.indexOf("+CGPSINFO")>=0){
      if(parseCgpsInfo(out,latOut,lonOut,hasEpoch,epochOut)){
        modem.sendAT("+CGPS=0"); modem.waitResponse(1000);
        return true;
      }
    }
    delay(2000);
  }
  modem.sendAT("+CGPS=0"); modem.waitResponse(1000);
  return false;
}

static void appendDaily(const String &path,const String &line){
  bool isNew=!SD.exists(path);
  File f=SD.open(path, FILE_APPEND);
  if(!f) return;
  if(isNew) f.println("epoch_utc,lat,lon,rain_mm_15m,batt_v,iccid,imei,rssi,gps_included_upload");
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

static bool httpPostPayload(const String &line,int &statusOut){
  http.connectionKeepAlive();
  http.beginRequest();
  http.post(SERVER_PATH);
  http.sendHeader("Content-Type","text/plain");
  http.sendHeader("Content-Length", line.length());
  http.beginBody();
  http.print(line);
  http.endRequest();
  statusOut=http.responseStatusCode();
  (void)http.responseBody();
  return (statusOut>=200 && statusOut<300);
}

static bool sendQueueFile(const String &path){
  File f=SD.open(path, FILE_READ);
  if(!f) return false;
  String line=f.readStringUntil('\n');
  f.close();
  int status=0;
  bool ok=httpPostPayload(line,status);
  if(ok) SD.remove(path.c_str());
  return ok;
}

static void sendAllQueuedFiles(){
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
    if(!sendQueueFile(p)) break;
  }
}

static float readRainTotalMm(){ return RainSensor.getRainfall(); }

static float rainDelta15m(float totalMm){
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

  g_wakeCounter++;
  if(g_epochEstimate>0) g_epochEstimate += WAKE_INTERVAL_SECONDS;

  if(!initSD()){
    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }

  String iccid="unknown", imei="unknown";
  loadIdentity(iccid, imei);

  uint32_t lastGpsEpoch=0;
  double lastLat=0.0, lastLon=0.0;
  loadLastGps(lastGpsEpoch, lastLat, lastLon);

  if(g_epochEstimate==0 && lastGpsEpoch>0) g_epochEstimate=lastGpsEpoch;

  Wire.begin();
  (void)RainSensor.begin();

  float totalMm=readRainTotalMm();
  float deltaMm=rainDelta15m(totalMm);

  uint32_t battMvRaw=0;
  float battV=readBatteryVoltage(battMvRaw);
  uint32_t battMvVBAT=(uint32_t)lroundf(battV*1000.0f);

  if(!modemPowerOn()){
    uint32_t epochNow=(g_epochEstimate>0)?g_epochEstimate:lastGpsEpoch;
    char dailyLine[256];
    snprintf(dailyLine,sizeof(dailyLine),
             "%lu,%.7f,%.7f,%.3f,%.3f,%s,%s,%d,%d",
             (unsigned long)epochNow,lastLat,lastLon,deltaMm,battV,
             iccid.c_str(),imei.c_str(),-1,0);
    appendDaily(dailyLogPath(epochNow?epochNow:0), String(dailyLine));

    String payload=buildUploadPayload(epochNow,g_wakeCounter,deltaMm,battMvVBAT,-1,imei,false,0,0);
    String qPath=makeQueueFilename(epochNow?epochNow:g_wakeCounter,g_wakeCounter);
    writeQueueLine(qPath,payload);

    esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
    DBG_FLUSH();
    esp_deep_sleep_start();
  }

  int rssi=-1;
  bool netOk=connectNetwork(iccid,imei,rssi);
  saveIdentity(iccid,imei);

  uint32_t epochNow=g_epochEstimate;
  if(epochNow==0 && lastGpsEpoch>0) epochNow=lastGpsEpoch;

  bool haveValidEpoch=(epochNow>0);
  bool needGps=(!haveValidEpoch) ||
               (lastGpsEpoch==0) ||
               (epochNow>=lastGpsEpoch && (epochNow-lastGpsEpoch)>=GPS_REFRESH_SECONDS);

  bool gpsIncludedInUpload=false;

  if(netOk && needGps){
    double lat=0.0, lon=0.0;
    bool hasGpsEpoch=false; uint32_t gpsEpoch=0;
    bool gpsOk=gpsAcquire(8UL*60UL*1000UL,lat,lon,hasGpsEpoch,gpsEpoch);
    if(gpsOk){
      lastLat=lat; lastLon=lon;
      if(hasGpsEpoch){
        lastGpsEpoch=gpsEpoch;
        g_epochEstimate=gpsEpoch;
        epochNow=gpsEpoch;
      } else {
        if(epochNow==0) epochNow=(uint32_t)(millis()/1000UL);
        lastGpsEpoch=epochNow;
        if(g_epochEstimate==0) g_epochEstimate=epochNow;
      }
      saveLastGps(lastGpsEpoch,lastLat,lastLon);
      gpsIncludedInUpload=true;
    }
  }

  if(epochNow==0) epochNow=lastGpsEpoch;

  char dailyLine[256];
  snprintf(dailyLine,sizeof(dailyLine),
           "%lu,%.7f,%.7f,%.3f,%.3f,%s,%s,%d,%d",
           (unsigned long)epochNow,lastLat,lastLon,deltaMm,battV,
           iccid.c_str(),imei.c_str(),rssi,gpsIncludedInUpload?1:0);
  appendDaily(dailyLogPath(epochNow?epochNow:0), String(dailyLine));

  String payload=buildUploadPayload(epochNow,g_wakeCounter,deltaMm,battMvVBAT,rssi,imei,
                                    gpsIncludedInUpload,lastLat,lastLon);

  String qPath=makeQueueFilename(epochNow?epochNow:g_wakeCounter,g_wakeCounter);
  writeQueueLine(qPath,payload);

  if(netOk) sendAllQueuedFiles();

  purgeLogsIfNeeded();
  modemBestEffortPowerDown();

  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_SECONDS*1000000ULL);
  DBG_FLUSH();
  esp_deep_sleep_start();
}

void loop(){}

