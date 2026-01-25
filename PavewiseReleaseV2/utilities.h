#ifndef PAVEWISE_UTILITIES_H
#define PAVEWISE_UTILITIES_H

#include <Arduino.h>

// ============================= USER SETTINGS =============================
// All user-configurable settings live in this file so you can adjust APN,
// server details, timing, and pin mappings without editing the main sketches.

// Wake interval (seconds).
#ifndef PAVEWISE_WAKE_INTERVAL_SECONDS
#define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)
#endif

// GPS refresh interval (seconds).
#ifndef PAVEWISE_GPS_REFRESH_SECONDS
#define PAVEWISE_GPS_REFRESH_SECONDS (6UL * 3600UL)
#endif

// GPS fix timeout strategy:
//   - default is 10 minutes
//   - after a successful fix, next timeout = last_fix_time * 2
#ifndef PAVEWISE_GPS_TIMEOUT_DEFAULT_MS
#define PAVEWISE_GPS_TIMEOUT_DEFAULT_MS (10UL * 60UL * 1000UL)
#endif
#ifndef PAVEWISE_GPS_TIMEOUT_MIN_MS
// Ensure GPS attempts always run for at least 3 minutes.
#define PAVEWISE_GPS_TIMEOUT_MIN_MS (3UL * 60UL * 1000UL)
#endif

// If a fix fails, retry next wake (seconds).
#ifndef PAVEWISE_GPS_RETRY_SECONDS
#define PAVEWISE_GPS_RETRY_SECONDS (15UL * 60UL)
#endif

// HTTP send strategy:
//   - timeout adapts based on last send duration (x5, capped).
//   - queue files are retried in order; failures stop the loop.
#ifndef PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS
#define PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS (30UL * 1000UL)
#endif

#ifndef PAVEWISE_HTTP_TIMEOUT_MAX_MS
#define PAVEWISE_HTTP_TIMEOUT_MAX_MS (120UL * 1000UL)
#endif

#ifndef PAVEWISE_HTTP_TIMEOUT_MULTIPLIER
#define PAVEWISE_HTTP_TIMEOUT_MULTIPLIER (5UL)
#endif

#ifndef PAVEWISE_ENABLE_HTTP
#define PAVEWISE_ENABLE_HTTP true
#endif

// Cellular APN settings.
#ifndef PAVEWISE_APN
#define PAVEWISE_APN "hologram"
#endif

#ifndef PAVEWISE_GPRS_USER
#define PAVEWISE_GPRS_USER ""
#endif

#ifndef PAVEWISE_GPRS_PASS
#define PAVEWISE_GPRS_PASS ""
#endif

// Web server settings.
#ifndef PAVEWISE_SERVER_HOST
#define PAVEWISE_SERVER_HOST "18.218.149.57"
#endif

#ifndef PAVEWISE_SERVER_PORT
#define PAVEWISE_SERVER_PORT 8080
#endif

#ifndef PAVEWISE_SERVER_PATH
#define PAVEWISE_SERVER_PATH "/ingest"
#endif

// SD purge thresholds.
#ifndef PAVEWISE_SD_PURGE_START_PCT
#define PAVEWISE_SD_PURGE_START_PCT 80.0f
#endif

#ifndef PAVEWISE_SD_PURGE_TARGET_PCT
#define PAVEWISE_SD_PURGE_TARGET_PCT 70.0f
#endif

// ============================= BOARD PINS =============================

// UART settings
#ifndef PAVEWISE_UART_BAUD
#define PAVEWISE_UART_BAUD 115200
#endif

// ESP32 UART -> SIM7600
#ifndef PAVEWISE_MODEM_TX
#define PAVEWISE_MODEM_TX 27
#endif

#ifndef PAVEWISE_MODEM_RX
#define PAVEWISE_MODEM_RX 26
#endif

// SIM7600 control pins (typical LilyGO mapping)
#ifndef PAVEWISE_MODEM_PWRKEY
#define PAVEWISE_MODEM_PWRKEY 4
#endif

#ifndef PAVEWISE_MODEM_DTR
#define PAVEWISE_MODEM_DTR 32
#endif

#ifndef PAVEWISE_MODEM_RI
#define PAVEWISE_MODEM_RI 33
#endif

#ifndef PAVEWISE_MODEM_FLIGHT
#define PAVEWISE_MODEM_FLIGHT 25
#endif

#ifndef PAVEWISE_MODEM_STATUS
#define PAVEWISE_MODEM_STATUS 34
#endif

// LILYGO SIM7600 reference timings:
// MODEM_POWERON_PULSE_WIDTH_MS = 500, MODEM_START_WAIT_MS = 15000.
#ifndef PAVEWISE_MODEM_PWRKEY_PREP_MS
#define PAVEWISE_MODEM_PWRKEY_PREP_MS 100
#endif

#ifndef PAVEWISE_MODEM_PWRKEY_PULSE_MS
#define PAVEWISE_MODEM_PWRKEY_PULSE_MS 500
#endif

#ifndef PAVEWISE_MODEM_BOOT_WAIT_MS
#define PAVEWISE_MODEM_BOOT_WAIT_MS 15000
#endif

#ifndef PAVEWISE_MODEM_POWEROFF_PULSE_MS
#define PAVEWISE_MODEM_POWEROFF_PULSE_MS 3000
#endif

// Battery ADC pin (typical on LilyGO)
#ifndef PAVEWISE_BAT_ADC_PIN
#define PAVEWISE_BAT_ADC_PIN 35
#endif

// SD SPI pins
#ifndef PAVEWISE_SD_MISO
#define PAVEWISE_SD_MISO 2
#endif

#ifndef PAVEWISE_SD_MOSI
#define PAVEWISE_SD_MOSI 15
#endif

#ifndef PAVEWISE_SD_SCLK
#define PAVEWISE_SD_SCLK 14
#endif

#ifndef PAVEWISE_SD_CS
#define PAVEWISE_SD_CS 13
#endif

// ============================= SD FILE STRUCTURE =============================

#ifndef PAVEWISE_DIR_LOGS
#define PAVEWISE_DIR_LOGS "/logs"
#endif

#ifndef PAVEWISE_DIR_QUEUE
#define PAVEWISE_DIR_QUEUE "/queue"
#endif

#ifndef PAVEWISE_DIR_STATE
#define PAVEWISE_DIR_STATE "/state"
#endif

#ifndef PAVEWISE_FILE_RAIN_PREV_TOTAL
#define PAVEWISE_FILE_RAIN_PREV_TOTAL "/state/rain_prev_total_mm.txt"
#endif

#ifndef PAVEWISE_FILE_GPS_LAST
#define PAVEWISE_FILE_GPS_LAST "/state/gps_last.txt"
#endif

#ifndef PAVEWISE_FILE_GPS_FIX_MS
#define PAVEWISE_FILE_GPS_FIX_MS "/state/gps_fix_ms.txt"
#endif

#ifndef PAVEWISE_FILE_GPS_RETRY_EPOCH
#define PAVEWISE_FILE_GPS_RETRY_EPOCH "/state/gps_retry_epoch.txt"
#endif

#ifndef PAVEWISE_FILE_IDENTITY
#define PAVEWISE_FILE_IDENTITY "/state/identity.txt"
#endif

#ifndef PAVEWISE_FILE_HTTP_LAST_MS
#define PAVEWISE_FILE_HTTP_LAST_MS "/state/http_last_ms.txt"
#endif

// ============================= CONFIG CONSTANTS =============================

static const uint32_t WAKE_INTERVAL_SECONDS = PAVEWISE_WAKE_INTERVAL_SECONDS;
static const uint32_t GPS_REFRESH_SECONDS   = PAVEWISE_GPS_REFRESH_SECONDS;
static const uint32_t GPS_TIMEOUT_DEFAULT_MS = PAVEWISE_GPS_TIMEOUT_DEFAULT_MS;
static const uint32_t GPS_TIMEOUT_MIN_MS     = PAVEWISE_GPS_TIMEOUT_MIN_MS;
static const uint32_t GPS_RETRY_SECONDS      = PAVEWISE_GPS_RETRY_SECONDS;

static const uint32_t HTTP_TIMEOUT_DEFAULT_MS = PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS;
static const uint32_t HTTP_TIMEOUT_MAX_MS     = PAVEWISE_HTTP_TIMEOUT_MAX_MS;
static const uint32_t HTTP_TIMEOUT_MULTIPLIER = PAVEWISE_HTTP_TIMEOUT_MULTIPLIER;
static const bool ENABLE_HTTP = PAVEWISE_ENABLE_HTTP;

static const char APN[]       = PAVEWISE_APN;
static const char GPRS_USER[] = PAVEWISE_GPRS_USER;
static const char GPRS_PASS[] = PAVEWISE_GPRS_PASS;

static const char SERVER_HOST[] = PAVEWISE_SERVER_HOST;
static const int  SERVER_PORT   = PAVEWISE_SERVER_PORT;
static const char SERVER_PATH[] = PAVEWISE_SERVER_PATH;

static const float SD_PURGE_START_PCT  = PAVEWISE_SD_PURGE_START_PCT;
static const float SD_PURGE_TARGET_PCT = PAVEWISE_SD_PURGE_TARGET_PCT;

#define UART_BAUD     PAVEWISE_UART_BAUD
#define MODEM_TX      PAVEWISE_MODEM_TX
#define MODEM_RX      PAVEWISE_MODEM_RX
#define MODEM_PWRKEY  PAVEWISE_MODEM_PWRKEY
#define MODEM_DTR     PAVEWISE_MODEM_DTR
#define MODEM_RI      PAVEWISE_MODEM_RI
#define MODEM_FLIGHT  PAVEWISE_MODEM_FLIGHT
#define MODEM_STATUS  PAVEWISE_MODEM_STATUS

static const uint32_t MODEM_PWRKEY_PREP_MS  = PAVEWISE_MODEM_PWRKEY_PREP_MS;
static const uint32_t MODEM_PWRKEY_PULSE_MS = PAVEWISE_MODEM_PWRKEY_PULSE_MS;
static const uint32_t MODEM_BOOT_WAIT_MS    = PAVEWISE_MODEM_BOOT_WAIT_MS;
static const uint32_t MODEM_POWEROFF_PULSE_MS = PAVEWISE_MODEM_POWEROFF_PULSE_MS;

#define BAT_ADC_PIN   PAVEWISE_BAT_ADC_PIN
#define SD_MISO       PAVEWISE_SD_MISO
#define SD_MOSI       PAVEWISE_SD_MOSI
#define SD_SCLK       PAVEWISE_SD_SCLK
#define SD_CS         PAVEWISE_SD_CS

static const char *DIR_LOGS  = PAVEWISE_DIR_LOGS;
static const char *DIR_QUEUE = PAVEWISE_DIR_QUEUE;
static const char *DIR_STATE = PAVEWISE_DIR_STATE;
static const char *FILE_RAIN_PREV_TOTAL = PAVEWISE_FILE_RAIN_PREV_TOTAL;
static const char *FILE_GPS_LAST        = PAVEWISE_FILE_GPS_LAST;
static const char *FILE_GPS_FIX_MS      = PAVEWISE_FILE_GPS_FIX_MS;
static const char *FILE_GPS_RETRY_EPOCH = PAVEWISE_FILE_GPS_RETRY_EPOCH;
static const char *FILE_IDENTITY        = PAVEWISE_FILE_IDENTITY;
static const char *FILE_HTTP_LAST_MS    = PAVEWISE_FILE_HTTP_LAST_MS;

#endif
