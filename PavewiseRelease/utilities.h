#ifndef PAVEWISE_UTILITIES_H
#define PAVEWISE_UTILITIES_H

#include <Arduino.h>

// ============================= USER SETTINGS =============================
// All user-configurable settings live in this file so you can adjust APN,
// server details, timing, and pin mappings without editing the main sketches.

// Serial baud rate for debug logging.
// Example: #define PAVEWISE_DEBUG_BAUD 115200
#ifndef PAVEWISE_DEBUG_BAUD
#define PAVEWISE_DEBUG_BAUD 115200
#endif

// Device serial number format (11 digits total):
//   - digits 1-2: software version (00-99)
//   - digits 3-4: SIM provider/manufacturer code (00-99)
//   - digits 5-11: device ID (0000000-9999999)
// Example: software version 1, provider 2, device ID 42 => "01020000042"
#ifndef PAVEWISE_SERIAL_SW_VERSION
#define PAVEWISE_SERIAL_SW_VERSION 1
#endif

#ifndef PAVEWISE_SERIAL_SIM_PROVIDER
#define PAVEWISE_SERIAL_SIM_PROVIDER 1
#endif

#ifndef PAVEWISE_SERIAL_DEVICE_ID
#define PAVEWISE_SERIAL_DEVICE_ID 1
#endif

// Wake interval (seconds) between deep-sleep cycles.
// Example: #define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)  // 15 minutes
#ifndef PAVEWISE_WAKE_INTERVAL_SECONDS
#define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)
#endif

// GPS refresh interval (seconds) to re-anchor time/location.
// Example: #define PAVEWISE_GPS_REFRESH_SECONDS (6UL * 3600UL)  // 6 hours
#ifndef PAVEWISE_GPS_REFRESH_SECONDS
#define PAVEWISE_GPS_REFRESH_SECONDS (6UL * 3600UL)
#endif

// GPS fix timeout strategy:
//   - default is 10 minutes
//   - after a successful fix, next timeout = last_fix_time * 2
// Example: #define PAVEWISE_GPS_TIMEOUT_DEFAULT_MS (10UL * 60UL * 1000UL)
#ifndef PAVEWISE_GPS_TIMEOUT_DEFAULT_MS
#define PAVEWISE_GPS_TIMEOUT_DEFAULT_MS (10UL * 60UL * 1000UL)
#endif
#ifndef PAVEWISE_GPS_TIMEOUT_MIN_MS
// Ensure GPS attempts always run for at least 3 minutes.
// Example: #define PAVEWISE_GPS_TIMEOUT_MIN_MS (3UL * 60UL * 1000UL)
#define PAVEWISE_GPS_TIMEOUT_MIN_MS (3UL * 60UL * 1000UL)
#endif

// If a fix fails, retry next wake (seconds).
// Example: #define PAVEWISE_GPS_RETRY_SECONDS (15UL * 60UL)
#ifndef PAVEWISE_GPS_RETRY_SECONDS
#define PAVEWISE_GPS_RETRY_SECONDS (15UL * 60UL)
#endif

// HTTP send strategy:
//   - timeout adapts based on last send duration (x5, capped).
//   - queue files are retried in order; failures stop the loop.
// Example: #define PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS (30UL * 1000UL)
#ifndef PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS
#define PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS (30UL * 1000UL)
#endif

// Example: #define PAVEWISE_HTTP_TIMEOUT_MAX_MS (60UL * 1000UL)
#ifndef PAVEWISE_HTTP_TIMEOUT_MAX_MS
#define PAVEWISE_HTTP_TIMEOUT_MAX_MS (60UL * 1000UL)
#endif

// Example: #define PAVEWISE_HTTP_TIMEOUT_MULTIPLIER (5UL)
#ifndef PAVEWISE_HTTP_TIMEOUT_MULTIPLIER
#define PAVEWISE_HTTP_TIMEOUT_MULTIPLIER (5UL)
#endif

// Enable/disable HTTP uploads.
// Example: #define PAVEWISE_ENABLE_HTTP true
#ifndef PAVEWISE_ENABLE_HTTP
#define PAVEWISE_ENABLE_HTTP true
#endif

// Invalid queue payload retention window (days).
// If server keeps returning "invalid payload", the file is deleted after this many days
// worth of wake cycles.
// Example: #define PAVEWISE_QUEUE_INVALID_RETENTION_DAYS 7UL
#ifndef PAVEWISE_QUEUE_INVALID_RETENTION_DAYS
#define PAVEWISE_QUEUE_INVALID_RETENTION_DAYS 7UL
#endif

// Queue preload test (Serial build only):
// When enabled, the sketch will add synthetic payloads to /queue once per SD card
// (guarded by a state file) so you can validate queue send logic.
// - PAVEWISE_QUEUE_PRELOAD_TEST enables a fixed payload count.
// - PAVEWISE_QUEUE_PRELOAD_WEEK enables a full week of payloads based on the wake interval.
//   This is helpful for validating week-long backlog uploads in a single test run.
#ifndef PAVEWISE_QUEUE_PRELOAD_TEST
#define PAVEWISE_QUEUE_PRELOAD_TEST false
#endif
#ifndef PAVEWISE_QUEUE_PRELOAD_COUNT
#define PAVEWISE_QUEUE_PRELOAD_COUNT 10
#endif
#ifndef PAVEWISE_QUEUE_PRELOAD_WEEK
#define PAVEWISE_QUEUE_PRELOAD_WEEK false
#endif

// Cellular APN settings.
// Example: #define PAVEWISE_APN "hologram"
#ifndef PAVEWISE_APN
#define PAVEWISE_APN "hologram"
#endif

// Example: #define PAVEWISE_GPRS_USER ""
#ifndef PAVEWISE_GPRS_USER
#define PAVEWISE_GPRS_USER ""
#endif

// Example: #define PAVEWISE_GPRS_PASS ""
#ifndef PAVEWISE_GPRS_PASS
#define PAVEWISE_GPRS_PASS ""
#endif

// Web server settings (ingest endpoint).
// Example:
//   #define PAVEWISE_SERVER_HOST "18.218.149.57"
//   #define PAVEWISE_SERVER_PORT 8080
//   #define PAVEWISE_SERVER_PATH "/ingest"
#ifndef PAVEWISE_SERVER_HOST
#define PAVEWISE_SERVER_HOST "18.218.149.57"
#endif

#ifndef PAVEWISE_SERVER_PORT
#define PAVEWISE_SERVER_PORT 8080
#endif

#ifndef PAVEWISE_SERVER_PATH
#define PAVEWISE_SERVER_PATH "/ingest"
#endif

// SD purge thresholds (percentage of card used).
// Example: #define PAVEWISE_SD_PURGE_START_PCT 80.0f
#ifndef PAVEWISE_SD_PURGE_START_PCT
#define PAVEWISE_SD_PURGE_START_PCT 80.0f
#endif

// Example: #define PAVEWISE_SD_PURGE_TARGET_PCT 70.0f
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

#ifndef PAVEWISE_FILE_IMEI_LAST_CHECK_EPOCH
#define PAVEWISE_FILE_IMEI_LAST_CHECK_EPOCH "/state/imei_last_check_epoch.txt"
#endif

#ifndef PAVEWISE_FILE_HTTP_LAST_MS
#define PAVEWISE_FILE_HTTP_LAST_MS "/state/http_last_ms.txt"
#endif

#ifndef PAVEWISE_FILE_BOOT_COUNTER
#define PAVEWISE_FILE_BOOT_COUNTER "/state/boot_counter.txt"
#endif

// ============================= CONFIG CONSTANTS =============================

static const uint32_t DEBUG_BAUD = PAVEWISE_DEBUG_BAUD;
static const uint8_t SERIAL_SW_VERSION = PAVEWISE_SERIAL_SW_VERSION;
static const uint8_t SERIAL_SIM_PROVIDER = PAVEWISE_SERIAL_SIM_PROVIDER;
static const uint32_t SERIAL_DEVICE_ID = PAVEWISE_SERIAL_DEVICE_ID;

static const uint32_t WAKE_INTERVAL_SECONDS = PAVEWISE_WAKE_INTERVAL_SECONDS;
static const uint32_t GPS_REFRESH_SECONDS   = PAVEWISE_GPS_REFRESH_SECONDS;
static const uint32_t GPS_TIMEOUT_DEFAULT_MS = PAVEWISE_GPS_TIMEOUT_DEFAULT_MS;
static const uint32_t GPS_TIMEOUT_MIN_MS     = PAVEWISE_GPS_TIMEOUT_MIN_MS;
static const uint32_t GPS_RETRY_SECONDS      = PAVEWISE_GPS_RETRY_SECONDS;

static const uint32_t HTTP_TIMEOUT_DEFAULT_MS = PAVEWISE_HTTP_TIMEOUT_DEFAULT_MS;
static const uint32_t HTTP_TIMEOUT_MAX_MS     = PAVEWISE_HTTP_TIMEOUT_MAX_MS;
static const uint32_t HTTP_TIMEOUT_MULTIPLIER = PAVEWISE_HTTP_TIMEOUT_MULTIPLIER;
static const bool ENABLE_HTTP = PAVEWISE_ENABLE_HTTP;
static const uint32_t QUEUE_INVALID_RETENTION_DAYS = PAVEWISE_QUEUE_INVALID_RETENTION_DAYS;
static const bool QUEUE_PRELOAD_TEST_ENABLED = PAVEWISE_QUEUE_PRELOAD_TEST;
static const bool QUEUE_PRELOAD_WEEK_ENABLED = PAVEWISE_QUEUE_PRELOAD_WEEK;
static const uint32_t QUEUE_PRELOAD_COUNT = PAVEWISE_QUEUE_PRELOAD_COUNT;
static const uint32_t QUEUE_PRELOAD_WEEK_COUNT =
  ((7UL * 24UL * 3600UL) / WAKE_INTERVAL_SECONDS) > 0
    ? ((7UL * 24UL * 3600UL) / WAKE_INTERVAL_SECONDS)
    : 1UL;

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
static const char *FILE_IMEI_LAST_CHECK_EPOCH = PAVEWISE_FILE_IMEI_LAST_CHECK_EPOCH;
static const char *FILE_HTTP_LAST_MS    = PAVEWISE_FILE_HTTP_LAST_MS;

#endif
