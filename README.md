# Pavewise Mobile Rain Gauge (LilyGo T-SIM7600)

This repository contains three primary firmware sketches for a LilyGo T-SIM7600 (SIM7600 + ESP32) rain gauge using the DFRobot rainfall sensor. The sketches share the same core logic but are tuned for different deployment stages (serial debug, release, and no-HTTP logging). The goal is to **collect rainfall and battery data at a fixed cadence**, periodically refresh GPS and time from GNSS, log to SD, and (when enabled) send a compact payload over HTTP.

## Files and roles

- **`PavewiseSerialTestVer2.ino`**
  - Debug build intended for serial visibility.
  - Prints detailed modem, network, GPS, SD, rain, battery, and HTTP timing output.
  - Sends HTTP (when configured).
- **`PavewiseReleaseV2.ino`**
  - Field build with minimal serial output for performance/power.
  - Sends HTTP payloads and logs to SD.
- **`PavewisenoHTTPTestV2.ino`**
  - Same logic as serial tester but **HTTP disabled** and **5‑minute cadence**.
  - Useful for validating SD logging, GPS refresh, and modem bring‑up before enabling HTTP.
- **`TestSerial_fixed_gps.ino`**
  - Known‑working reference for GPS + cellular bring‑up sequence.

## Hardware assumptions

- **Board**: LilyGo T‑SIM7600 (ESP32 + SIM7600)
- **Rain sensor**: DFRobot Rainfall Sensor (I2C)
- **SD card**: SPI mode
- **Battery measurement**: ADC pin 35 (scaled with board divider)

Pin definitions are consistent with the working `TestSerial_fixed_gps.ino` sketch and shared across all three Pavewise sketches.

## User configuration (required)

Before running on hardware, set these values in **all three sketches** (or at least in the one you will use):

1. **APN settings** (for cellular data):
   ```cpp
   static const char APN[]       = "hologram";
   static const char GPRS_USER[] = "";
   static const char GPRS_PASS[] = "";
   ```

2. **HTTP server settings** (used by serial + release builds):
   ```cpp
   static const char SERVER_HOST[] = "example.com";
   static const int  SERVER_PORT   = 80;
   static const char SERVER_PATH[] = "/ingest";
   ```

3. **Wake interval**
   - Serial / Release: 15 minutes (see `WAKE_INTERVAL_SECONDS`)
   - No‑HTTP: 5 minutes (explicitly set)

4. **Optional: GPS / HTTP timing knobs** (already set but can be tuned):
   - GPS refresh every 6 hours (`GPS_REFRESH_SECONDS`)
   - GPS timeout: default 10 minutes, then adaptive (`GPS_TIMEOUT_DEFAULT_MS`)
   - HTTP timeout multiplier: 5x last send (`HTTP_TIMEOUT_MULTIPLIER`)

## System flow (high‑level)

Each wake cycle (after deep sleep) runs the **entire program in `setup()`**:

1. **Power optimizations**
   - CPU downclock to 80 MHz.
   - WiFi/Bluetooth disabled.

2. **SD initialization and housekeeping**
   - Mounts SD and ensures `/logs`, `/queue`, `/state` directories.
   - If SD usage exceeds 80%, deletes oldest daily logs until ~70% usage.

3. **Load persistent state**
   - Cached IMEI/ICCID.
   - Last GPS fix time + coordinates.
   - Last GPS fix duration (for adaptive timeout).
   - Last HTTP send duration (for adaptive timeout).

4. **Read sensors**
   - **Rainfall**: reads cumulative total from DFRobot sensor, logs delta since last wake.
   - **Battery**: reads ADC (pin 35), scales to VBAT.

5. **Modem bring‑up** (SIM7600 power sequence)
   - FLIGHT high, DTR low, PWRKEY pulse.
   - UART start, AT responsiveness check.
   - `modem.init()` and GNSS configuration (`setGNSSMode`).

6. **Network registration + GPRS**
   - Waits for network registration.
   - Prints operator, RSSI, local IP.
   - Attempts GPRS attach for HTTP (if enabled).

7. **GPS refresh logic** (every 6 hours)
   - On first boot or every 6 hours, attempt GNSS fix.
   - Uses **adaptive timeout**: next timeout = last fix time × 2.
   - If fix fails, schedules retry 15 minutes later.

8. **Payload generation**
   - Compact format: `IMEI|BATT_MV|RAIN_X100|EPOCH[|LAT|LON]`
   - Lat/Lon only included on GPS refresh wakes.

9. **SD logging**
   - Daily CSV log under `/logs/log_YYYYMMDD.csv`.
   - Payload also stored as a queued file under `/queue/`.

10. **HTTP send (if enabled)**
    - Serial + Release: sends queued payloads via HTTP.
    - No‑HTTP: explicitly disabled.

11. **Shutdown + deep sleep**
    - Attempts graceful modem power‑down.
    - Schedules deep sleep for next wake.

## Detailed GPS strategy

- **Refresh cadence**: every 6 hours.
- **First boot**: GPS is forced if no valid epoch is stored.
- **Timeout strategy**:
  - Default = 10 minutes.
  - After a successful fix: next timeout = `fix_time × 2`.
  - If fix fails: retry on the next 15‑minute interval.
- **Time anchoring**:
  - When GPS provides valid date/time, it becomes the epoch reference.
  - Between GPS refreshes, the RTC estimate is incremented each wake.

## HTTP queue + retry logic (what happens on failure)

The code uses a **queue folder on SD** to guarantee eventual delivery.

### Queue storage
- Each payload is written to `/queue/q_<epoch>_<wake>.txt`.
- Each file stores exactly one payload line.

### Send flow (serial + release builds)
1. On each wake, if network is available, the code scans `/queue`.
2. Files are sorted by name (oldest first).
3. Each queued file is sent **individually**.
4. On success:
   - The file is deleted.
   - The HTTP duration is recorded to tune the next timeout.
5. On failure:
   - The file is **left in the queue**.
   - The send loop stops and exits early.
   - The last HTTP duration is saved (if available).

### What happens next cycle
- The next wake attempts the queue again, starting with the oldest file.
- If the network is still down or HTTP fails again, the data stays queued.
- If it succeeds later, files are removed one by one in order.

This ensures **no data is lost**, even with intermittent cellular coverage.

## Notes on payload encoding (server decoding)

To reduce data cost, the payload avoids commas and decimals:

- `RAIN_X100` = rain millimeters × 100 (integer)
- `BATT_MV` = battery voltage in millivolts (integer)
- `LAT` / `LON` = degrees × 1e7 (integer)

On the server:
- `rain_mm = RAIN_X100 / 100.0`
- `batt_v  = BATT_MV / 1000.0`
- `lat     = LAT / 1e7`
- `lon     = LON / 1e7`

## Server setup (EC2 Ubuntu)

The device posts payloads to an HTTP endpoint. The easiest path is to run the provided
PostgreSQL-backed ingest service on a **Linux EC2 instance** (tested on Ubuntu). The
setup script installs PostgreSQL + a Python Flask app, creates the database/table,
and writes a connection details file you can use in DB Beaver and `utilities.h`.

### 1) Clone the repo on your EC2 instance

```bash
sudo apt-get update
sudo apt-get install -y git
git clone <this-repo-url>
cd PavewiseRainGuage
```

**If you cannot use git:** download a zip from GitHub (Code → Download ZIP), copy it to the
instance, and unzip it:

```bash
sudo apt-get update
sudo apt-get install -y unzip curl
curl -L -o pavewise.zip <zip-download-url>
unzip pavewise.zip
cd <unzipped-folder>
```

### 2) Run the setup script

```bash
./scripts/setup_ec2_server.sh
```

You will be prompted for:
- Database name
- Database username
- Database password
- Ingest port (default 8080)
- Ingest path (default `/ingest`)

At the end, the script prints and saves a file at:

```
/opt/pavewise-rain-gauge/connection_details.txt
```

This file includes:
- The HTTP ingest URL for the device.
- The Postgres connection details for DB Beaver.
- The exact `PAVEWISE_SERVER_HOST/PORT/PATH` values to paste into `utilities.h`.

### 3) Open firewall rules (EC2 security group)

Allow inbound traffic for:
- **TCP 8080** (or the port you selected) for the HTTP ingest endpoint.
- **TCP 5432** for PostgreSQL (DB Beaver access).

### 4) Configure `utilities.h`

Use the values in `connection_details.txt` to update:

```cpp
#define PAVEWISE_SERVER_HOST "<public-ip>"
#define PAVEWISE_SERVER_PORT 8080
#define PAVEWISE_SERVER_PATH "/ingest"
```

Rebuild and flash the firmware after updating the values.

### 5) Connect from DB Beaver

Create a new **PostgreSQL** connection with:
- **Host**: your EC2 public IP
- **Port**: 5432
- **Database**: the name you provided to the setup script
- **User / Password**: the credentials you provided

Your readings are stored in the `rain_gauge_readings` table. The server automatically
converts:
- `epoch` → `epoch_utc` (UTC timestamp)
- `rain_x100` → `rain_mm`
- `batt_mv` → `batt_v`
- `lat/lon` to decimal degrees (when present)

## Build selection guidance

- Start with **`PavewisenoHTTPTestV2.ino`** to validate SD/GPS/modem without HTTP.
- Move to **`PavewiseSerialTestVer2.ino`** to validate HTTP and queue behavior.
- Deploy **`PavewiseReleaseV2.ino`** for field use (minimal serial overhead).

## Troubleshooting tips

- If the Arduino IDE turns the rest of a file blue (comment‑highlight), ensure no inline lambdas are used in parsing code (see `nmeaDdmmToDeg` helper).
- If GPS fixes are slow, allow a longer timeout or test with clear sky view.
- If network registration fails, verify SIM activation + APN.
