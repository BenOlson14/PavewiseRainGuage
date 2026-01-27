# Pavewise Mobile Rain Gauge (LilyGo T-SIM7600)

This repository contains two primary firmware sketches for a LilyGo T-SIM7600 (SIM7600 + ESP32) rain gauge using the DFRobot rainfall sensor. The sketches share the same core logic but are tuned for different deployment stages (release and no-HTTP logging). The goal is to **collect rainfall and battery data at a fixed cadence**, periodically refresh GPS and time from GNSS, log to SD, and (when enabled) send a compact payload over HTTP.

## Files and roles

- **`PavewiseSerialTestVer2.ino`** (release build)
  - Field build that sends HTTP payloads and logs to SD.
  - Serial output is kept minimal to reduce power/time spent awake.
- **`PavewisenoHTTPTestV2.ino`**
  - Same logic as the release build but **HTTP disabled**.
  - Uses the same 15‑minute cadence to keep behavior consistent without uploading data.

## Hardware assumptions

- **Board**: LilyGo T‑SIM7600 (ESP32 + SIM7600)
- **Rain sensor**: DFRobot Rainfall Sensor (I2C)
- **SD card**: SPI mode
- **Battery measurement**: ADC pin 35 (scaled with board divider)

Pin definitions are shared across both Pavewise sketches.

## User configuration (required)

Before running on hardware, set these values in **both sketches** (or at least in the one you will use):

1. **APN settings** (for cellular data):
   ```cpp
   static const char APN[]       = "hologram";
   static const char GPRS_USER[] = "";
   static const char GPRS_PASS[] = "";
   ```

2. **HTTP server settings** (used by the release build):
   ```cpp
   static const char SERVER_HOST[] = "example.com";
   static const int  SERVER_PORT   = 80;
   static const char SERVER_PATH[] = "/ingest";
   ```

3. **Wake interval**
   - Serial / Release / No‑HTTP: 15 minutes (see `WAKE_INTERVAL_SECONDS`)

4. **Optional: GPS / HTTP timing knobs** (already set but can be tuned):
   - GPS refresh every 6 hours (`GPS_REFRESH_SECONDS`)
   - GPS timeout: default 10 minutes, then adaptive (`GPS_TIMEOUT_DEFAULT_MS`)
   - GPS timeout minimum: 3 minutes (`GPS_TIMEOUT_MIN_MS`)
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
   - `modem.init()` and GNSS configuration (`setGPSMode`).

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
    - Release: sends queued payloads via HTTP.
    - No‑HTTP: explicitly disabled.
    - Release: logs the HTTP status and any response body (e.g. `{"status":"stored"}`) so you can confirm the worker accepted the payload.

11. **Shutdown + deep sleep**
    - Attempts graceful modem power‑down.
    - Schedules deep sleep for next wake.

## Detailed GPS strategy

- **Refresh cadence**: every 6 hours.
- **First boot**: GPS is forced if no valid epoch is stored.
- **Timeout strategy**:
  - Default = 10 minutes.
  - Minimum floor = 3 minutes.
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

### Send flow (release build)
1. On each wake, if network is available, the code scans `/queue`.
2. Files are sorted by name (oldest first).
3. Each queued file is sent **individually**.
4. If the server responds with a **payload format error**:
   - The device retries that payload **once immediately**.
   - If it still fails, the payload is retried **once per wake cycle**.
   - After **10 consecutive cycles** of format errors, the queue file is deleted.
5. On success:
   - The file is deleted **only after** a successful HTTP 2xx response.
   - The HTTP duration is recorded to tune the next timeout.
6. On other failures:
   - The file is **left in the queue**.
   - The send loop stops and exits early.
   - The last HTTP duration is saved (if available).

### What happens next cycle
- The next wake attempts the queue again, starting with the oldest file.
- If the network is still down or HTTP fails again, the data stays queued.
- If it succeeds later, files are removed one by one in order.
- The device treats `{"status":"stored"}` as the expected server acknowledgment.

This ensures **no data is lost**, even with intermittent cellular coverage.

## Database schema + expected columns

The server stores payloads in PostgreSQL using the schema in `server/schema.sql`. The primary table is `rain_gauge_readings` with the following expectations:

| Column | Type | Expected format |
| --- | --- | --- |
| `id` | `BIGSERIAL` | Auto-generated primary key. |
| `imei` | `TEXT` | **15-digit numeric IMEI** from the modem. |
| `batt_v` | `NUMERIC(10, 3)` | Battery voltage in volts (`BATT_MV / 1000.0`). |
| `rain_amount` | `NUMERIC(10, 4)` | Rain amount in the device unit (mm or in). |
| `epoch_seconds` | `BIGINT` | Unix epoch seconds from the payload. |
| `epoch_utc` | `TIMESTAMPTZ` | UTC timestamp derived from `epoch_seconds`. |
| `lat_deg` | `NUMERIC(10, 7)` | Latitude in decimal degrees (optional). |
| `lon_deg` | `NUMERIC(10, 7)` | Longitude in decimal degrees (optional). |
| `has_gps` | `BOOLEAN` | True when GPS lat/lon were included. |
| `received_at` | `TIMESTAMPTZ` | Server ingest timestamp. |

The `device_rain_units` table maps each IMEI to a preferred unit (`mm` or `in`) and tracks when the device was first/last seen.

> **Note on time zones:** `TIMESTAMPTZ` values are stored in UTC, but clients display them in the session time zone. If you want `epoch_utc` (and `received_at`) to always render as UTC in tools like DBeaver, set the session time zone to UTC (e.g., `SET TIME ZONE 'UTC';`) or configure the connection’s time zone setting to `UTC`. You can also query with `epoch_utc AT TIME ZONE 'UTC'` to force UTC in a specific SELECT.

### Payload expectations (server validation)
- `IMEI` must be **15 digits**.
- `BATT_MV`, `RAIN_X100`, `EPOCH`, `LAT`, and `LON` must be **integers** (LAT/LON allow a leading `-`).
- Units must be `mm` or `in` when provided.
- GPS coordinates must fall within valid ranges (lat ±90, lon ±180).

Invalid payloads are rejected with an HTTP 400 response and a JSON body like:
`{"error":"...","code":"invalid_payload","field":"imei"}`.

## Why "SMS DONE" can show up in IMEI

SIM7600 modems can emit unsolicited result codes (URCs) such as **`SMS DONE`** while the firmware is reading AT command responses. If a URC lands in the middle of an IMEI read, the raw response can be polluted and cached as the IMEI. The firmware now **drains the modem serial buffer and retries IMEI reads** (with a short delay) and **validates IMEI format** so `SMS DONE` cannot enter the payload or database.

## Notes on payload encoding (server decoding)

To reduce data cost, the payload avoids commas and decimals:

- `RAIN_X100` = rain millimeters × 100 (integer, always mm on the wire)
- `BATT_MV` = battery voltage in millivolts (integer)
- `LAT` / `LON` = degrees × 1e7 (integer)

On the server:
- `rain_amount` is stored in the device's preferred unit (`mm` or `in`).
  - `rain_amount = RAIN_X100 / 100.0` when the unit is `mm`.
  - `rain_amount = (RAIN_X100 / 100.0) / 25.4` when the unit is `in`.
- `batt_v = BATT_MV / 1000.0`
- `lat     = LAT / 1e7`
- `lon     = LON / 1e7`

## Server process manager (systemd auto-restart)

For a production setup, use a systemd service so the worker restarts automatically if it
dies. Example unit file is included at `server/pavewise.service` and sets `Restart=always`
with a short delay. Adjust the paths, user, and environment file as needed for your host.

### Where logs go
- **systemd**: logs are written to journald (stdout/stderr). View with:
  `journalctl -u pavewise --since "1 hour ago" -f`
- **Direct run** (`python server/app.py`): logs go to your terminal stdout/stderr.

## Server setup (EC2 Linux)

The device posts payloads to an HTTP endpoint. The easiest path is to run the provided
PostgreSQL-backed ingest service on a **Linux EC2 instance** (tested on Ubuntu). The
setup script installs PostgreSQL + a Python Flask app, creates the database/table,
and writes a connection details file you can use in DB Beaver and `utilities.h`.

### Security groups (AWS inbound rules)
You must explicitly allow the inbound ports that match your firmware and server config:

- **HTTP ingest port**: allow the port configured in `PAVEWISE_PORT` (default `8080`).
  - Source can be `0.0.0.0/0` for public access, **or** a restricted CIDR if devices
    have static IPs or a fixed NAT gateway.
- **SSH (22)**: allow from your admin IP only.
- **PostgreSQL (5432)**: **do not open to the public internet** for production.
  - Restrict to your VPC CIDR or a bastion host.
  - If you only need local access, keep it closed and use SSH tunneling.

The setup script currently configures PostgreSQL to listen on all interfaces
(`listen_addresses='*'`) and allows `0.0.0.0/0` in `pg_hba.conf`. This is convenient for
initial testing, but you should lock it down for production by restricting the CIDR or
reverting to `localhost` only.

### Hardening for production (recommended)
- **Restrict database access**: limit `pg_hba.conf` to your VPC CIDR or `127.0.0.1/32`
  and change `listen_addresses` to `localhost` if you only need local access.
- **Reduce DB privileges**: the setup script grants broad privileges to the app user
  for convenience. For production, create a dedicated role with only `INSERT` on the
  `rain_gauge_readings` table (no `CREATEDB`), and rotate the password regularly.
- **Ingress control**: if devices are behind a fixed NAT, restrict the HTTP security
  group rule to that IP range instead of `0.0.0.0/0`.

### 1) Fetch only the server folder on your EC2 instance (recommended)

```bash
sudo apt-get update
sudo apt-get install -y git python3 python3-pip
REPO_URL="https://github.com/BenOlson14/PavewiseRainGuage.git"
git clone --depth 1 --filter=blob:none --sparse "${REPO_URL}" pavewise-rain-gauge
cd pavewise-rain-gauge
git sparse-checkout set server
cd server
```

If Git prompts for a username, the URL is likely private or requires authentication.
For public access, make sure you are using the exact URL above. For private repos,
use a GitHub Personal Access Token (PAT) as the password when prompted.

If you already cloned the repo, you can also run the original scripts from the repo root:

```bash
./scripts/setup_ec2_server.sh
```

### 2) Run the setup script (main server)

```bash
sudo bash setup_ec2_server.sh
```

> Note: Run as a normal user with sudo, or as root. The script uses sudo internally.

You will be prompted for:
- Database name
- Database username
- Database password
- Ingest port (default 8080)
- Ingest path (default `/ingest`)
- Gunicorn worker count (default `CPU * 2 + 1`)
- Gunicorn threads per worker (default `4`)

### 2a) Run the testing setup script (fixed defaults)

If you want a dedicated test database/user, run:

```bash
sudo bash setup_ec2_server_test.sh
```

**One-shot copy/paste blocks**

Main server:

```bash
sudo apt-get update
sudo apt-get install -y git python3 python3-pip
REPO_URL="https://github.com/BenOlson14/PavewiseRainGuage.git"
git clone --depth 1 --filter=blob:none --sparse "${REPO_URL}" pavewise-rain-gauge
cd pavewise-rain-gauge
git sparse-checkout set server
cd server
sudo bash setup_ec2_server.sh
```

Testing server:

```bash
sudo apt-get update
sudo apt-get install -y git python3 python3-pip
REPO_URL="https://github.com/BenOlson14/PavewiseRainGuage.git"
git clone --depth 1 --filter=blob:none --sparse "${REPO_URL}" pavewise-rain-gauge
cd pavewise-rain-gauge
git sparse-checkout set server
cd server
sudo bash setup_ec2_server_test.sh
```

This script uses the defaults for port/path/workers and creates:
- **Database**: `PavewiseTest`
- **User**: `PavewiseTester`
- **Password**: `Pavewise`

**Worker sizing guidance**
- Each Gunicorn **worker** runs a separate Python process. More workers allow more concurrent device requests.
- Each worker can also run multiple **threads**. Threads are useful for I/O-bound work like HTTP + database calls.
- Defaults are chosen to be safe on small EC2 instances; you can raise workers/threads if CPU and memory allow it.
- If you change these later, rerun the setup script to regenerate the systemd service.

### How the ingest worker stays alive
- The `pavewise-ingest` **systemd service** runs Gunicorn and restarts on failure
  (`Restart=on-failure` with a short delay).
- The Flask app also includes a lightweight **heartbeat thread** that logs a periodic
  "alive" message (`PAVEWISE_HEARTBEAT_SECONDS`, default 900s). This helps confirm
  the worker is still running even when no device posts are arriving.

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

`utilities.h` includes a **connection placeholder** comment block that shows example
`PAVEWISE_SERVER_HOST/PORT/PATH` values.

Additional device options you can adjust:
- `PAVEWISE_DEBUG_BAUD` to set the Serial logging baud rate.
- `PAVEWISE_RAIN_UNIT` to choose `"mm"` or `"in"` for reported rainfall (sent once on
  the first successful upload for each IMEI).
  - If the server does not recognize the IMEI and the unit was not included, it
    responds with `unit_required` and the device will retry with the unit until stored.

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
- `rain_x100` → `rain_amount` (stored in the unit configured for that IMEI)
- `batt_mv` → `batt_v`
- `lat/lon` to decimal degrees (when present)

Device reporting units are stored in the `device_rain_units` table (keyed by IMEI).

## Build selection guidance

- Start with **`PavewisenoHTTPTestV2.ino`** to validate SD/GPS/modem without HTTP.
- Move to **`PavewiseSerialTestVer2.ino`** to validate HTTP and queue behavior.
- Deploy **`PavewiseSerialTestVer2.ino`** for field use (minimal serial overhead).
- Serial logging is always enabled; expect longer wake times and higher UART/power
  usage due to Serial I/O.

## Troubleshooting tips

- If the Arduino IDE turns the rest of a file blue (comment‑highlight), ensure no inline lambdas are used in parsing code (see `nmeaDdmmToDeg` helper).
- If GPS fixes are slow, allow a longer timeout or test with clear sky view.
- If network registration fails, verify SIM activation + APN.
