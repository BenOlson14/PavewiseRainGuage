# Pavewise Mobile Rain Gauge (LilyGo T-SIM7600)

This repository contains the **production firmware** for a LilyGo T-SIM7600 rain gauge and a companion **HTTP ingest server** that stores readings in PostgreSQL.

## What this system does

The device wakes on a timer, reads rainfall and battery voltage, maintains time/location with GPS on a slower cadence, writes data to SD card for durability, and uploads queued payloads over cellular HTTP when available.

At a high level, each wake cycle does this:

1. Boot from deep sleep and apply low-power settings.
2. Mount SD and ensure required folders exist.
3. Read rainfall + battery.
4. Power and initialize the SIM7600 modem.
5. Refresh GPS/time on schedule (or retry if prior fix failed).
6. Build a compact payload string.
7. Append a daily CSV log and queue the payload file on SD.
8. Attempt to POST queued payloads oldest-first.
9. Save state and return to deep sleep.

---

## Repository structure (current)

- `PavewiseRelease/PavewiseRelease.ino`  
  Main firmware sketch (production behavior, HTTP enabled).
- `PavewiseRelease/utilities.h`  
  User configuration: APN, server endpoint, serial number parts, wake timing, queue retention, and other tunables.
- `server/app.py`  
  Flask ingest service (`/ingest`) that validates and stores payloads in PostgreSQL.
- `server/schema.sql`  
  Database schema for `rain_gauge_readings`.
- `server/requirements.txt`  
  Python dependencies for the server.
- `server/pavewise.service`  
  Example `systemd` service file.
- `server/setup_ec2_server.sh` / `server/setup_ec2_server_test.sh`  
  Provisioning scripts for server setup.
- `scripts/setup_ec2_server.sh` / `scripts/setup_ec2_server_test.sh`  
  Wrapper/script variants for setup workflows.

---

## Hardware and software prerequisites

### Hardware
- LilyGo T-SIM7600 (ESP32 + SIM7600)
- DFRobot rainfall sensor (I2C)
- SIM card with data plan + APN credentials
- microSD card (formatted and inserted)
- USB cable for firmware upload

### Arduino IDE
- Install ESP32 board support
- In **Tools → Board**, choose **ESP32 Dev Module**
- Open `PavewiseRelease/PavewiseRelease.ino`

> **Important:** Keep only one `.ino` sketch file in `PavewiseRelease/` to avoid duplicate-definition compile errors.

---

## New device setup checklist (before upload)

Use this checklist for every new customer device.

### 1) Set unique serial-number components

Edit `PavewiseRelease/utilities.h`:

```cpp
#define PAVEWISE_SERIAL_SW_VERSION 1
#define PAVEWISE_SERIAL_SIM_PROVIDER 1
#define PAVEWISE_SERIAL_DEVICE_ID 1
```

These are encoded into an 11-digit serial string:
- 2 digits: software version
- 2 digits: SIM provider/manufacturer code
- 7 digits: device ID

Example:
- `SW_VERSION=3`, `SIM_PROVIDER=12`, `DEVICE_ID=45`
- Serial becomes `03120000045`

### 2) Set cellular APN credentials

```cpp
#define PAVEWISE_APN "hologram"
#define PAVEWISE_GPRS_USER ""
#define PAVEWISE_GPRS_PASS ""
```

Use values from your SIM provider.

### 3) Set server endpoint

```cpp
#define PAVEWISE_SERVER_HOST "18.218.149.57"
#define PAVEWISE_SERVER_PORT 8080
#define PAVEWISE_SERVER_PATH "/ingest"
```

Match these exactly to your deployed ingest server.

### 4) Verify wake interval

```cpp
#define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)
```

Set this in `PavewiseRelease/utilities.h` under the user settings section. The firmware now sources wake cadence from `utilities.h`, so changing this value there will take effect. Default is 15 minutes.

### 5) Confirm queue retention behavior

```cpp
#define PAVEWISE_QUEUE_INVALID_RETENTION_DAYS 7UL
```

This controls how long to keep repeatedly-invalid payload queue files before deletion.

### 6) Upload and verify boot logs

After upload, open Serial Monitor and confirm:
- SD mounted
- sensor read succeeded
- modem initialized
- network registration reached
- queue file creation and/or HTTP responses appear

---

## How to change measurement send interval

The wake interval controls how often measurements are captured and queued/sent.

### Default (15 minutes)

```cpp
#define PAVEWISE_WAKE_INTERVAL_SECONDS (15UL * 60UL)
```

### Example: set to 3 minutes for testing

Change to:

```cpp
#define PAVEWISE_WAKE_INTERVAL_SECONDS (3UL * 60UL)
```

Then re-upload firmware.

### Important side effects when reducing interval

- More frequent cellular wake-ups increase power usage.
- Queue growth can accelerate during network outages.
- GPS refresh cadence is independent (`PAVEWISE_GPS_REFRESH_SECONDS`, default 6 hours).

---

## Payload format sent by device

Payload format:

`IMEI|SERIAL|BATT_MV|RAIN_X100|EPOCH[|LAT|LON]`

- `IMEI`: 15-digit modem IMEI (fallback allowed until valid IMEI is captured)
- `SERIAL`: 11-digit serial composed from `utilities.h` constants
- `BATT_MV`: battery millivolts integer
- `RAIN_X100`: rainfall in mm multiplied by 100
- `EPOCH`: Unix epoch seconds
- `LAT`/`LON`: optional; included on GPS refresh wakes, scaled by 1e7

---

## SD card file layout

The firmware uses these paths:

- `/logs/log_YYYYMMDD.csv` – daily readable history
- `/queue/q_<epoch>_<wake>.txt` – queued payloads (one per file)
- `/state/rain_prev_total_mm.txt`
- `/state/gps_last.txt`
- `/state/gps_fix_ms.txt`
- `/state/gps_retry_epoch.txt`
- `/state/http_last_ms.txt`
- `/state/identity.txt`
- `/state/queue_retry_*.txt`

---

## Server setup summary

1. Provision Linux host (EC2 supported by included scripts).
2. Install dependencies from `server/requirements.txt`.
3. Create database objects from `server/schema.sql`.
4. Configure environment variables for DB connection.
5. Run `server/app.py` directly or via `server/pavewise.service`.
6. Open firewall/security group for ingest port (default 8080) and admin SSH.

---

## Customer-facing quick-start (board upload)

For non-developer technicians:

1. Install Arduino IDE + ESP32 support.
2. Connect device via USB.
3. Open `PavewiseRelease/PavewiseRelease.ino`.
4. Edit `PavewiseRelease/utilities.h`:
   - serial number parts
   - APN credentials
   - server host/port/path
   - wake interval (15 min normal, 3 min for short tests)
5. Select correct COM port and board profile.
6. Click Upload.
7. Open Serial Monitor at `115200` baud.
8. Confirm successful sensor/modem/network cycle.
9. Deploy device in field.

---

## Troubleshooting basics

- **No HTTP uploads but data appears on SD**: verify APN and server endpoint values first.
- **Payload rejected by server**: check IMEI/serial formatting and integer field expectations.
- **GPS missing for long periods**: confirm antenna placement and sky visibility.
- **Queue keeps growing**: indicates network or server reachability issue; queue drains automatically once restored.

---

## Notes

- Libraries are vendored under `libraries/` for reproducible builds.
- Server accepts and validates compact integer payload fields, then stores decoded values in PostgreSQL.
