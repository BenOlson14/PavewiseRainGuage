# Pavewise System Architecture

This document captures the current system architecture for the Pavewise rain gauge platform based on the release firmware, the Flask ingest service, the PostgreSQL schema, and the EC2 deployment scripts.

## 1. End-to-end deployment / networking diagram

```mermaid
flowchart LR
    subgraph Field[Field Deployment]
        RS["DFRobot Rainfall Sensor<br/>I2C"]
        BAT["Battery ADC<br/>GPIO 35"]
        SD["MicroSD Card<br/>logs, queue, state"]
        GNSS["SIM7600 GNSS"]
        MCU["LilyGo T-SIM7600<br/>ESP32 firmware"]
        LTE["SIM7600 LTE modem<br/>APN and GPRS"]

        RS --> MCU
        BAT --> MCU
        SD <--> MCU
        GNSS --> MCU
        MCU <--> LTE
    end

    LTE -- "HTTP POST /ingest" --> CELL((Cellular Network))
    CELL --> INTERNET["Public Internet"]

    subgraph AWS[AWS EC2 Host]
        SG["Security Group<br/>Allow TCP 8080<br/>Allow SSH 22<br/>Restrict 5432"]
        GUNI["Gunicorn and systemd<br/>pavewise-ingest.service"]
        API["Flask ingest API<br/>server/app.py"]
        PG[("PostgreSQL<br/>rain_gauge_readings")]
        JOURNAL["systemd journal<br/>heartbeat and request logs"]
        DETAILS["connection_details.txt<br/>server host, port, path"]

        SG --> GUNI
        GUNI --> API
        API --> PG
        GUNI --> JOURNAL
        DETAILS -. "setup output" .-> GUNI
    end

    INTERNET --> SG
```

### Networking notes

- The device uses the SIM7600 modem for LTE registration and GPRS data service before attempting HTTP uploads.
- The firmware posts compact payloads to the configured `PAVEWISE_SERVER_HOST`, `PAVEWISE_SERVER_PORT`, and `PAVEWISE_SERVER_PATH` values.
- The EC2 setup script provisions Gunicorn behind a systemd service and binds the Flask app to `0.0.0.0:<port>`.
- PostgreSQL is installed on the same EC2 instance by the setup script and is accessed locally by the Flask app via environment variables.

## 2. Firmware wake-cycle flow diagram

```mermaid
flowchart TD
    A["Wake from deep sleep"] --> B["Reduce power usage<br/>CPU 80 MHz<br/>WiFi and BT off"]
    B --> C["Mount SD card<br/>Ensure logs, queue, state"]
    C --> D["Load persisted state<br/>identity, GPS, HTTP timing"]
    D --> E["Read rainfall sensor<br/>and battery ADC"]
    E --> F["Power up SIM7600<br/>UART and AT checks"]
    F --> G{"Network registered?"}
    G -- No --> Q["Keep queue on SD<br/>Log failure state"]
    G -- Yes --> H["Attach GPRS for HTTP"]
    H --> I{"GPS refresh due?<br/>first boot, 6h, retry"}
    I -- Yes --> J["Attempt GNSS fix<br/>adaptive timeout"]
    I -- No --> K["Use cached location<br/>and RTC epoch estimate"]
    J --> K
    K --> L["Build compact payload<br/>IMEI, SERIAL, BATT_MV, RAIN_X100, EPOCH, LAT, LON"]
    L --> M["Append daily CSV log"]
    M --> N["Write queue file"]
    N --> O{"HTTP enabled and online?"}
    O -- No --> Q
    O -- Yes --> P["Send queued payloads<br/>oldest first"]
    P --> R{"HTTP 2xx?"}
    R -- Yes --> S["Delete sent queue file<br/>record HTTP duration"]
    R -- No --> T["Leave file on SD<br/>retry next wake"]
    S --> U["Graceful modem shutdown"]
    T --> U
    Q --> U
    U --> V["Deep sleep until next interval"]
```

## 3. Server ingest and data flow diagram

```mermaid
sequenceDiagram
    participant Device as Rain Gauge Device
    participant Queue as SD Queue
    participant API as Flask /ingest
    participant DB as PostgreSQL
    participant Logs as App Logs

    Device->>Queue: Write payload file q_epoch_wake.txt
    Device->>API: HTTP POST compact payload
    API->>API: Validate IMEI, serial, integers, and GPS ranges
    alt Invalid payload
        API->>Logs: warning with invalid payload details
        API-->>Device: HTTP 400 JSON error
        Device->>Queue: Keep payload and retry per policy
    else Valid payload
        API->>DB: Insert into rain_gauge_readings
        DB-->>API: Commit success
        API->>Logs: info db_insert_success
        API-->>Device: HTTP 200 status stored
        Device->>Queue: Delete file after success
    end
```

## 4. Data ownership by subsystem

| Subsystem | Responsibility | Primary files |
| --- | --- | --- |
| Device firmware | Sampling rainfall, battery, GNSS, queueing payloads, HTTP retries, deep sleep | `PavewiseRelease/PavewiseRelease.ino`, `PavewiseRelease/utilities.h` |
| Ingest API | Parse compact payloads, validate fields, store readings, expose health endpoint | `server/app.py` |
| Database | Persist normalized readings and ingest timestamp | `server/schema.sql` |
| Provisioning | Install PostgreSQL, Gunicorn, systemd service, and environment settings on EC2 | `server/setup_ec2_server.sh` |

## 5. Trust boundaries and interfaces

1. **Sensor bus boundary**: The ESP32 trusts local I2C and ADC readings from the rainfall sensor and battery divider.
2. **Cellular/network boundary**: Payload delivery depends on LTE registration, APN access, and public internet routing.
3. **API boundary**: The Flask service validates all incoming fields before inserting into PostgreSQL.
4. **Persistence boundary**: The SD card protects field data during connectivity outages; PostgreSQL becomes the system of record after successful ingest.

## 6. Failure-handling summary

- **No cellular / HTTP outage**: payload remains on SD queue and is retried on later wake cycles.
- **GPS failure**: device falls back to cached epoch/location and retries GPS later.
- **Invalid payload**: server returns HTTP 400, firmware retains the queue item and eventually drops it after the configured invalid-payload retention window.
- **Database insert failure**: server returns HTTP 500, and the device keeps the queue file for later retry.
