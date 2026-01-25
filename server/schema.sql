CREATE TABLE IF NOT EXISTS rain_gauge_readings (
    id BIGSERIAL PRIMARY KEY,
    imei TEXT NOT NULL,
    batt_mv INTEGER NOT NULL,
    batt_v NUMERIC(10, 3) NOT NULL,
    rain_x100 INTEGER NOT NULL,
    rain_mm NUMERIC(10, 2) NOT NULL,
    epoch_seconds BIGINT NOT NULL,
    epoch_utc TIMESTAMPTZ NOT NULL,
    lat_deg NUMERIC(10, 7),
    lon_deg NUMERIC(10, 7),
    has_gps BOOLEAN NOT NULL DEFAULT FALSE,
    received_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE TABLE IF NOT EXISTS device_rain_units (
    imei TEXT PRIMARY KEY,
    rain_unit TEXT NOT NULL CHECK (rain_unit IN ('mm', 'in')),
    first_seen TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_seen TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_rain_gauge_epoch_utc
    ON rain_gauge_readings (epoch_utc DESC);
