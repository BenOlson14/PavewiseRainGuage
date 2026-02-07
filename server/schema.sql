CREATE TABLE IF NOT EXISTS rain_gauge_readings (
    id BIGSERIAL PRIMARY KEY,
    imei TEXT NOT NULL,
    serial_number TEXT NOT NULL,
    batt_v NUMERIC(10, 3) NOT NULL,
    rain_amount NUMERIC(10, 4) NOT NULL,
    epoch_seconds BIGINT NOT NULL,
    lat_deg NUMERIC(10, 7),
    lon_deg NUMERIC(10, 7),
    received_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_rain_gauge_epoch_seconds
    ON rain_gauge_readings (epoch_seconds DESC);
