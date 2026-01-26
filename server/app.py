import logging
import os
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone

import psycopg2
from flask import Flask, jsonify, request


def _env(name: str, default: str | None = None) -> str:
    """Read an env var or raise if required and missing."""
    value = os.getenv(name, default)
    if value is None:
        raise RuntimeError(f"Missing required env var: {name}")
    return value


def get_db_conn():
    """Open a short-lived PostgreSQL connection for each request."""
    return psycopg2.connect(
        host=_env("PAVEWISE_DB_HOST", "localhost"),
        port=int(_env("PAVEWISE_DB_PORT", "5432")),
        dbname=_env("PAVEWISE_DB_NAME"),
        user=_env("PAVEWISE_DB_USER"),
        password=_env("PAVEWISE_DB_PASSWORD"),
        connect_timeout=5,
    )


def _normalize_unit(raw_unit: str | None) -> str | None:
    """Normalize unit tokens sent by devices."""
    if raw_unit is None:
        return None
    unit = raw_unit.lower()
    if unit in ("inch", "inches"):
        unit = "in"
    if unit not in ("mm", "in"):
        return None
    return unit


@dataclass(frozen=True)
class PayloadError(Exception):
    message: str
    field: str


_IMEI_RE = re.compile(r"^\d{15}$")


def _require_int(value: str, field: str, allow_negative: bool = False) -> int:
    """Parse an integer with strict format checks."""
    token = value.strip()
    if not token:
        raise PayloadError("Missing value", field)
    if allow_negative:
        if token[0] == "-":
            if not token[1:].isdigit():
                raise PayloadError("Expected signed integer", field)
        elif not token.isdigit():
            raise PayloadError("Expected signed integer", field)
    elif not token.isdigit():
        raise PayloadError("Expected integer", field)
    return int(token)


def _validate_imei(imei: str) -> None:
    if not _IMEI_RE.match(imei):
        raise PayloadError("Expected 15-digit IMEI", "imei")


def parse_payload(raw_payload: str) -> dict:
    """Decode the compact payload into typed values with strict format checks."""
    # Trim and split on the compact delimiter. Empty segments are discarded.
    parts = [part.strip() for part in raw_payload.strip().split("|") if part.strip()]
    if len(parts) not in (4, 5, 6, 7):
        raise ValueError(
            "Invalid payload format. Expected IMEI|BATT_MV|RAIN_X100|EPOCH[|UNIT][|LAT|LON]."
        )

    imei = parts[0]
    _validate_imei(imei)
    batt_mv = _require_int(parts[1], "batt_mv")
    rain_x100 = _require_int(parts[2], "rain_x100")
    epoch_seconds = _require_int(parts[3], "epoch_seconds")

    unit = None
    lat_deg = None
    lon_deg = None
    has_gps = False

    # Optional unit token (mm/in) can be sent once, or whenever it changes.
    if len(parts) in (5, 7):
        unit = _normalize_unit(parts[4])
        if unit is None:
            raise PayloadError("Invalid unit token", "rain_unit")

    # GPS coordinates are only present when the device refreshes GNSS.
    if len(parts) == 6:
        lat_deg = _require_int(parts[4], "lat_deg", allow_negative=True) / 1e7
        lon_deg = _require_int(parts[5], "lon_deg", allow_negative=True) / 1e7
        has_gps = True
    elif len(parts) == 7:
        lat_deg = _require_int(parts[5], "lat_deg", allow_negative=True) / 1e7
        lon_deg = _require_int(parts[6], "lon_deg", allow_negative=True) / 1e7
        has_gps = True

    if batt_mv < 0 or batt_mv > 8000:
        raise PayloadError("Battery millivolts out of range", "batt_mv")
    if rain_x100 < 0:
        raise PayloadError("Rain amount must be non-negative", "rain_x100")
    if epoch_seconds < 0:
        raise PayloadError("Epoch seconds must be non-negative", "epoch_seconds")
    if has_gps:
        if lat_deg is None or lon_deg is None:
            raise PayloadError("Missing GPS coordinates", "gps")
        if not (-90.0 <= lat_deg <= 90.0):
            raise PayloadError("Latitude out of range", "lat_deg")
        if not (-180.0 <= lon_deg <= 180.0):
            raise PayloadError("Longitude out of range", "lon_deg")

    return {
        "imei": imei,
        "batt_mv": batt_mv,
        "batt_v": round(batt_mv / 1000.0, 3),
        "rain_x100": rain_x100,
        "rain_unit": unit,
        "epoch_seconds": epoch_seconds,
        "epoch_utc": datetime.fromtimestamp(epoch_seconds, tz=timezone.utc),
        "lat_deg": lat_deg,
        "lon_deg": lon_deg,
        "has_gps": has_gps,
    }


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)

app = Flask(__name__)
PAVEWISE_PATH = os.getenv("PAVEWISE_PATH", "/ingest")
PAVEWISE_HEARTBEAT_SECONDS = int(os.getenv("PAVEWISE_HEARTBEAT_SECONDS", "900"))


def _start_worker_heartbeat(interval_seconds: int) -> None:
    """Emit periodic log messages so we can detect a stalled worker."""
    if interval_seconds <= 0:
        app.logger.info("worker_heartbeat disabled interval_s=%s", interval_seconds)
        return

    def _run() -> None:
        app.logger.info("worker_heartbeat started interval_s=%s", interval_seconds)
        while True:
            time.sleep(interval_seconds)
            app.logger.info("worker_heartbeat alive interval_s=%s", interval_seconds)

    thread = threading.Thread(target=_run, daemon=True, name="worker-heartbeat")
    thread.start()


_start_worker_heartbeat(PAVEWISE_HEARTBEAT_SECONDS)


@app.get("/health")
def healthcheck():
    """Simple health probe."""
    return jsonify({"status": "ok"})


@app.route(PAVEWISE_PATH, methods=["POST"])
def ingest():
    """Main ingest endpoint for rain gauge payloads or unit registration."""
    # Payload is a compact, pipe-delimited text record from the device.
    payload = request.get_data(as_text=True) or ""
    payload = payload.strip()
    if not payload:
        app.logger.warning("ingest_empty_payload")
        return jsonify({"error": "Empty payload"}), 400

    # A short unit-only payload (IMEI|UNIT) is used to register the device unit.
    unit_parts = [part.strip() for part in payload.split("|") if part.strip()]
    if len(unit_parts) == 2:
        unit = _normalize_unit(unit_parts[1])
        if unit is None:
            return jsonify({"error": "Invalid unit payload", "code": "invalid_payload"}), 400
        try:
            _validate_imei(unit_parts[0])
        except PayloadError as exc:
            return (
                jsonify({"error": exc.message, "code": "invalid_payload", "field": exc.field}),
                400,
            )
        try:
            with get_db_conn() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        INSERT INTO device_rain_units (imei, rain_unit, first_seen, last_seen)
                        VALUES (%s, %s, NOW(), NOW())
                        ON CONFLICT (imei) DO UPDATE
                            SET rain_unit = EXCLUDED.rain_unit,
                                last_seen = NOW()
                        """,
                        (unit_parts[0], unit),
                    )
        except Exception as exc:
            app.logger.exception("db_unit_insert_error payload=%s error=%s", payload, exc)
            return jsonify({"error": "Database insert failed"}), 500
        return jsonify({"status": "stored"})

    try:
        parsed = parse_payload(payload)
    except PayloadError as exc:
        app.logger.warning("ingest_parse_error payload=%s error=%s field=%s", payload, exc, exc.field)
        return (
            jsonify({"error": exc.message, "code": "invalid_payload", "field": exc.field}),
            400,
        )
    except ValueError as exc:
        app.logger.warning("ingest_parse_error payload=%s error=%s", payload, exc)
        return jsonify({"error": str(exc), "code": "invalid_payload"}), 400

    app.logger.info("ingest_request payload=%s", payload)
    app.logger.info("ingest_parsed payload=%s parsed=%s", payload, parsed)

    insert_sql = """
        INSERT INTO rain_gauge_readings (
            imei,
            batt_v,
            rain_amount,
            epoch_seconds,
            epoch_utc,
            lat_deg,
            lon_deg,
            has_gps
        )
        VALUES (%(imei)s, %(batt_v)s, %(rain_amount)s,
                %(epoch_seconds)s, %(epoch_utc)s, %(lat_deg)s, %(lon_deg)s, %(has_gps)s)
    """

    try:
        app.logger.info("db_insert_attempt payload=%s", payload)
        with get_db_conn() as conn:
            with conn.cursor() as cur:
                # If the payload did not include a unit, look up the stored one.
                if parsed["rain_unit"] is None:
                    cur.execute(
                        "SELECT rain_unit FROM device_rain_units WHERE imei = %s",
                        (parsed["imei"],),
                    )
                    row = cur.fetchone()
                    if row is None:
                        return (
                            jsonify(
                                {
                                    "error": "Unit required for unknown IMEI.",
                                    "code": "unit_required",
                                }
                            ),
                            400,
                        )
                    parsed["rain_unit"] = row[0]
                else:
                    # Upsert the unit if it was explicitly provided.
                    cur.execute(
                        """
                        INSERT INTO device_rain_units (imei, rain_unit, first_seen, last_seen)
                        VALUES (%(imei)s, %(rain_unit)s, NOW(), NOW())
                        ON CONFLICT (imei) DO UPDATE
                            SET rain_unit = EXCLUDED.rain_unit,
                                last_seen = NOW()
                        """,
                        parsed,
                    )

                # Payload rain values are scaled millimeters (RAIN_X100).
                # Store the rain amount in the device's preferred unit.
                rain_mm = parsed["rain_x100"] / 100.0
                if parsed["rain_unit"] == "in":
                    parsed["rain_amount"] = round(rain_mm / 25.4, 4)
                else:
                    parsed["rain_amount"] = round(rain_mm, 4)

                cur.execute(insert_sql, parsed)
    except Exception as exc:
        app.logger.exception("db_insert_error payload=%s error=%s", payload, exc)
        return jsonify({"error": "Database insert failed"}), 500

    app.logger.info("db_insert_success payload=%s", payload)
    return jsonify({"status": "stored"})


if __name__ == "__main__":
    port = int(os.getenv("PAVEWISE_PORT", "8080"))
    app.run(host="0.0.0.0", port=port)
