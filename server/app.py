import logging
import os
import threading
import time
from datetime import datetime, timezone

import psycopg2
from flask import Flask, jsonify, request


def _env(name: str, default: str | None = None) -> str:
    value = os.getenv(name, default)
    if value is None:
        raise RuntimeError(f"Missing required env var: {name}")
    return value


def get_db_conn():
    return psycopg2.connect(
        host=_env("PAVEWISE_DB_HOST", "localhost"),
        port=int(_env("PAVEWISE_DB_PORT", "5432")),
        dbname=_env("PAVEWISE_DB_NAME"),
        user=_env("PAVEWISE_DB_USER"),
        password=_env("PAVEWISE_DB_PASSWORD"),
        connect_timeout=5,
    )


def parse_payload(raw_payload: str) -> dict:
    parts = [part.strip() for part in raw_payload.strip().split("|") if part.strip()]
    if len(parts) not in (4, 5, 6, 7):
        raise ValueError(
            "Invalid payload format. Expected IMEI|BATT_MV|RAIN_X100|EPOCH[|UNIT][|LAT|LON]."
        )

    imei = parts[0]
    batt_mv = int(parts[1])
    rain_x100 = int(parts[2])
    epoch_seconds = int(parts[3])

    unit = None
    lat_deg = None
    lon_deg = None
    has_gps = False

    if len(parts) in (5, 7):
        unit = parts[4].lower()
        if unit in ("inch", "inches"):
            unit = "in"
        if unit not in ("mm", "in"):
            unit = None

    if len(parts) == 6:
        lat_deg = int(parts[4]) / 1e7
        lon_deg = int(parts[5]) / 1e7
        has_gps = True
    elif len(parts) == 7:
        lat_deg = int(parts[5]) / 1e7
        lon_deg = int(parts[6]) / 1e7
        has_gps = True

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
    return jsonify({"status": "ok"})


@app.route(PAVEWISE_PATH, methods=["POST"])
def ingest():
    payload = request.get_data(as_text=True) or ""
    payload = payload.strip()
    if not payload:
        app.logger.warning("ingest_empty_payload")
        return jsonify({"error": "Empty payload"}), 400

    try:
        parsed = parse_payload(payload)
    except ValueError as exc:
        app.logger.warning("ingest_parse_error payload=%s error=%s", payload, exc)
        return jsonify({"error": str(exc)}), 400

    app.logger.info("ingest_request payload=%s", payload)
    app.logger.info("ingest_parsed payload=%s parsed=%s", payload, parsed)

    insert_sql = """
        INSERT INTO rain_gauge_readings (
            imei,
            batt_mv,
            batt_v,
            rain_x100,
            rain_mm,
            epoch_seconds,
            epoch_utc,
            lat_deg,
            lon_deg,
            has_gps
        )
        VALUES (%(imei)s, %(batt_mv)s, %(batt_v)s, %(rain_x100)s, %(rain_mm)s,
                %(epoch_seconds)s, %(epoch_utc)s, %(lat_deg)s, %(lon_deg)s, %(has_gps)s)
    """

    try:
        app.logger.info("db_insert_attempt payload=%s", payload)
        with get_db_conn() as conn:
            with conn.cursor() as cur:
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

                if parsed["rain_unit"] == "in":
                    parsed["rain_mm"] = round((parsed["rain_x100"] / 100.0) * 25.4, 2)
                else:
                    parsed["rain_mm"] = round(parsed["rain_x100"] / 100.0, 2)

                cur.execute(insert_sql, parsed)
    except Exception as exc:
        app.logger.exception("db_insert_error payload=%s error=%s", payload, exc)
        return jsonify({"error": "Database insert failed"}), 500

    app.logger.info("db_insert_success payload=%s", payload)
    return jsonify({"status": "stored"})


if __name__ == "__main__":
    port = int(os.getenv("PAVEWISE_PORT", "8080"))
    app.run(host="0.0.0.0", port=port)
