import os
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
    if len(parts) not in (4, 6):
        raise ValueError(
            "Invalid payload format. Expected IMEI|BATT_MV|RAIN_X100|EPOCH[|LAT|LON]."
        )

    imei = parts[0]
    batt_mv = int(parts[1])
    rain_x100 = int(parts[2])
    epoch_seconds = int(parts[3])

    lat_deg = None
    lon_deg = None
    has_gps = False

    if len(parts) == 6:
        lat_deg = int(parts[4]) / 1e7
        lon_deg = int(parts[5]) / 1e7
        has_gps = True

    return {
        "imei": imei,
        "batt_mv": batt_mv,
        "batt_v": round(batt_mv / 1000.0, 3),
        "rain_x100": rain_x100,
        "rain_mm": round(rain_x100 / 100.0, 2),
        "epoch_seconds": epoch_seconds,
        "epoch_utc": datetime.fromtimestamp(epoch_seconds, tz=timezone.utc),
        "lat_deg": lat_deg,
        "lon_deg": lon_deg,
        "has_gps": has_gps,
    }


app = Flask(__name__)
PAVEWISE_PATH = os.getenv("PAVEWISE_PATH", "/ingest")


@app.get("/health")
def healthcheck():
    return jsonify({"status": "ok"})


@app.route(PAVEWISE_PATH, methods=["POST"])
def ingest():
    payload = request.get_data(as_text=True) or ""
    payload = payload.strip()
    if not payload:
        return jsonify({"error": "Empty payload"}), 400

    try:
        parsed = parse_payload(payload)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400

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

    with get_db_conn() as conn:
        with conn.cursor() as cur:
            cur.execute(insert_sql, parsed)

    return jsonify({"status": "stored"})


if __name__ == "__main__":
    port = int(os.getenv("PAVEWISE_PORT", "8080"))
    app.run(host="0.0.0.0", port=port)
