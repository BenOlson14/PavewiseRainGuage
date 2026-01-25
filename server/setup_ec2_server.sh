#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SERVER_SRC_DIR="${SCRIPT_DIR}"
INSTALL_DIR="/opt/pavewise-rain-gauge"
APP_DIR="${INSTALL_DIR}/server"
ENV_FILE="/etc/pavewise-ingest.env"
SERVICE_FILE="/etc/systemd/system/pavewise-ingest.service"
CONNECTION_FILE="${INSTALL_DIR}/connection_details.txt"

if ! command -v sudo >/dev/null 2>&1; then
  echo "This script requires sudo. Install sudo or run as a user with sudo access." >&2
  exit 1
fi

read -rp "Postgres database name: " DB_NAME
read -rp "Postgres username: " DB_USER
read -rsp "Postgres password: " DB_PASS
echo
read -rp "Server listen port [8080]: " SERVER_PORT
SERVER_PORT=${SERVER_PORT:-8080}

# Port 80/443 require elevated privileges (CAP_NET_BIND_SERVICE) when running as www-data.
# Use a high, non-privileged port by default to avoid service startup failures.
if [[ "${SERVER_PORT}" -lt 1024 ]]; then
  echo "[WARN] Port ${SERVER_PORT} is privileged (<1024). Using 8080 instead so the service can bind as www-data."
  SERVER_PORT=8080
fi
read -rp "HTTP ingest path [/ingest]: " SERVER_PATH
SERVER_PATH=${SERVER_PATH:-/ingest}
DEFAULT_WORKERS=$(command -v nproc >/dev/null 2>&1 && nproc || echo 2)
DEFAULT_WORKERS=$((DEFAULT_WORKERS * 2 + 1))
read -rp "Gunicorn worker count [${DEFAULT_WORKERS}]: " GUNICORN_WORKERS
GUNICORN_WORKERS=${GUNICORN_WORKERS:-${DEFAULT_WORKERS}}
read -rp "Gunicorn threads per worker [4]: " GUNICORN_THREADS
GUNICORN_THREADS=${GUNICORN_THREADS:-4}

if [[ ! "${DB_NAME}" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "Database name must contain only letters, numbers, or underscores." >&2
  exit 1
fi

if [[ ! "${DB_USER}" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "Database username must contain only letters, numbers, or underscores." >&2
  exit 1
fi

DB_PASS_ESC=${DB_PASS//\'/\'\'}

PUBLIC_IP=$(curl -fsS http://checkip.amazonaws.com || true)
PUBLIC_IP=${PUBLIC_IP:-"<your-ec2-public-ip>"}

if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y python3-venv python3-pip postgresql postgresql-contrib curl rsync
else
  echo "Unsupported package manager. This script is intended for Ubuntu (apt-get)." >&2
  exit 1
fi

sudo systemctl enable --now postgresql

PG_CONF=$(sudo -u postgres psql -tAc "SHOW config_file;")
PG_HBA=$(sudo -u postgres psql -tAc "SHOW hba_file;")

if [[ -n "${PG_CONF}" ]]; then
  sudo cp "${PG_CONF}" "${PG_CONF}.bak"
  if sudo grep -q "^[#]*listen_addresses" "${PG_CONF}"; then
    sudo sed -i "s/^[#]*listen_addresses.*/listen_addresses = '*'/g" "${PG_CONF}"
  else
    echo "listen_addresses = '*'" | sudo tee -a "${PG_CONF}" >/dev/null
  fi
fi

if [[ -n "${PG_HBA}" ]]; then
  sudo cp "${PG_HBA}" "${PG_HBA}.bak"
  if ! sudo grep -qE "^host\\s+all\\s+all\\s+0.0.0.0/0\\s+scram-sha-256" "${PG_HBA}"; then
    echo "host    all             all             0.0.0.0/0               scram-sha-256" | sudo tee -a "${PG_HBA}" >/dev/null
  fi
  if ! sudo grep -qE "^host\\s+all\\s+all\\s+::/0\\s+scram-sha-256" "${PG_HBA}"; then
    echo "host    all             all             ::/0                    scram-sha-256" | sudo tee -a "${PG_HBA}" >/dev/null
  fi
fi

sudo systemctl restart postgresql

sudo -u postgres psql -v ON_ERROR_STOP=1 <<SQL
DO \$\$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_roles WHERE rolname = '${DB_USER}') THEN
        CREATE ROLE "${DB_USER}" LOGIN PASSWORD '${DB_PASS_ESC}' CREATEDB;
    ELSE
        ALTER ROLE "${DB_USER}" WITH LOGIN PASSWORD '${DB_PASS_ESC}' CREATEDB;
    END IF;
END
\$\$;
SQL

if ! sudo -u postgres psql -tAc "SELECT 1 FROM pg_database WHERE datname='${DB_NAME}'" | grep -q 1; then
  sudo -u postgres createdb -O "${DB_USER}" "${DB_NAME}"
fi

sudo -u postgres psql -v ON_ERROR_STOP=1 <<SQL
GRANT ALL PRIVILEGES ON DATABASE "${DB_NAME}" TO "${DB_USER}";
SQL

sudo -u postgres psql -v ON_ERROR_STOP=1 -d "${DB_NAME}" <<SQL
GRANT ALL PRIVILEGES ON SCHEMA public TO "${DB_USER}";
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT ALL ON TABLES TO "${DB_USER}";
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT ALL ON SEQUENCES TO "${DB_USER}";
SQL

sudo mkdir -p "${INSTALL_DIR}"
sudo rsync -a --delete "${SERVER_SRC_DIR}/" "${APP_DIR}/"

sudo python3 -m venv "${INSTALL_DIR}/venv"
sudo "${INSTALL_DIR}/venv/bin/pip" install --upgrade pip
sudo "${INSTALL_DIR}/venv/bin/pip" install -r "${APP_DIR}/requirements.txt"

sudo -u postgres psql -d "${DB_NAME}" -f "${APP_DIR}/schema.sql"

sudo tee "${ENV_FILE}" >/dev/null <<ENV
PAVEWISE_DB_HOST=localhost
PAVEWISE_DB_PORT=5432
PAVEWISE_DB_NAME=${DB_NAME}
PAVEWISE_DB_USER=${DB_USER}
PAVEWISE_DB_PASSWORD=${DB_PASS}
PAVEWISE_PORT=${SERVER_PORT}
PAVEWISE_PATH=${SERVER_PATH}
PAVEWISE_GUNICORN_WORKERS=${GUNICORN_WORKERS}
PAVEWISE_GUNICORN_THREADS=${GUNICORN_THREADS}
ENV

sudo chown -R www-data:www-data "${INSTALL_DIR}"
sudo chmod 640 "${ENV_FILE}"
sudo chgrp www-data "${ENV_FILE}"

sudo tee "${SERVICE_FILE}" >/dev/null <<SERVICE
[Unit]
Description=Pavewise Rain Gauge Ingest API
After=network.target postgresql.service

[Service]
Type=simple
User=www-data
Group=www-data
EnvironmentFile=${ENV_FILE}
WorkingDirectory=${APP_DIR}
ExecStart=${INSTALL_DIR}/venv/bin/gunicorn --bind 0.0.0.0:${SERVER_PORT} --worker-class gthread --workers ${GUNICORN_WORKERS} --threads ${GUNICORN_THREADS} app:app
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable --now pavewise-ingest

sudo tee "${CONNECTION_FILE}" >/dev/null <<DETAILS
Pavewise Rain Gauge Server Connection Details
===========================================

HTTP ingest endpoint:
  URL: http://${PUBLIC_IP}:${SERVER_PORT}${SERVER_PATH}

PostgreSQL database (for DB Beaver):
  Host: ${PUBLIC_IP}
  Port: 5432
  Database: ${DB_NAME}
  Username: ${DB_USER}
  Password: ${DB_PASS}

Utilities.h settings to update:
  #define PAVEWISE_SERVER_HOST "${PUBLIC_IP}"
  #define PAVEWISE_SERVER_PORT ${SERVER_PORT}
  #define PAVEWISE_SERVER_PATH "${SERVER_PATH}"

Where to place these values:
  - Open utilities.h in this repo.
  - Replace PAVEWISE_SERVER_HOST/PORT/PATH with the values above.
  - Rebuild and flash the firmware.

Notes:
  - If your EC2 instance uses a security group, allow inbound TCP ${SERVER_PORT} (HTTP) and 5432 (Postgres).
  - Rerun this script any time you change the database credentials or ingest path.
DETAILS

echo
if [[ -f "${CONNECTION_FILE}" ]]; then
  if [[ -t 1 ]] && command -v "${PAGER:-less}" >/dev/null 2>&1; then
    "${PAGER:-less}" "${CONNECTION_FILE}"
  else
    cat "${CONNECTION_FILE}"
  fi
else
  echo "Warning: ${CONNECTION_FILE} was not created. Check disk permissions and rerun." >&2
fi
