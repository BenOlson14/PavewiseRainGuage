#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
SERVER_SRC_DIR="${REPO_ROOT}/server"
INSTALL_DIR="/opt/pavewise-rain-gauge"
APP_DIR="${INSTALL_DIR}/server"
ENV_FILE="/etc/pavewise-ingest.env"
SERVICE_FILE="/etc/systemd/system/pavewise-ingest.service"
CONNECTION_FILE="${INSTALL_DIR}/connection_details.txt"

if [[ $(id -u) -eq 0 ]]; then
  echo "Please run this script as a non-root user with sudo access." >&2
  exit 1
fi

read -rp "Postgres database name: " DB_NAME
read -rp "Postgres username: " DB_USER
read -rsp "Postgres password: " DB_PASS
echo
read -rp "Server listen port [8080]: " SERVER_PORT
SERVER_PORT=${SERVER_PORT:-8080}
read -rp "HTTP ingest path [/ingest]: " SERVER_PATH
SERVER_PATH=${SERVER_PATH:-/ingest}

PUBLIC_IP=$(curl -fsS http://checkip.amazonaws.com || true)
PUBLIC_IP=${PUBLIC_IP:-"<your-ec2-public-ip>"}

sudo apt-get update
sudo apt-get install -y python3-venv python3-pip postgresql postgresql-contrib curl rsync

sudo systemctl enable --now postgresql

sudo -u postgres psql -v db_user="${DB_USER}" -v db_pass="${DB_PASS}" -v db_name="${DB_NAME}" <<'SQL'
DO $$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_roles WHERE rolname = :'db_user') THEN
        CREATE ROLE :"db_user" LOGIN PASSWORD :'db_pass';
    END IF;
END
$$;

DO $$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_database WHERE datname = :'db_name') THEN
        EXECUTE format('CREATE DATABASE %I OWNER %I', :'db_name', :'db_user');
    END IF;
END
$$;
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
ExecStart=${INSTALL_DIR}/venv/bin/gunicorn --bind 0.0.0.0:${SERVER_PORT} app:app
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
