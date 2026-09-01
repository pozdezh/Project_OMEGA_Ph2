#!/usr/bin/env bash
# Omega Brick 3 (true-DTLS) server installer.
#
# Idempotent: safe to run on a bare Ubuntu box or to re-run for an update. It
# auto-detects fresh vs update, installs ALL Python deps in both modes, sets up
# the DTLS listener + dashboard as systemd services, and schedules the DB
# retention and daily-stats cron jobs. Every failure exits with a distinct [Exx]
# code. Run `./install_server.sh --check` for a non-destructive health report.
#
# Corrections folded in from field testing:
#  - device_config.json in the OLD (Brick 1/2) format is replaced, never kept,
#    so the roster is never empty (previously every device was rejected);
#  - the venv gets the FULL requirements in update mode too, not just crypto
#    (previously flask/dtls/mcp could be missing after an update);
#  - certificates are validated up front so the listener never boots keyless.

set -u
INSTALLER_VERSION="2026-08-29a-wolfssl-pinned"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PAYLOAD="$SCRIPT_DIR/payload"
# MUST match make-units' OMEGA and the directory the live server actually
# runs from. They disagreed until 2026-08-25: the installer targeted
# ~/omega_server, the unit maker looked in ~/omega_brick4, and the running
# server was ~/omega_brick4 - so the installer had never in fact built the
# server it claimed to install, and on a fresh machine it would have left the
# unit maker searching an empty directory.
TARGET="${OMEGA_HOME:-$HOME/omega_brick4}"
VENV="$TARGET/venv"
PY="$VENV/bin/python"
PIP="$VENV/bin/pip"
PKI_SRC="${OMEGA_PKI_SRC:-$SCRIPT_DIR/pki}"
SERVICE_USER="${SUDO_USER:-$USER}"

log()  { echo "[omega] $*"; }
err()  { echo "[omega][ERROR $1] ${*:2}" >&2; }
die()  { err "$1" "${*:2}"; exit "$1"; }

banner() {
  echo "=================================================="
  echo " Omega Brick 4 server installer  ($INSTALLER_VERSION)"
  echo " target: $TARGET"
  echo "=================================================="
}

require_linux() {
  [ "$(uname -s)" = "Linux" ] || die 10 "this installer targets Linux (the Ubuntu server)"
}

require_python() {
  command -v python3 >/dev/null 2>&1 || die 11 "python3 not found; run: sudo apt install -y python3 python3-venv"
  python3 -c 'import venv' 2>/dev/null || die 11 "python3-venv missing; run: sudo apt install -y python3-venv"
}

ensure_venv() {
  if [ ! -x "$PY" ]; then
    log "creating venv at $VENV"
    python3 -m venv "$VENV" || die 20 "venv creation failed"
  fi
}

ensure_requirements() {
  # Runs in BOTH fresh and update mode - this is the field fix for missing deps.
  log "installing/upgrading server requirements (this can take a minute)"
  "$PIP" install --upgrade pip >/dev/null 2>&1
  "$PIP" install -r "$PAYLOAD/requirements.txt" || die 21 "pip install failed"
  "$PY" -c "import wolfssl, flask, cryptography, mcp, zeroconf" 2>/dev/null \
    || die 21 "a required package did not import after install (wolfssl/flask/cryptography/mcp/zeroconf)"
}

copy_payload() {
  mkdir -p "$TARGET/templates" "$TARGET/pki"
  # Replace code every time; treat config specially below.
  for f in listener.py session.py storage.py cause_validation.py config_store.py acks.py \
           nmu_mailbox.py identity_guard.py db_retention.py daily_stats.py clear_database.py \
           device_api.py mcp_server.py device_live.py discovery.py app.py \n           device_addresses.py requirements.txt; do
    cp "$PAYLOAD/$f" "$TARGET/$f"
  done
  cp "$PAYLOAD/templates/index.html" "$TARGET/templates/index.html"
}

install_config() {
  # Field fix: an existing config that is NOT already in Brick 3 format (no
  # nmu/amu blocks) is stale and must be replaced, or the roster is empty and
  # every device is rejected. A valid Brick 3 config is preserved so operator
  # heartbeat/revocation edits survive an update.
  local dst="$TARGET/device_config.json"
  if [ -f "$dst" ] && "$PY" -c "import json,sys; d=json.load(open('$dst')); sys.exit(0 if ('nmu' in d and 'amu' in d) else 1)" 2>/dev/null; then
    log "keeping existing Brick 3 device_config.json"
  else
    log "installing Brick 3 device_config.json (replacing any stale config)"
    cp "$PAYLOAD/device_config.json" "$dst"
  fi
}

install_pki() {
  local need="ca-cert.pem omega-server-cert.pem omega-server-key.pem"
  for f in $need; do
    [ -f "$PKI_SRC/$f" ] || die 30 "missing $f in $PKI_SRC; generate with omega_pki.py init and copy the CA + server cert/key here"
  done
  cp "$PKI_SRC/ca-cert.pem" "$TARGET/pki/"
  cp "$PKI_SRC/omega-server-cert.pem" "$TARGET/pki/"
  cp "$PKI_SRC/omega-server-key.pem" "$TARGET/pki/"
  chmod 600 "$TARGET/pki/omega-server-key.pem"
  log "installed server certificate + CA (server key mode 600)"
}

write_service() {
  # $1 name, $2 description, $3 exec-args
  local name="$1" desc="$2" args="$3"
  sudo tee "/etc/systemd/system/$name.service" >/dev/null <<EOF || die 40 "cannot write $name.service"
[Unit]
Description=$desc
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$TARGET
Environment=OMEGA_PKI_DIR=$TARGET/pki
Environment=OMEGA_DEVICE_CONFIG=$TARGET/device_config.json
Environment=OMEGA_DB=$TARGET/sensor_data.db
ExecStart=$PY $args
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
}

install_services() {
  # Brick 1 is frozen and uses the conflicting legacy ports 5000/8080.
  # Disable it so only the active Brick 4 stack starts after boot.
  sudo systemctl disable --now smart_listener.service smart_web.service \
    >/dev/null 2>&1 || true
  write_service "omega-listener" "Omega true-DTLS listener" "$TARGET/listener.py"
  write_service "omega-web" "Omega dashboard" "$TARGET/app.py"
  sudo systemctl daemon-reload || die 40 "systemctl daemon-reload failed"
  sudo systemctl enable --now omega-listener omega-web >/dev/null 2>&1 || die 40 "failed to enable services"
  sudo systemctl restart omega-listener omega-web || die 40 "failed to (re)start services"
  log "services omega-listener + omega-web active"
}

install_cron() {
  sudo tee /etc/cron.d/omega-maintenance >/dev/null <<EOF || die 41 "cannot write cron file"
# Omega Brick 4 maintenance ($INSTALLER_VERSION)
OMEGA_DB_MAX_SIZE_MB=500
15 3 * * * $SERVICE_USER $PY $TARGET/db_retention.py $TARGET/sensor_data.db >> $TARGET/retention.log 2>&1
30 3 * * * $SERVICE_USER $PY $TARGET/daily_stats.py $TARGET/sensor_data.db >> $TARGET/stats.log 2>&1
EOF
  log "cron installed: retention 03:15, daily stats 03:30 (set OMEGA_STATS_URL in the cron env to enable the push)"
}

install_gateway() {
  # Reach the dashboard WITHOUT knowing the server's IP address, over HTTPS,
  # and only from a machine holding a certificate this project's own
  # authority issued.
  #
  # The router hands out addresses and can change them, so telling a carer
  # "open 192.168.0.112:8081" is a promise that breaks on the next reboot.
  # Avahi publishes the name smartageing.local on the local network (the same
  # mechanism that makes printers appear by name); nginx terminates HTTPS on
  # port 443 and forwards to the dashboard on 8081 internally.
  #
  # mTLS (mutual TLS - both sides present a certificate, not just the server)
  # is enforced with ssl_verify_client: a browser with no client certificate,
  # or one not signed by omega-ca, is refused at the TLS handshake - the
  # request never reaches the dashboard code at all. This is the same
  # authority and the same enforcement point as the device fleet: one trust
  # model for humans and devices both, not two parallel security stories.
  # Plain HTTP on :80 issues a redirect to HTTPS rather than serving content.
  if ! command -v nginx >/dev/null 2>&1; then
    sudo apt-get install -y nginx >/dev/null 2>&1 || {
      log "WARNING: nginx not installed - dashboard stays on :8081 only"
      return 0
    }
  fi
  if ! command -v avahi-daemon >/dev/null 2>&1; then
    sudo apt-get install -y avahi-daemon >/dev/null 2>&1 || {
      log "WARNING: avahi not installed - .local name unavailable"
    }
  fi

  local cert="$TARGET/pki/omega-server-cert.pem"
  local key="$TARGET/pki/omega-server-key.pem"
  local ca="$TARGET/pki/ca-cert.pem"
  if [ ! -f "$cert" ] || [ ! -f "$key" ] || [ ! -f "$ca" ]; then
    log "WARNING: server cert/key/CA not found in $TARGET/pki - dashboard stays on :8081"
    return 0
  fi

  sudo tee /etc/nginx/sites-available/omega >/dev/null <<NGINX || { log "WARNING: could not write nginx site"; return 0; }
server {
    listen 80 default_server;
    server_name smartageing.local _;
    return 301 https://\$host\$request_uri;
}

server {
    listen 443 ssl default_server;
    server_name smartageing.local _;

    ssl_certificate $cert;
    ssl_certificate_key $key;
    ssl_client_certificate $ca;
    ssl_verify_client on;

    location / {
        proxy_pass http://127.0.0.1:8081;
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-SSL-Client-CN \$ssl_client_s_dn;
    }

    # Live-update push stream: nginx buffers proxied responses by default,
    # which would hold every event in a buffer instead of forwarding it -
    # silently turning the push stream back into a delayed poll. Off here,
    # and app.py also sends X-Accel-Buffering: no on this route directly.
    location /api/stream {
        proxy_pass http://127.0.0.1:8081;
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-SSL-Client-CN \$ssl_client_s_dn;
        proxy_buffering off;
        proxy_cache off;
        proxy_read_timeout 1h;
    }
}
NGINX
  sudo ln -sf /etc/nginx/sites-available/omega /etc/nginx/sites-enabled/omega
  sudo rm -f /etc/nginx/sites-enabled/default
  if sudo nginx -t >/dev/null 2>&1 && sudo systemctl restart nginx >/dev/null 2>&1; then
    log "gateway up: https://smartageing.local (client certificate required)"
  else
    # Never a quiet warning: without this gateway the dashboard and its
    # config-writing routes are reachable on plain :8081 by anyone on the LAN.
    log "SECURITY: nginx gateway did NOT come up. The dashboard is served"
    log "SECURITY: unauthenticated over plain HTTP on :8081. Diagnose with"
    log "SECURITY: 'sudo nginx -t' before letting anyone use this server."
    sudo nginx -t 2>&1 | sed 's/^/    /'
  fi

  # _omega._udp is NOT advertised from a static file here. listener.py
  # registers it at runtime with python-zeroconf (discovery.register_mdns),
  # so the advertisement exists only while the service that answers it is
  # actually running. A static file would keep advertising a dead port
  # whenever the listener is down, and would sit alongside the live record
  # as a second, contradictory answer to the same query.
  #
  # avahi-daemon is still required, for the smartageing.local HOSTNAME the
  # dashboard is reached by. That is a different job from service discovery.
  sudo systemctl enable --now avahi-daemon >/dev/null 2>&1 || true
  log "avahi-daemon up for smartageing.local; _omega._udp is advertised by"
  log "the listener itself, only while it is running"
  return 0
}

doctor() {
  banner
  echo "-- health check --"
  [ -x "$PY" ] && echo "venv: OK" || echo "venv: MISSING"
  "$PY" -c "import wolfssl, flask, cryptography, mcp, zeroconf" 2>/dev/null && echo "deps: OK" || echo "deps: MISSING (re-run installer)"
  for f in ca-cert.pem omega-server-cert.pem omega-server-key.pem; do
    [ -f "$TARGET/pki/$f" ] && echo "pki $f: OK" || echo "pki $f: MISSING"
  done
  [ -f "$TARGET/device_config.json" ] && "$PY" -c "import json; d=json.load(open('$TARGET/device_config.json')); assert 'nmu' in d and 'amu' in d" 2>/dev/null \
    && echo "config: OK (Brick 3 format)" || echo "config: MISSING or stale"
  systemctl is-active --quiet omega-listener && echo "listener: ACTIVE" || echo "listener: NOT active"
  systemctl is-active --quiet omega-web && echo "web: ACTIVE" || echo "web: NOT active"
}

main() {
  require_linux
  if [ "${1:-}" = "--check" ]; then doctor; exit 0; fi
  banner
  require_python
  ensure_venv
  ensure_requirements
  copy_payload
  install_config
  install_pki
  install_services
  install_cron
  install_gateway
  echo "--------------------------------------------------"
  log "install complete. Dashboard: https://smartageing.local (operator certificate required)"
  log "health check any time: $0 --check"
}

main "$@"
