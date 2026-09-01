#!/usr/bin/env bash
# Omega Brick 3 (true-DTLS) AMU installer for the Raspberry Pi.
#
# Idempotent: fresh install or re-run update. Sets up the DTLS sensor service
# under systemd. Every failure exits with a distinct [Exx] code. Run
# `./install_amu.sh --check` for a non-destructive health report.
#
# Corrections folded in from field testing:
#  - REFUSES to run as root [E51]: running the installer with sudo previously
#    left a root-owned copy AND the user service both driving GPIO4, which
#    fought over the DHT pin ("Unable to set line 4 to input");
#  - sensor libraries are installed into the service's OWN venv, never with
#    --user, and `import board` is verified before success (previously the
#    service died with ModuleNotFoundError: No module named 'board').

set -u
INSTALLER_VERSION="2026-08-29a-wolfssl-pinned"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PAYLOAD="$SCRIPT_DIR/payload"
TARGET="${OMEGA_HOME:-$HOME/omega_amu}"
VENV="$TARGET/venv"
PY="$VENV/bin/python"
PIP="$VENV/bin/pip"
PKI_SRC="${OMEGA_PKI_SRC:-$SCRIPT_DIR/pki}"

# The inbound live-query agent runs on its OWN interpreter, separate from the
# telemetry service above. See install_service() and FINDINGS #35 for why:
# the wolfSSL binding is fatally broken on CPython 3.12+, which is what this
# Pi ships. Built by build_live_py.sh; absent is not fatal, the installer
# just skips the agent and says so.
LIVE_VENV="$TARGET/venv311"
LIVE_PY="$LIVE_VENV/bin/python"

log() { echo "[omega] $*"; }
die() { echo "[omega][ERROR $1] ${*:2}" >&2; exit "$1"; }

guard_not_root() {
  if [ "$(id -u)" = "0" ]; then
    die 51 "do NOT run the AMU installer as root/sudo. Run as the pi user, e.g. ./install_amu.sh. Root creates a second GPIO owner that fights the sensor service."
  fi
}

banner() {
  echo "=================================================="
  echo " Omega Brick 4 AMU installer  ($INSTALLER_VERSION)"
  echo " target: $TARGET   user: $USER"
  echo "=================================================="
}

require_python() {
  command -v python3 >/dev/null 2>&1 || die 11 "python3 not found"
  python3 -c 'import venv' 2>/dev/null || die 11 "python3-venv missing; run: sudo apt install -y python3-venv"
}

# Packages the pip step cannot do without, installed BEFORE it runs.
#
# The wolfSSL binding has no prebuilt wheel for this Pi's Python, so pip
# compiles the wolfSSL C library from source - which needs autotools and a
# compiler. A fresh Raspberry Pi OS has neither, and the failure is a
# hundred-line traceback ending in "FileNotFoundError: libtoolize", which
# reads like a broken package rather than a missing build tool.
#
# This installer used to assume these were present. They were, on the Pi that
# had been used for development, and on no other.
APT_BUILD_DEPS="build-essential autoconf automake libtool pkg-config python3-dev python3-venv"

ensure_system_packages() {
  local missing=""
  for pkg in $APT_BUILD_DEPS; do
    dpkg -s "$pkg" >/dev/null 2>&1 || missing="$missing $pkg"
  done
  if [ -z "$missing" ]; then
    log "system build packages already present"
    return 0
  fi

  log "installing system packages needed to build wolfSSL:$missing"
  log "(this is a one-time cost on a fresh Raspberry Pi OS)"
  if ! sudo -n true 2>/dev/null; then
    die 12 "need sudo to install:$missing - run 'sudo apt-get install -y$missing' then re-run this installer"
  fi
  sudo apt-get update -qq || die 12 "apt-get update failed"
  sudo apt-get install -y $missing || die 12 "could not install:$missing"
  log "system packages installed"
}

ensure_venv() {
  if [ ! -x "$PY" ]; then
    log "creating venv (with system site packages for GPIO/I2C access)"
    python3 -m venv --system-site-packages "$VENV" || die 20 "venv creation failed"
  fi
}

ensure_sensor_deps() {
  # Install into $PY, NEVER --user, then prove the hardware bridge imports.
  log "installing sensor + DTLS deps into the service venv"
  "$PIP" install --upgrade pip >/dev/null 2>&1
  # A wheel that shipped with this bundle is used in preference to building
  # from source. wolfSSL has no published wheel for this Pi's Python, and
  # compiling it takes several minutes; the result is identical on identical
  # hardware, so it is built once and carried. --find-links adds the folder
  # as a source, it does not force it: anything not present there is still
  # fetched or built exactly as before.
  local wheel_args=""
  if ls "$SCRIPT_DIR/wheels"/*.whl >/dev/null 2>&1; then
    wheel_args="--find-links $SCRIPT_DIR/wheels"
    log "using $(ls "$SCRIPT_DIR/wheels"/*.whl | wc -l) prebuilt wheel(s) - no compiler needed"
  fi
  "$PIP" install $wheel_args -r "$PAYLOAD/requirements_delta.txt" || die 21 "pip install failed"
  "$PY" -c "import board" 2>/dev/null \
    || die 21 "'import board' failed after install - the service would not start. Check that I2C/SPI are enabled (sudo raspi-config)."
  "$PY" -c "import wolfssl, zeroconf" 2>/dev/null || die 21 "wolfssl/zeroconf did not import"
  log "sensor bridge verified (import board OK)"
}

copy_payload() {
  mkdir -p "$TARGET/config" "$TARGET/pki"
  cp "$PAYLOAD/main.py" "$TARGET/"
  cp "$PAYLOAD/amu_config.py" "$TARGET/"
  cp "$PAYLOAD/sensors.py" "$TARGET/"
  cp "$PAYLOAD/triggers.py" "$TARGET/"
  cp "$PAYLOAD/buffer.py" "$TARGET/"
  cp "$PAYLOAD/recovery.py" "$TARGET/"
  cp "$PAYLOAD/retry_schedule.py" "$TARGET/"
  cp "$PAYLOAD/applog.py" "$TARGET/"
  cp "$PAYLOAD/clock.py" "$TARGET/"
  cp "$PAYLOAD/network.py" "$TARGET/"
  cp "$PAYLOAD/dtls_client.py" "$TARGET/"
  cp "$PAYLOAD/wolfssl_guard.py" "$TARGET/"
  cp "$PAYLOAD/server_discovery.py" "$TARGET/"
  cp "$PAYLOAD/live_server.py" "$TARGET/"
  cp "$PAYLOAD/live_agent.py" "$TARGET/"
  cp "$PAYLOAD/live_cache.py" "$TARGET/"
  cp "$PAYLOAD/requirements_delta.txt" "$TARGET/"
  # A bundle built by make-amu-bundle ships a config already filled in for
  # this unit, so there is nothing to edit on the Pi. That matters more than
  # it looks: the device_id in this file must match the certificate in pki/,
  # and a hand-edited mismatch is only discovered later, as a refused
  # handshake, on a unit already screwed to a wall.
  if [ -f "$TARGET/config/global.ini" ]; then
    log "keeping existing config/global.ini"
  elif [ -f "$SCRIPT_DIR/config/global.ini" ]; then
    cp "$SCRIPT_DIR/config/global.ini" "$TARGET/config/global.ini"
    log "installed the config that shipped with this bundle - nothing to edit"
  else
    cp "$PAYLOAD/config/global.ini.example" "$TARGET/config/global.ini"
    log "created config/global.ini from example - EDIT it (device_id, WLAN, server_host) before starting"
  fi
}

install_pki() {
  local dev
  dev="$(grep -E '^device_id' "$TARGET/config/global.ini" | head -1 | awk -F= '{gsub(/ /,"",$2); print $2}')"
  [ -n "$dev" ] || die 50 "device_id not set in config/global.ini"
  for f in ca-cert.pem "$dev-cert.pem" "$dev-key.pem"; do
    [ -f "$PKI_SRC/$f" ] || die 30 "missing $f in $PKI_SRC; generate on the server (omega_pki.py) and copy this unit's cert/key + ca-cert.pem here"
  done
  cp "$PKI_SRC/ca-cert.pem" "$TARGET/pki/"
  cp "$PKI_SRC/$dev-cert.pem" "$TARGET/pki/"
  cp "$PKI_SRC/$dev-key.pem" "$TARGET/pki/"
  chmod 600 "$TARGET/pki/$dev-key.pem"
  log "installed identity for $dev (device key mode 600)"
}

install_persistent_log() {
  # Make the unit's log survive its own reboot.
  #
  # The recovery ladder's last rung reboots the box, so with a RAM-only
  # journal the single most interesting event ERASES ITS OWN EVIDENCE. That
  # happened for real on 2026-08-27: AMU_15 rebooted itself unaided - the
  # first time the ladder ever reached that rung - and the rung-1 and rung-2
  # lines that led up to it were gone by the time anyone could read them.
  #
  # Storage=auto only persists when /var/log/journal already exists when
  # journald starts. On these cards the directory existed but was empty, so
  # "auto" quietly meant "volatile". Say persistent and mean it.
  sudo mkdir -p /var/log/journal
  sudo systemd-tmpfiles --create --prefix /var/log/journal >/dev/null 2>&1 || true
  if ! grep -qE '^Storage=persistent' /etc/systemd/journald.conf; then
    sudo sed -i 's/^#\?Storage=.*/Storage=persistent/' /etc/systemd/journald.conf
  fi
  # Bounded, because these are SD cards: a runaway log is its own outage.
  if ! grep -qE '^SystemMaxUse=' /etc/systemd/journald.conf; then
    echo 'SystemMaxUse=200M' | sudo tee -a /etc/systemd/journald.conf >/dev/null
  fi
  sudo systemctl restart systemd-journald >/dev/null 2>&1 || true
  log "persistent journal enabled (survives the ladder's own reboot, capped 200M)"
}

install_recovery_privileges() {
  # The ladder has two rungs that need a root privilege the "amu" account does
  # not otherwise have: rung 1 brings WiFi back up (nmcli), rung 3 reboots the
  # unit when a process restart cannot clear a wedged kernel WiFi driver and
  # nobody is there to press its button. Scoped to exactly those two commands
  # - never a blanket ALL - so a compromised or buggy AMU process can move the
  # network and reboot the box, and nothing else, as root.
  #
  # The nmcli grant was missing from every install/clone before 2026-08-26.
  # Every deployed unit could reboot itself but never reconnect its own WiFi -
  # rung 1 always failed silently (sudo just re-prompted for a password nmcli
  # was never allowed to skip), so every real outage climbed straight past it
  # to a full reboot. Found on AMU_15 by running the exact ladder command by
  # hand and reading its exit code.
  local rule=/etc/sudoers.d/omega-amu-reboot
  local tmp
  tmp=$(mktemp)
  printf '%s ALL=(root) NOPASSWD: /sbin/reboot
%s ALL=(root) NOPASSWD: /usr/bin/nmcli connection up *
%s ALL=(root) NOPASSWD: /usr/bin/nmcli dev wifi connect *
' "$(id -un)" "$(id -un)" "$(id -un)" > "$tmp"
  if sudo visudo -c -f "$tmp" >/dev/null 2>&1; then
    sudo install -m 0440 -o root -g root "$tmp" "$rule"
    log "recovery privileges installed ($rule): reboot + nmcli reconnect"
  else
    log "WARNING: sudoers rule failed validation - the recovery ladder will"
    log "WARNING: not be able to reconnect WiFi or reboot the unit as root."
  fi
  rm -f "$tmp"
}

install_service() {
  sudo tee /etc/systemd/system/omega-amu.service >/dev/null <<EOF || die 40 "cannot write service"
[Unit]
Description=Omega Brick 3 AMU (true-DTLS)
After=network-online.target
Wants=network-online.target

StartLimitIntervalSec=0

[Service]
Type=simple
User=$USER
WorkingDirectory=$TARGET
Environment=PYTHONUNBUFFERED=1
ExecStart=$PY $TARGET/main.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
  # Second unit, second PROCESS, and deliberately a DIFFERENT INTERPRETER.
  #
  # The wolfSSL Python binding mismanages CPython thread state on its DTLS
  # server path. CPython 3.12 added strict checking that turns that latent
  # bug into an instant fatal error, so on the Pi's system Python 3.13 this
  # listener dies after serving exactly one query (FINDINGS #35). The
  # binding's own PyPI metadata never claimed 3.12+ support, and 5.9.2.post0
  # is the newest release, so there is no upstream fix to wait for.
  #
  # $LIVE_PY is a locally built Python 3.11 - the newest release below that
  # threshold. main.py keeps the system interpreter and is untouched. Only a
  # separate PROCESS can run a different interpreter, which is why this is a
  # second unit rather than a thread.
  if [ ! -x "$LIVE_PY" ]; then
    log "WARNING: $LIVE_PY not found - inbound live-query agent NOT installed."
    log "         Telemetry is UNAFFECTED and starts normally below."
    log "         To enable live queries, run:  bash $SCRIPT_DIR/build_live_py.sh"
    log "         (~25 min, no sudo), then re-run this installer."
    sudo systemctl daemon-reload || true
    sudo systemctl enable --now omega-amu >/dev/null 2>&1 || die 40 "enable failed"
    sudo systemctl restart omega-amu || die 40 "restart failed"
    log "service omega-amu active (live agent skipped)"
    return 0
  fi
  sudo tee /etc/systemd/system/omega-amu-live.service >/dev/null <<EOF || die 40 "cannot write live service"
[Unit]
Description=Omega Brick 4 AMU inbound live-query agent
After=network-online.target omega-amu.service
Wants=network-online.target

StartLimitIntervalSec=0

[Service]
Type=simple
User=$USER
WorkingDirectory=$TARGET
Environment=PYTHONUNBUFFERED=1
ExecStart=$LIVE_PY $TARGET/live_agent.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
  sudo systemctl daemon-reload || die 40 "daemon-reload failed"
  sudo systemctl enable --now omega-amu >/dev/null 2>&1 || die 40 "enable failed"
  sudo systemctl restart omega-amu || die 40 "restart failed"
  log "service omega-amu active"
  sudo systemctl enable --now omega-amu-live >/dev/null 2>&1 || die 40 "enable live agent failed"
  sudo systemctl restart omega-amu-live || die 40 "restart live agent failed"
  log "service omega-amu-live active (inbound live-query agent, own process)"
}

doctor() {
  banner
  echo "-- health check --"
  [ -x "$PY" ] && echo "venv: OK" || echo "venv: MISSING"
  "$PY" -c "import board" 2>/dev/null && echo "sensor bridge (board): OK" || echo "sensor bridge: MISSING (enable I2C, re-run)"
  "$PY" -c "import wolfssl, zeroconf" 2>/dev/null && echo "dtls13+discovery: OK" || echo "dtls13+discovery: MISSING"
  [ -f "$TARGET/config/global.ini" ] && echo "config: present" || echo "config: MISSING"
  ls "$TARGET/pki"/*-key.pem >/dev/null 2>&1 && echo "identity cert: present" || echo "identity cert: MISSING"
  systemctl is-active --quiet omega-amu && echo "service: ACTIVE" || echo "service: NOT active"
}

main() {
  guard_not_root
  if [ "${1:-}" = "--check" ]; then doctor; exit 0; fi
  banner
  require_python
  ensure_system_packages
  ensure_venv
  ensure_sensor_deps
  copy_payload
  install_pki
  install_recovery_privileges
  install_persistent_log
  install_service
  echo "--------------------------------------------------"
  log "install complete. Follow logs: journalctl -u omega-amu -f"
  log "health check any time: $0 --check"
}

main "$@"
