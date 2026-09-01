#!/usr/bin/env bash
# Omega NMU factory - runs ON THE SERVER, the machine that holds the CA.
#
# Asks four questions, then produces one self-contained ZIP per batch. The ZIP
# is everything a person needs to turn bare ESP32-S3 boards into working NMU
# units on ANY Windows PC: no toolchain, no downloads, no account, no internet.
#
# The CA private key never leaves this machine. Only signed certificates, which
# are public material, travel inside the ZIP.
#
#   ./nmu_factory.sh                 interactive
#   ./nmu_factory.sh batch.conf      unattended (see --example)
#
# See ARCHITECTURE.md 19 for why the server is the factory.
#
# This tool BUILDS units. It does not flash them: the ZIP it produces is what
# does the flashing, on any Windows PC, with no toolchain.

set -u

FACTORY_VERSION="2026-08-24a-pendrive-ca"

HOME_DIR="${HOME:-/home/smart}"
OMEGA="$HOME_DIR/omega_brick4"
PKI_DIR="$OMEGA/pki_provisioning"
PKI_TOOL="$OMEGA/omega_pki.py"
IDENTITY_TOOL="$OMEGA/identity_guard.py"
IDENTITY_DB="$OMEGA/device_identities.json"
PYTHON="$HOME_DIR/omega_spike13/venv/bin/python"
SRC="$HOME_DIR/omega_build/stage3_beta_2_dtls"
TEMPLATE="$HOME_DIR/nmu_factory_assets"
OUT_ROOT="$HOME_DIR/nmu_factory_output"
ARDUINO_CLI="$HOME_DIR/bin/arduino-cli"

SKETCH="stage3_beta_2_dtls"
FQBN="esp32:esp32:adafruit_feather_esp32s3_nopsram:CDCOnBoot=cdc"
SERVER_PORT=11400
HANDSHAKE_TIMEOUT_MS=8000
ACK_TIMEOUT_MS=3000

say()  { echo "  $*"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

# The CA private key does not have to live on this machine's disk at all - the
# running server never reads it (only the DTLS listener's own certificate and
# the public CA certificate). find_ca_key_dir looks for a MOUNTED drive
# carrying ca-key.pem first, so a pendrive that is plugged in only while making
# units is the normal case, not a fallback. $PKI_DIR/ca-key.pem (the key
# resident on this machine) is still honoured if present, for a server that
# deliberately keeps it locally.
find_ca_key_dir() {
  if [ -f "$PKI_DIR/ca-key.pem" ]; then
    echo "$PKI_DIR"
    return 0
  fi
  local hit
  hit=$(find /media /mnt /run/media -mindepth 1 -maxdepth 4 \
             -name ca-key.pem -not -path '*/.*' 2>/dev/null | head -1)
  if [ -n "$hit" ]; then
    dirname "$hit"
    return 0
  fi
  return 1
}

preflight() {
  local missing=0
  for path in "$PKI_TOOL" "$PYTHON" "$ARDUINO_CLI"; do
    [ -x "$path" ] || [ -f "$path" ] || { echo "MISSING: $path" >&2; missing=1; }
  done
  [ -d "$SRC" ]      || { echo "MISSING firmware source: $SRC" >&2; missing=1; }
  [ -d "$PKI_DIR" ]  || { echo "MISSING PKI directory: $PKI_DIR" >&2; missing=1; }
  [ -f "$PKI_DIR/ca-cert.pem" ] || {
    echo "MISSING CA certificate: $PKI_DIR/ca-cert.pem" >&2
    echo "This is the public trust anchor, not the private key - it should" >&2
    echo "already be here from setting up this server. Restore it before" >&2
    echo "trying again." >&2
    missing=1; }
  CA_KEY_DIR=$(find_ca_key_dir) || {
    echo "MISSING CA private key: not at $PKI_DIR/ca-key.pem and no mounted" >&2
    echo "drive under /media, /mnt or /run/media carries one either." >&2
    echo "Plug in the pendrive holding ca-key.pem and try again." >&2
    missing=1; }
  for asset in esptool.exe boot_app0.bin; do
    [ -f "$TEMPLATE/$asset" ] || {
      echo "MISSING flash-pack asset: $TEMPLATE/$asset" >&2; missing=1; }
  done
  command -v zip >/dev/null 2>&1 || { echo "MISSING: zip (sudo apt install zip)" >&2; missing=1; }
  [ "$missing" -eq 0 ] || fail "preflight failed - nothing was built"
  if [ "$CA_KEY_DIR" != "$PKI_DIR" ]; then
    say "CA private key found on removable media: $CA_KEY_DIR"
  fi
}

name_taken() {
  # Taken means: already has a certificate on this server, OR was already
  # chosen earlier in this same batch. Both would put one identity on two
  # boards, which the server's clone detector then rejects in the field.
  # REBUILD_EXISTING=1 allows an existing name deliberately, for re-flashing
  # a unit that is already in the fleet.
  local candidate="$1"
  case " ${CHOSEN:-} " in *" $candidate "*) return 0 ;; esac
  [ -n "${REBUILD_EXISTING:-}" ] && return 1
  [ -f "$PKI_DIR/$candidate-cert.pem" ] && return 0
  return 1
}

next_free_name() {
  local n=1 candidate
  while :; do
    candidate=$(printf "NMU_%02d" "$n")
    name_taken "$candidate" || { echo "$candidate"; return; }
    n=$((n + 1))
  done
}

write_config_h() {
  local dest="$1" ssid="$2" pass="$3"
  cat >"$dest" <<CONFIG
#ifndef CONFIG_H
#define CONFIG_H
#define OMEGA_WIFI_SSID   "$ssid"
#define OMEGA_WIFI_PASS   "$pass"
#define OMEGA_SERVER_IP   ""
#define OMEGA_SERVER_PORT $SERVER_PORT
#define OMEGA_HANDSHAKE_TIMEOUT_MS $HANDSHAKE_TIMEOUT_MS
#define OMEGA_ACK_TIMEOUT_MS       $ACK_TIMEOUT_MS
#endif
CONFIG
}

build_unit() {
  # Builds one unit into $PACK/<unit>/. Echoes nothing; returns non-zero on
  # failure so the caller can record it and carry on with the rest of the batch.
  local unit="$1" ssid="$2" pass="$3" pack="$4"
  local work="$WORK_ROOT/$unit"
  local sketch_dir="$work/$SKETCH"      # folder name MUST equal the sketch name

  rm -rf "$work"
  mkdir -p "$sketch_dir" || return 1
  cp -r "$SRC"/*.ino "$SRC"/*.h "$SRC"/*.cpp "$SRC"/partitions.csv "$sketch_dir"/ 2>/dev/null

  # Re-flashing an existing unit REUSES its certificate by default, so a
  # firmware update never changes identity and the server keeps accepting it.
  # FORCE_REISSUE=1 mints a new one instead, which is a new identity and so
  # must also clear the server's pinned fingerprint or the unit is refused as
  # a clone of itself. See FINDINGS #56.
  if [ -f "$PKI_DIR/$unit-cert.pem" ] && [ -z "${FORCE_REISSUE:-}" ]; then
    say "$unit already has a certificate - reusing it (identity unchanged)"
  else
    "$PYTHON" "$PKI_TOOL" device "$PKI_DIR" "$unit" --ca-key-dir "$CA_KEY_DIR" \
      ${FORCE_REISSUE:+--force} >>"$LOG" 2>&1 || return 2
    if [ -n "${FORCE_REISSUE:-}" ] && [ -f "$IDENTITY_TOOL" ]; then
      "$PYTHON" "$IDENTITY_TOOL" forget "$IDENTITY_DB" "$unit" >>"$LOG" 2>&1
    fi
  fi
  "$PYTHON" "$PKI_TOOL" arduino "$PKI_DIR" "$unit" "$sketch_dir/omega_certs.h" \
    >>"$LOG" 2>&1 || return 2
  [ -s "$sketch_dir/omega_certs.h" ] || return 2

  write_config_h "$sketch_dir/config.h" "$ssid" "$pass"

  "$ARDUINO_CLI" compile --fqbn "$FQBN" \
    --output-dir "$sketch_dir/build" "$sketch_dir" >>"$LOG" 2>&1 || return 3

  local unit_out="$pack/$unit"
  mkdir -p "$unit_out"
  for artefact in "$SKETCH.ino.bin" "$SKETCH.ino.bootloader.bin" "$SKETCH.ino.partitions.bin"; do
    cp "$sketch_dir/build/$artefact" "$unit_out/" || return 4
  done
  cp "$TEMPLATE/boot_app0.bin" "$unit_out/" || return 4
  return 0
}

write_flash_bat() {
  local dest="$1"
  cat >"$dest" <<'BAT'
@echo off
setlocal enabledelayedexpansion
title Omega NMU flasher
cd /d "%~dp0"

echo ==================================================
echo   Omega NMU flasher
echo ==================================================
echo.

set COUNT=0
for /d %%U in (NMU_*) do (
  set /a COUNT+=1
  set "UNIT_!COUNT!=%%U"
  echo   [!COUNT!] %%U
)
if !COUNT!==0 (
  echo No units found in this folder. Did you unzip it first?
  pause & exit /b 1
)
echo.
set "PICK="
set /p "PICK=Type the number of the unit to flash: "
if not defined PICK exit /b 0
echo !PICK!| findstr /r "^[0-9][0-9]*$" >nul
if errorlevel 1 (
  echo Type just the number, for example 1
  pause & exit /b 1
)
set "UNIT=!UNIT_%PICK%!"
if not defined UNIT (
  echo That is not one of the numbers above.
  pause & exit /b 1
)
if not exist "!UNIT!\" (
  echo That is not one of the numbers above.
  pause & exit /b 1
)

echo.
echo   Unit: !UNIT!
echo.
echo   Plug the ESP32 board into this PC with a USB cable.
echo   Unplug any OTHER ESP32 board first.
echo.
pause
echo.

esptool.exe --chip esp32s3 --baud 460800 ^
  --before default-reset --after watchdog-reset write-flash -z ^
  0x0     "!UNIT!\stage3_beta_2_dtls.ino.bootloader.bin" ^
  0x8000  "!UNIT!\stage3_beta_2_dtls.ino.partitions.bin" ^
  0xe000  "!UNIT!\boot_app0.bin" ^
  0x20000 "!UNIT!\stage3_beta_2_dtls.ino.bin"

if errorlevel 1 (
  echo.
  echo   FLASHING FAILED.
  echo.
  echo   Try this, leaving the cable plugged in:
  echo     1. hold down BOOT
  echo     2. press RESET once and let go of RESET
  echo     3. let go of BOOT
  echo   Then run this again and pick the same number.
  pause & exit /b 1
)

echo.
echo ==================================================
echo   !UNIT! is ready. You can unplug it.
echo   It joins the server by itself within about a minute.
echo ==================================================
pause
BAT
  # A .bat with bare LF line endings is mis-parsed by cmd.exe; the file is
  # written here on Linux, so it must be converted before it ships.
  sed -i 's/\r\?$/\r/' "$dest"
}

write_readme() {
  local dest="$1" ssid="$2"; shift 2
  {
    echo "OMEGA NMU FLASH PACK"
    echo "===================="
    echo
    echo "Built $(date '+%Y-%m-%d %H:%M') by nmu_factory.sh $FACTORY_VERSION"
    echo "WiFi network these units will join: $ssid"
    echo
    echo "Units in this pack:"
    for unit in "$@"; do echo "  - $unit"; done
    echo
    echo "WHAT TO DO"
    echo "----------"
    echo "1. Copy this whole folder onto a Windows PC and unzip it."
    echo "2. Double-click FLASH.bat"
    echo "3. Type the number of the unit you want to make."
    echo "4. Plug ONE ESP32 board in with a USB cable when it asks."
    echo "5. Wait for the green 'is ready' message. Unplug the board."
    echo
    echo "Repeat from step 2 for each board. Use a different number each time -"
    echo "two boards must never be given the same unit name."
    echo
    echo "IF IT SAYS 'No ESP32 board detected'"
    echo "------------------------------------"
    echo "Leave the cable plugged in and do this on the board:"
    echo "  1. hold down BOOT"
    echo "  2. press RESET once and let go of RESET"
    echo "  3. let go of BOOT"
    echo "Then run FLASH.bat again."
    echo
    echo "THIS PACK CONTAINS PRIVATE KEYS"
    echo "-------------------------------"
    echo "Each unit's firmware carries that unit's own identity key. Delete"
    echo "this folder once the boards are made, and do not e-mail it."
  } >"$dest"
}

# ---------------------------------------------------------------- main

echo
echo "=================================================="
echo "  Omega NMU factory  ($FACTORY_VERSION)"
echo "=================================================="
echo

preflight

if [ "$#" -ge 1 ] && [ -f "$1" ]; then
  # Parsed, never sourced. A real WiFi password contains characters the shell
  # would act on - a password like "Abcde$2026!" expands $2 and aborts under
  # set -u. Sourcing a config file also lets it run arbitrary commands.
  WIFI_SSID=""; WIFI_PASS=""; UNIT_NAMES=""
  while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in
      \#*|"") continue ;;
    esac
    key=${line%%=*}
    value=${line#*=}
    value=${value#\"}; value=${value%\"}
    case "$key" in
      WIFI_SSID)  WIFI_SSID=$value ;;
      WIFI_PASS)  WIFI_PASS=$value ;;
      UNIT_NAMES) UNIT_NAMES=$value ;;
    esac
  done < "$1"
  [ -n "$WIFI_SSID" ] || fail "batch file sets no WIFI_SSID"
else
  read -r -p "WiFi network name (SSID): " WIFI_SSID
  read -r -s -p "WiFi password: " WIFI_PASS; echo
  read -r -p "How many units to make: " UNIT_COUNT
  case "$UNIT_COUNT" in ''|*[!0-9]*) fail "that is not a number" ;; esac
  [ "$UNIT_COUNT" -ge 1 ] || fail "nothing to do"
  echo
  echo "Leave a name blank to accept the suggested one."
  CHOSEN=""
  for _ in $(seq 1 "$UNIT_COUNT"); do
    suggestion=$(next_free_name)
    read -r -p "  name [$suggestion]: " chosen
    chosen="${chosen:-$suggestion}"
    if name_taken "$chosen"; then
      say "$chosen is already in use - a second board with this name would be"
      say "rejected by the server as a clone. Skipped."
      continue
    fi
    CHOSEN="$CHOSEN $chosen"
  done
  UNIT_NAMES="$CHOSEN"
fi

[ -n "${UNIT_NAMES// /}" ] || fail "no unit names given"

STAMP=$(date '+%Y%m%d_%H%M')
PACK="$OUT_ROOT/NMU_flash_pack_$STAMP"
WORK_ROOT="$OUT_ROOT/.work_$STAMP"
LOG="$OUT_ROOT/factory_$STAMP.log"
mkdir -p "$PACK" "$WORK_ROOT" || fail "cannot write to $OUT_ROOT"
: >"$LOG"

echo
echo "Building. Each unit takes about a minute."
echo "Full compiler output: $LOG"
echo

BUILT=""
FAILED=""
for unit in $UNIT_NAMES; do
  printf "  %-10s " "$unit"
  build_unit "$unit" "$WIFI_SSID" "$WIFI_PASS" "$PACK"
  case "$?" in
    0) echo "built";          BUILT="$BUILT $unit" ;;
    2) echo "CERTIFICATE FAILED"; FAILED="$FAILED $unit" ;;
    3) echo "COMPILE FAILED";     FAILED="$FAILED $unit" ;;
    *) echo "FAILED";             FAILED="$FAILED $unit" ;;
  esac
done

if [ -z "${BUILT// /}" ]; then
  rm -rf "$WORK_ROOT"
  fail "no unit built - see $LOG"
fi

cp "$TEMPLATE/esptool.exe" "$PACK/" || fail "could not stage esptool.exe"
write_flash_bat "$PACK/FLASH.bat"
# shellcheck disable=SC2086
write_readme "$PACK/README.txt" "$WIFI_SSID" $BUILT

ZIP="$OUT_ROOT/NMU_flash_pack_$STAMP.zip"
( cd "$OUT_ROOT" && zip -qr "$ZIP" "$(basename "$PACK")" ) || fail "could not create the zip"
rm -rf "$WORK_ROOT"
chmod 600 "$ZIP"

echo
echo "=================================================="
echo "  Done"
echo "=================================================="
for unit in $BUILT;  do echo "  built   $unit"; done
for unit in $FAILED; do echo "  FAILED  $unit"; done
echo
echo "  Flash pack: $ZIP"
echo "  Copy it to a Windows PC, unzip, double-click FLASH.bat."
echo
