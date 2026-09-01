#!/usr/bin/env bash
# Omega hub installer - bare Ubuntu to a complete working system.
#
# WHAT THIS IS. One machine that is the server, the certificate authority,
# the unit factory and the operator console all at once. That is a deliberate
# PROTOTYPE choice, not an oversight, and its cost is stated plainly in
# "SECURITY POSTURE" below and in the memo. It exists because a single hub
# has far fewer moving parts to fail during a field test than four machines
# that must find each other.
#
# WHAT IT IS NOT. It is not a production deployment. A production install
# keeps the certificate authority off this machine entirely - see the
# pendrive tool in provisioning/pendrive, which is the same fleet with the
# signing key carried in someone's pocket instead.
#
# SECURITY POSTURE, stated up front so nobody has to discover it:
#   * The CA private key lives on this machine. Anyone who gains root here
#     can mint a certificate for a device that does not exist, and the fleet
#     will accept it. That is the single biggest trade this file makes.
#     Mitigation path, already built: provisioning/pendrive keeps the same
#     key on removable media, so units can only be added while a person is
#     physically present.
#   * The operator certificate is generated here too, so root here can also
#     read the dashboard and drive the MCP tools.
#   * What this does NOT weaken: the protocol. Devices still authenticate the
#     server and each other with mutual DTLS 1.3, records are still encrypted
#     and replay-protected, and a revoked unit is still refused. Compromising
#     the hub lets an attacker JOIN the fleet; it does not let them read or
#     forge traffic between units that already exist without doing so.
#
# Every phase is idempotent - run it again as often as you like.
#
#   ./install_hub.sh              run every phase in order
#   ./install_hub.sh --check      report only, change nothing
#   ./install_hub.sh --only pki   run one phase

set -u
HUB_VERSION="2026-08-28a-clone-amu-card"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd || echo "$SCRIPT_DIR")"

OMEGA="${OMEGA_HOME:-$HOME/omega_brick4}"
PKI_DIR="$OMEGA/pki_provisioning"
BIN_DIR="$HOME/bin"
ESP32_INDEX="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

CHECK_ONLY=0
ONLY_PHASE=""

log()   { echo "[hub] $*"; }
ok()    { printf '  \033[1;32mOK\033[0m      %s\n' "$*"; }
miss()  { printf '  \033[1;31mMISSING\033[0m %s\n' "$*"; }
head2() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
die()   { printf '\033[1;31m[hub][ERROR]\033[0m %s\n' "$*" >&2; exit 1; }

# Where each tool lives in the bundle. Kept in one place because this script
# is run from a repo checkout AND from a flattened pendrive copy.
find_in_bundle() {
  local name="$1" candidate
  for candidate in "$BUNDLE/brick4_dtls13/$name" "$BUNDLE/$name" "$SCRIPT_DIR/$name"; do
    [ -e "$candidate" ] && { printf '%s' "$candidate"; return 0; }
  done
  return 1
}

phase_wanted() { [ -z "$ONLY_PHASE" ] || [ "$ONLY_PHASE" = "$1" ]; }

# ---------------------------------------------------------------- 1 system

APT_PACKAGES="python3 python3-venv python3-pip nginx avahi-daemon avahi-utils sqlite3 curl git openssl"

# A package can be present without dpkg knowing: curl on this very server is
# a snap, so a dpkg-only check reported it MISSING and would have had the
# operator reinstall something they already had. If the command answers, the
# requirement is met, whatever installed it.
have_package() {
  # curl is checked by PACKAGE, not by command, and deliberately. A snap curl
  # answers `command -v` while being sandboxed away from the home directory,
  # and it has now broken three separate installs that way - arduino-cli and
  # Claude Code among them - each time with an error blaming the download
  # rather than the sandbox. If the deb is absent, install it even though
  # something called curl already answers.
  if [ "$1" = "curl" ]; then
    dpkg -s curl >/dev/null 2>&1
    return $?
  fi
  command -v "$1" >/dev/null 2>&1 && return 0
  dpkg -s "$1" >/dev/null 2>&1
}

phase_system() {
  head2 "1. system packages"
  local missing=""
  for pkg in $APT_PACKAGES; do
    have_package "$pkg" || missing="$missing $pkg"
  done
  if [ -z "$missing" ]; then ok "all base packages present"; return 0; fi
  if [ "$CHECK_ONLY" = "1" ]; then miss "apt packages:$missing"; return 0; fi
  log "installing:$missing"
  sudo apt-get update -qq || die "apt-get update failed"
  sudo apt-get install -y $missing || die "apt-get install failed"
  ok "installed:$missing"
}

# ------------------------------------------------------------------- 2 pki

phase_pki() {
  head2 "2. certificate authority and certificates"
  local pki_tool
  pki_tool="$(find_in_bundle provisioning/omega_pki.py)" || die "omega_pki.py not found in the bundle"

  if [ -f "$PKI_DIR/ca-key.pem" ] && [ -f "$PKI_DIR/omega-server-cert.pem" ] && [ -f "$PKI_DIR/operator-cert.pem" ]; then
    ok "CA, server and operator certificates already present"
    return 0
  fi
  if [ "$CHECK_ONLY" = "1" ]; then miss "PKI in $PKI_DIR"; return 0; fi

  mkdir -p "$PKI_DIR"
  chmod 700 "$PKI_DIR"

  # A CA carried in from somewhere else is ADOPTED, never replaced. This is
  # the whole reason the fleet can be rebuilt after the server dies: the
  # devices trust a CA, not a machine, so a hub that starts life with the
  # OLD ca-key.pem keeps every existing unit working. A hub that generates a
  # fresh CA is a fresh fleet - every device must be re-issued.
  #
  # Looked for beside this bundle because that is where a pendrive puts it.
  local carried
  for carried in "$BUNDLE/ca" "$BUNDLE/brick4_dtls13/provisioning/pendrive/ca" \
                 "$SCRIPT_DIR/../../provisioning/pendrive/ca"; do
    if [ -f "$carried/ca-key.pem" ] && [ -f "$carried/ca-cert.pem" ]; then
      cp "$carried/ca-key.pem" "$carried/ca-cert.pem" "$PKI_DIR/"
      chmod 600 "$PKI_DIR/ca-key.pem"
      ok "adopted the CA carried on the drive - existing units keep working"
      break
    fi
  done

  # init ADOPTS an existing CA rather than replacing it, so dropping an
  # offline ca-key.pem/ca-cert.pem into $PKI_DIR before running this is the
  # whole migration path from another machine.
  python3 "$pki_tool" init "$PKI_DIR" || die "PKI init failed"
  ok "PKI ready in $PKI_DIR"
}

# ---------------------------------------------------------------- 3 server

phase_server() {
  head2 "3. server (listener, dashboard, gateway, cron)"
  local installer
  installer="$(find_in_bundle deploy/server/install_server.sh)" || die "install_server.sh not found in the bundle"

  if [ "$CHECK_ONLY" = "1" ]; then
    for unit in omega-listener omega-web nginx avahi-daemon; do
      systemctl is-active --quiet "$unit" && ok "$unit active" || miss "$unit not active"
    done
    return 0
  fi

  # A server that is already up is LEFT ALONE. Re-running the installer over
  # a live fleet rewrites service files and restarts the listener, which is
  # never what someone wants when they plugged the drive in to add a unit.
  # Reinstalling deliberately is still one flag away.
  if [ "${OMEGA_FORCE_SERVER:-0}" != "1" ] \
     && systemctl is-active --quiet omega-listener \
     && systemctl is-active --quiet omega-web \
     && [ -d "$OMEGA" ]; then
    ok "server already running - left untouched"
    log "to reinstall anyway: OMEGA_FORCE_SERVER=1 $0 --only server"
    return 0
  fi

  OMEGA_HOME="$OMEGA" OMEGA_PKI_SRC="$PKI_DIR" bash "$installer" || die "install_server.sh failed"
  ok "server installed"
}

# ------------------------------------------------------------- 4 toolchain

# Libraries the NMU firmware #includes. Pinned, because these are compiled
# INTO a device that then has to be reflashed by hand if a newer version
# changes an API - the version that was tested is the version that ships.
# The graphical IDE's Library Manager and `arduino-cli lib install` are the
# same mechanism writing to the same folder; the GUI is only a front end,
# so nothing here needs a desktop.
ARDUINO_LIBS="wolfssl@5.8.4 ArduinoJson@7.4.3"
ARDUINO_CLI_VERSION="1.5.1"

# Arduino publish a one-line installer meant to be piped into a shell. This
# does not use it, for two reasons found the hard way on 2026-08-25.
#
# It broke: that script downloads with curl into /tmp and then untars from
# /tmp. Where curl is installed as a SNAP - as it is on this very server -
# the download lands inside the snap's private /tmp and tar, which is not
# confined, looks in the real one and finds nothing. The failure reads
# "Cannot open: No such file or directory" and blames tar, and it only
# happens on a machine where nobody installed the curl deb, which is exactly
# a bare machine.
#
# And piping a remote script into a shell is a poor habit for a project
# whose subject is supply-chain-adjacent trust. Downloading a pinned
# artefact and extracting one known file is both more reliable and easier to
# defend.
install_arduino_cli() {
  local arch url
  case "$(uname -m)" in
    x86_64)  arch="Linux_64bit" ;;
    aarch64) arch="Linux_ARM64" ;;
    armv7l)  arch="Linux_ARMv7" ;;
    *) die "unsupported architecture for arduino-cli: $(uname -m)" ;;
  esac
  url="https://downloads.arduino.cc/arduino-cli/arduino-cli_${ARDUINO_CLI_VERSION}_${arch}.tar.gz"
  log "downloading arduino-cli $ARDUINO_CLI_VERSION ($arch)"
  python3 - "$url" "$BIN_DIR" <<'PY'
import io
import os
import sys
import tarfile
import urllib.request

url, bindir = sys.argv[1], sys.argv[2]
with urllib.request.urlopen(url, timeout=180) as response:
    payload = response.read()
os.makedirs(bindir, exist_ok=True)
with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as archive:
    archive.extract(archive.getmember("arduino-cli"), bindir)
target = os.path.join(bindir, "arduino-cli")
os.chmod(target, 0o755)
print("installed %s (%d bytes downloaded)" % (target, len(payload)))
PY
}

phase_toolchain() {
  head2 "4. ESP32 toolchain (needed only to MAKE new NMU units)"
  local cli="$BIN_DIR/arduino-cli"
  local have_core=0 missing_libs=""
  if [ -x "$cli" ]; then
    "$cli" core list 2>/dev/null | grep -q "^esp32:esp32" && have_core=1
    local installed
    installed="$("$cli" lib list 2>/dev/null)"
    for spec in $ARDUINO_LIBS; do
      case "$installed" in
        *"${spec%@*}"*) ;;
        *) missing_libs="$missing_libs $spec" ;;
      esac
    done
  else
    missing_libs="$ARDUINO_LIBS"
  fi

  if [ -x "$cli" ] && [ "$have_core" = "1" ] && [ -z "$missing_libs" ]; then
    ok "arduino-cli, the ESP32 core and the firmware libraries are installed"
    return 0
  fi
  if [ "$CHECK_ONLY" = "1" ]; then
    [ -x "$cli" ] || miss "arduino-cli ($cli)"
    [ "$have_core" = "1" ] || miss "esp32:esp32 core"
    [ -z "$missing_libs" ] || miss "firmware libraries:$missing_libs"
    return 0
  fi

  mkdir -p "$BIN_DIR"
  if [ ! -x "$cli" ]; then
    install_arduino_cli || die "arduino-cli install failed"
  fi
  # The ESP32 core pulls a ~600 MB compiler toolchain from GitHub's release
  # assets. arduino-cli's default network timeout is far too short for that
  # on a domestic connection, and the failure is opaque - a "context deadline
  # exceeded" on a HEAD request, printed with the whole signed URL, which
  # reads like a permissions problem rather than a slow download.
  "$cli" config set network.connection_timeout 600s >/dev/null 2>&1 || true

  if [ "$have_core" != "1" ]; then
    log "installing the ESP32 core (about 600 MB, one time - this is slow)"
    "$cli" core update-index --additional-urls "$ESP32_INDEX" >/dev/null || die "core index update failed"
    # One retry: a large download over a home connection fails often enough
    # that giving up after a single attempt would send an operator hunting
    # for a fault that is not there.
    if ! "$cli" core install esp32:esp32 --additional-urls "$ESP32_INDEX"; then
      log "core download failed - retrying once"
      "$cli" core install esp32:esp32 --additional-urls "$ESP32_INDEX" \
        || die "esp32 core install failed twice - check the network and run again; already-downloaded parts are kept"
    fi
  fi

  if [ -n "$missing_libs" ]; then
    log "installing firmware libraries:$missing_libs"
    "$cli" lib update-index >/dev/null 2>&1
    for spec in $missing_libs; do
      "$cli" lib install "$spec" || die "could not install Arduino library $spec"
    done
  fi
  ok "toolchain ready"
}

# ----------------------------------------------------------------- 5 tools

phase_tools() {
  head2 "5. unit factory"
  local maker firmware
  maker="$(find_in_bundle provisioning/pendrive/make-units)" || die "make-units not found in the bundle"
  firmware="$(find_in_bundle provisioning/pendrive/firmware)" || die "firmware source not found in the bundle"

  if [ "$CHECK_ONLY" = "1" ]; then
    [ -x "$BIN_DIR/omega-make-units" ] && ok "omega-make-units installed" || miss "omega-make-units"
    return 0
  fi

  mkdir -p "$BIN_DIR" "$OMEGA/factory"
  rm -rf "$OMEGA/factory/firmware"
  cp -r "$firmware" "$OMEGA/factory/firmware"

  # A one-line launcher rather than a copy of the tool: the tool stays the
  # single source of truth, and the launcher supplies the hub's paths.
  {
    echo "#!/usr/bin/env bash"
    echo "# Generated by install_hub.sh $HUB_VERSION - do not edit; re-run the hub."
    echo "export OMEGA_HOME=\"$OMEGA\""
    echo "export OMEGA_FIRMWARE_SRC=\"$OMEGA/factory/firmware\""
    echo "exec \"$maker\" \"\$@\""
  } > "$BIN_DIR/omega-make-units"
  chmod +x "$BIN_DIR/omega-make-units"

  # The AMU card tools. Each needs root (they mount or write a card), so the
  # launcher does not try to hide that - they are invoked with sudo.
  #
  # There are TWO ways to make an air unit and both must be installed, because
  # the manual offers both and an operator has whichever starting point they
  # have:
  #   clone-amu-card    blank card + the golden image. Writes the image, then
  #                     strips the donor's identity and issues a fresh one.
  #   prepare-amu-card  a card already written by Raspberry Pi Imager. Writes
  #                     only the software and identity; the Pi installs itself
  #                     on first boot.
  # clone-amu-card was missing from this list until 2026-08-28, so a fresh
  # install shipped without the ONLY tool that writes the golden image - and
  # the manual named prepare-amu-card for that job, which refuses a blank card
  # because it finds no cloud-init user-data on it. Invisible on a machine
  # that already had both.
  #
  # The other two: make-amu-bundle issues a unit its identity and packs its
  # software; patch-amu-card updates a deployed card in place.
  local amu_maker
  for amu_tool in make-amu-bundle clone-amu-card prepare-amu-card patch-amu-card read-amu-card; do
    # Two locations, because these tools grew up in two places: the card
    # tools beside the PKI, and read-amu-card with the pendrive kit it was
    # written for. A fresh install missed read-amu-card entirely that way -
    # and that is the tool that MAKES the golden image every AMU is cloned
    # from, so the gap only shows up on the first machine that has no image.
    if amu_maker="$(find_in_bundle provisioning/$amu_tool)" ||
       amu_maker="$(find_in_bundle provisioning/pendrive/$amu_tool)"; then
      {
        echo "#!/usr/bin/env bash"
        echo "# Generated by install_hub.sh $HUB_VERSION - do not edit; re-run the hub."
        echo "export OMEGA_HOME=\"$OMEGA\""
        echo "export OMEGA_PKI_DIR=\"$PKI_DIR\""
        echo "export OMEGA_PKI_TOOL=\"$OMEGA/omega_pki.py\""
        echo "exec \"$amu_maker\" \"\$@\""
      } > "$BIN_DIR/omega-$amu_tool"
      chmod +x "$BIN_DIR/omega-$amu_tool"
      ok "omega-$amu_tool installed"
    fi
  done

  # Every tool here writes to a card or a certificate store, so every tool
  # here is run with sudo - and sudo deliberately ignores the operator's PATH
  # in favour of a fixed system list. Installed only in ~/bin they are found
  # when typed and NOT found when sudoed, which reads as "command not found"
  # on a tool the installer just reported as installed. Linking them into
  # /usr/local/bin, which is on that fixed list, is what makes the short name
  # work the way the manual says it does.
  if [ -d /usr/local/bin ] && [ -w /usr/local/bin ]; then
    ln -sf "$BIN_DIR"/omega-* /usr/local/bin/ 2>/dev/null       && ok "tools linked into /usr/local/bin (usable under sudo)"
  else
    miss "run once as root to finish: sudo ln -sf $BIN_DIR/omega-* /usr/local/bin/"
  fi

  # make-units reads the CA key from ITS OWN directory, so on the hub that is
  # a copy of the resident PKI.
  mkdir -p "$(dirname "$maker")/ca"
  if [ -f "$PKI_DIR/ca-key.pem" ]; then
    cp "$PKI_DIR/ca-key.pem" "$(dirname "$maker")/ca/ca-key.pem"
    chmod 600 "$(dirname "$maker")/ca/ca-key.pem"
  fi
  ok "omega-make-units installed"
}

# -------------------------------------------------------------- 6 operator

phase_operator() {
  head2 "6. operator pass (dashboard + MCP)"
  local pki_tool out="$OMEGA/operator"
  pki_tool="$(find_in_bundle provisioning/omega_pki.py)" || die "omega_pki.py missing"

  if [ "$CHECK_ONLY" = "1" ]; then
    [ -f "$out/operator.p12" ] && ok "operator.p12 present" || miss "operator.p12"
    command -v claude >/dev/null 2>&1 && ok "claude code CLI present" || miss "claude code CLI (optional)"
    return 0
  fi

  mkdir -p "$out"
  chmod 700 "$out"
  cp "$PKI_DIR/operator-cert.pem" "$PKI_DIR/operator-key.pem" "$PKI_DIR/ca-cert.pem" "$out/" 2>/dev/null
  if python3 "$pki_tool" export-operator "$PKI_DIR" "${OMEGA_P12_PASSWORD:-omega}" >/dev/null 2>&1; then
    cp "$PKI_DIR/operator.p12" "$out/operator.p12"
    chmod 600 "$out/operator.p12"
  fi
  ok "operator files in $out"

  {
    echo "OPERATOR PASS"
    echo "============="
    echo
    echo "operator.p12  - your pass to the dashboard. Import it on the machine you"
    echo "                browse from, as a personal certificate."
    echo "                Password: whatever OMEGA_P12_PASSWORD was set to"
    echo "                (default \"omega\" - change it before this leaves the building)."
    echo
    echo "Then open:    https://smartageing.local"
    echo
    echo "Without this certificate the dashboard refuses the connection outright."
    echo "That is the gate working, not a fault."
    echo
    echo "TALKING TO THE FLEET IN PLAIN LANGUAGE (MCP)"
    echo "--------------------------------------------"
    echo "Use CLAUDE CODE, the terminal tool. It runs on this Ubuntu machine."
    echo
    echo "Claude DESKTOP, the windowed app, is macOS and Windows only - so on this"
    echo "hub the console is Claude Code, and the configuration differs from the"
    echo "claude_desktop_config.json described in deploy/MCP_LIVE.md."
    echo
    echo "  1. Install (once):"
    echo "       curl -fsSL https://claude.ai/install.sh | bash"
    echo
    echo "  2. Register the fleet tools (once):"
    echo "       claude mcp add omega -- $OMEGA/venv/bin/python $OMEGA/mcp_server.py"
    echo
    echo "  3. Ask for things:"
    echo "       claude"
    echo "       > which units have reported in the last hour?"
    echo
    echo "The tools are fixed functions. Claude chooses WHICH one to call; it never"
    echo "invents the command that reaches a device."
  } > "$out/README_OPERATOR.txt"
  ok "wrote $out/README_OPERATOR.txt"

  # Firefox keeps its OWN certificate store, per profile, and does not read
  # the system one - so the pass has to be imported into each profile or the
  # dashboard answers "No required SSL certificate was sent", which reads as
  # a fault and is in fact the mutual-authentication gate working.
  local cert_tool
  if cert_tool="$(find_in_bundle provisioning/install-operator-cert)"; then
    cp "$cert_tool" "$BIN_DIR/omega-install-operator-cert"
    chmod +x "$BIN_DIR/omega-install-operator-cert"
    ok "omega-install-operator-cert installed (run it to open the dashboard here)"
  fi
}

# ------------------------------------------------------------- 7 console

# Two things that turn a working server into one somebody can actually USE
# without being told anything: the plain-language console, and a browser
# that opens on the dashboard by itself.
#
# Both are deliberately non-fatal. A hub with no internet, or no desktop,
# is still a complete and correct fleet server - it just has to be driven
# from the terminal. Failing the whole install over a convenience would be
# the wrong trade.

phase_console() {
  head2 "7. operator console (Claude Code + dashboard on login)"

  if [ "$CHECK_ONLY" = "1" ]; then
    command -v claude >/dev/null 2>&1 && ok "claude code CLI present" || miss "claude code CLI"
    [ -f "$HOME/.config/autostart/omega-dashboard.desktop" ] \
      && ok "dashboard opens on login" || miss "dashboard autostart"
    return 0
  fi

  if command -v claude >/dev/null 2>&1; then
    ok "claude code CLI already installed"
  else
    log "installing Claude Code (needs internet; skipped cleanly if offline)"
    if curl -fsSL --max-time 120 https://claude.ai/install.sh | bash >/dev/null 2>&1; then
      ok "claude code CLI installed"
    else
      miss "claude code CLI - no internet, or the installer changed."
      log "  install later with: curl -fsSL https://claude.ai/install.sh | bash"
    fi
  fi

  # The installer drops claude into ~/.local/bin, which THIS shell does not
  # have on its PATH yet - it was resolved before the install ran. Without
  # this line the registration below is skipped in silence on exactly the
  # machine that just installed the tool, and shows up much later as "claude
  # cannot see the fleet".
  PATH="$HOME/.local/bin:$PATH"

  # Register the fleet tools. Re-adding the same name REPLACES it, which is
  # what makes this safe to re-run - and is also the repair: a registration
  # made by hand can point at a venv that has since moved or been deleted,
  # and it keeps answering until the day that folder goes. Pointing it at
  # this install's own venv every time is what stops that.
  if command -v claude >/dev/null 2>&1 && [ -x "$OMEGA/venv/bin/python" ]; then
    claude mcp remove omega >/dev/null 2>&1 || true
    claude mcp add omega -- "$OMEGA/venv/bin/python" "$OMEGA/mcp_server.py" >/dev/null 2>&1 \
      && ok "fleet tools registered with Claude Code (ask it about the units)" \
      || miss "could not register the MCP tools - run the command in README_OPERATOR.txt"
  elif command -v claude >/dev/null 2>&1; then
    # Silence here would be the worst outcome: the console installs, the
    # operator asks it about the fleet, and it answers that it has no tools.
    # Say which file is missing and why, so the cause is the message rather
    # than an afternoon.
    miss "$OMEGA/venv/bin/python is not there, so the fleet tools were NOT registered"
    log "  this machine's server was installed before the current layout and"
    log "  runs from a venv elsewhere - check: systemctl cat omega-listener"
    log "  a fresh install puts it at $OMEGA/venv and this registers itself"
  fi

  # The dashboard refuses anyone without the operator certificate, and Firefox
  # keeps its certificate store per PROFILE rather than system-wide. So the
  # autostart entry runs the importer first and opens the page second: opening
  # it in the other order shows a refusal that looks like a broken server and
  # is in fact the gate doing its job.
  if [ -d /usr/share/xsessions ] || [ -n "${XDG_CURRENT_DESKTOP:-}" ]; then
    mkdir -p "$HOME/.config/autostart"
    cat > "$HOME/.config/autostart/omega-dashboard.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=Omega dashboard
Comment=Import the operator pass, then open the fleet dashboard
Exec=bash -c '$BIN_DIR/omega-install-operator-cert >/dev/null 2>&1; sleep 5; xdg-open https://smartageing.local'
X-GNOME-Autostart-enabled=true
Terminal=false
DESKTOP
    ok "dashboard will open by itself at next login"

    # Stop the browser restoring its own idea of what should be on screen.
    #
    # After a hard power cut this machine came back with TWO dashboard tabs:
    # one restored from the previous session - still rendering the OLD
    # dashboard it had cached from before the last deploy - and one opened
    # fresh by the entry above. An operator glancing at the wall has no way
    # to tell which of the two is current, which is worse than showing
    # nothing.
    #
    # The cause is that a power cut is indistinguishable from a crash:
    # Firefox sees an unclean shutdown and helpfully restores the session.
    # Correct for a person's browser, wrong for an unattended display where
    # the autostart entry must be the ONLY thing deciding what is shown.
    #
    # user.js is re-read at every startup and overrides prefs.js, so the
    # browser cannot quietly undo this when it writes its own state on exit.
    # Applied to every profile found, because which one is "default" can
    # change and a wrong guess fixes nothing.
    local ff_profiles ff_written=0
    for ff_profiles in "$HOME"/snap/firefox/common/.mozilla/firefox/*.default*                        "$HOME"/.mozilla/firefox/*.default*; do
      [ -d "$ff_profiles" ] || continue
      cat > "$ff_profiles/user.js" <<'FFPREFS'
// Installed by install_hub.sh - unattended dashboard display.
// Start blank; the autostart entry supplies the URL.
user_pref("browser.startup.page", 0);
// A power cut is an unclean shutdown - do not restore the old session.
user_pref("browser.sessionstore.resume_from_crash", false);
// No restore prompt either; nobody is there to answer it.
user_pref("browser.sessionstore.max_resumed_crashes", 0);
// No default-browser nag over the dashboard.
user_pref("browser.shell.checkDefaultBrowser", false);
FFPREFS
      ff_written=$((ff_written + 1))
    done
    if [ "$ff_written" -gt 0 ]; then
      ok "browser set to show only the dashboard ($ff_written profile(s))"
    else
      ok "no Firefox profile yet - run Firefox once, then re-run this phase"
    fi
  else
    ok "no desktop on this machine - dashboard stays at https://smartageing.local"
  fi
}

# ---------------------------------------------------------------- 8 doctor

phase_doctor() {
  head2 "8. health check"
  local bad=0
  for unit in omega-listener omega-web nginx avahi-daemon; do
    if systemctl is-active --quiet "$unit"; then ok "$unit active"; else miss "$unit NOT active"; bad=1; fi
  done
  if [ -f "$PKI_DIR/ca-key.pem" ]; then ok "CA key present (see SECURITY POSTURE)"; else miss "CA key"; bad=1; fi
  if [ -x "$BIN_DIR/omega-make-units" ]; then ok "unit factory ready"; else miss "unit factory"; bad=1; fi
  # -r is what RESOLVES a service; plain -p only lists that one exists, and
  # its lines start with "+" not "=", so the old grep could never match and
  # reported a healthy server as unadvertised.
  if timeout 8 avahi-browse -rpt _omega._udp 2>/dev/null | grep -q "^="; then
    ok "server advertising itself on the network"
  else
    miss "no _omega._udp advertisement seen"
    bad=1
  fi
  # Anything reported MISSING counts. A summary that says "healthy" with
  # failures listed above it is worse than no summary at all.
  if [ "$bad" = "0" ]; then
    printf '\n\033[1;32mHub is healthy.\033[0m\n'
  else
    printf '\n\033[1;31mHub has problems - see MISSING above.\033[0m\n'
  fi
  return 0
}

# ------------------------------------------------------------------- main

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    --only)  shift; ONLY_PHASE="${1:-}" ;;
    -h|--help) sed -n '1,35p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

printf '\033[1m Omega hub installer  (%s)\033[0m\n' "$HUB_VERSION"
[ "$CHECK_ONLY" = "1" ] && log "check only - nothing will be changed"
log "bundle: $BUNDLE"
log "target: $OMEGA"

for phase in system pki server toolchain tools operator console doctor; do
  phase_wanted "$phase" && "phase_$phase"
done
echo
