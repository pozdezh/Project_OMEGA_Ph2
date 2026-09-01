#!/usr/bin/env bash
# Build the AMU live agent's dedicated Python interpreter and virtualenv.
#
# WHY A SECOND INTERPRETER EXISTS ON THIS DEVICE
#
# The wolfSSL Python binding mismanages CPython thread state on its DTLS
# SERVER (accept) path. CPython 3.12 added strict thread-state checking that
# turns that latent bug into an immediate fatal error, so on Raspberry Pi OS
# trixie (Python 3.13) the inbound live-query listener dies after serving
# exactly one query:
#
#     Fatal Python error: _PyThreadState_Attach: non-NULL old thread state
#
# Proven by holding the code constant and changing only the environment: the
# same live_server.py answered 6/6 queries under Python 3.10 on the server
# and crashed after one under 3.13 on the AMU, with identical wolfSSL
# 5.9.2-0. The binding's PyPI metadata never claimed 3.12+ support, and
# 5.9.2.post0 is the newest release, so there is no upstream fix to take.
# Full account: FINDINGS #35.
#
# 3.11 is the newest release below the 3.12 threshold, so the device keeps a
# supported interpreter rather than being pinned to an EOL one.
#
# main.py (sampling + telemetry) KEEPS the system Python and is untouched.
# Only a separate PROCESS can run a different interpreter, which is why the
# agent is its own systemd unit rather than a thread.
#
# No root required: every build dependency ships with Raspberry Pi OS, and
# everything installs under $HOME.
#
# Runtime: roughly 20-30 minutes on a Pi 4. Safe to re-run; it skips work
# that is already done.

set -u

PREFIX="${OMEGA_LIVE_PY_PREFIX:-$HOME/py311}"
BUILDDIR="$HOME/.py311_build"
TARGET="${OMEGA_HOME:-$HOME/omega_amu}"
LIVE_VENV="$TARGET/venv311"
JOBS="$(nproc)"
VERSIONS="3.11.13 3.11.12 3.11.11 3.11.9"

log() { echo "[live-py] $*"; }
die() { echo "[live-py][ERROR $1] ${*:2}" >&2; exit "$1"; }

[ "$(id -u)" -ne 0 ] || die 1 "do not run as root - this installs under \$HOME"

build_interpreter() {
  if [ -x "$PREFIX/bin/python3.11" ]; then
    log "interpreter already present: $("$PREFIX/bin/python3.11" -V)"
    return 0
  fi

  for tool in gcc make curl tar; do
    command -v "$tool" >/dev/null 2>&1 || die 2 "$tool is required but missing"
  done

  mkdir -p "$BUILDDIR" || die 3 "cannot create $BUILDDIR"
  cd "$BUILDDIR" || die 3 "cannot enter $BUILDDIR"

  local tarball="" version=""
  for v in $VERSIONS; do
    log "fetching Python $v"
    if curl -fsSL --max-time 300 -o "Python-$v.tgz" \
        "https://www.python.org/ftp/python/$v/Python-$v.tgz"; then
      tarball="Python-$v.tgz"; version="$v"; break
    fi
    rm -f "Python-$v.tgz"
  done
  [ -n "$tarball" ] || die 4 "could not download any 3.11.x source"

  tar xzf "$tarball" || die 5 "extract failed"
  cd "Python-$version" || die 5 "source tree missing"

  # No --enable-optimizations: PGO roughly doubles build time for a speed
  # gain irrelevant to a listener that handles a few queries a day.
  log "configuring"
  ./configure --prefix="$PREFIX" --with-ensurepip=install \
    > /tmp/live_py_configure.log 2>&1 || {
      tail -20 /tmp/live_py_configure.log; die 6 "configure failed"; }

  log "compiling with $JOBS jobs (slow - 15-25 min on a Pi 4)"
  make -j"$JOBS" > /tmp/live_py_make.log 2>&1 || {
      tail -30 /tmp/live_py_make.log; die 7 "compile failed"; }

  log "installing to $PREFIX"
  make install > /tmp/live_py_install.log 2>&1 || {
      tail -20 /tmp/live_py_install.log; die 8 "install failed"; }

  # make install creates the binary BEFORE it finishes copying the standard
  # library, so "the binary exists" is NOT proof the install is complete.
  # Checking an actual stdlib import is (learned the hard way 2026-08-21).
  "$PREFIX/bin/python3.11" -c "import _posixsubprocess, ssl, ctypes, subprocess" \
    || die 9 "interpreter installed but its standard library is incomplete"
  log "built $("$PREFIX/bin/python3.11" -V)"
}

build_venv() {
  local py="$PREFIX/bin/python3.11"
  if [ ! -x "$LIVE_VENV/bin/python" ]; then
    "$py" -m venv "$LIVE_VENV" || die 10 "venv creation failed"
    log "created $LIVE_VENV"
  fi

  "$LIVE_VENV/bin/pip" install --quiet --upgrade pip setuptools wheel \
    || die 11 "pip bootstrap failed"

  if "$LIVE_VENV/bin/python" -c "import wolfssl" >/dev/null 2>&1; then
    log "wolfssl already installed in the agent venv"
    return 0
  fi

  # Let the binding build and BUNDLE its own wolfSSL. Do NOT point it at the
  # system library with USE_LOCAL_WOLFSSL: the copy in /usr/local on this
  # unit was compiled without ALPN, and linking against it produced an
  # _ffi.so that imported and then died on
  # "undefined symbol: wolfSSL_get0_next_proto_negotiated". The working
  # Python 3.13 venv on this same device bundles its own copy - verified, its
  # _ffi.so has no external libwolfssl dependency at all. Match what works.
  log "building wolfssl from source (several minutes)"
  "$LIVE_VENV/bin/pip" install --no-cache-dir --no-binary wolfssl wolfssl==5.9.2.post0 \
    > /tmp/live_py_wolfssl.log 2>&1 || {
      tail -25 /tmp/live_py_wolfssl.log; die 12 "wolfssl build failed"; }
}

verify() {
  "$LIVE_VENV/bin/python" - <<'PYEOF' || die 13 "wolfssl unusable under this interpreter"
import sys
import wolfssl
print("  python  %s" % sys.version.split()[0])
print("  wolfssl %s" % getattr(wolfssl, "__version__", "unknown"))
ctx = wolfssl.SSLContext(wolfssl.PROTOCOL_DTLSv1_3, server_side=True)
print("  DTLS 1.3 server context: OK")
PYEOF
  log "agent interpreter ready at $LIVE_VENV"
  log "now re-run install_amu.sh to install the omega-amu-live service"
}

build_interpreter
build_venv
verify
