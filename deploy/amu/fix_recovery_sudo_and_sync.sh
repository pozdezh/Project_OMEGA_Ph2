#!/bin/sh
# Run ONCE from this PC. Brings the 7 remaining AMUs up to the fixed build.
#
# Per unit: install the recovery sudo rule, copy the corrected network.py,
# restart the service, and print the file fingerprint so the fleet can be
# checked for uniformity afterwards.
#
# ssh -t is not optional: without a terminal on the far end sudo has nowhere
# to ask for its password and every unit fails silently. sudo caches the
# answer for the rest of that session, so each unit asks exactly once.
#
# All paths are derived from this script's own location - it must work no
# matter which directory it is invoked from.
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/payload/network.py"
RULE="$HERE/omega_fix_recovery_sudo.sh"
# Every AMU, always. AMU_15 was left out of the first run on 2026-08-26
# because a CACHED sudo timestamp made it look like it already had the
# privilege. These Pis use timestamp_type=global: one sudo unlocks the
# whole machine for every session for ~15 minutes, so "sudo -n works"
# proves nothing unless "sudo -k" was run first. Re-running on an
# already-correct unit is harmless; skipping one is not.
UNITS="${OMEGA_AMU_UNITS:-10 11 12 13 14 15 16 17}"

[ -f "$SRC" ] || { echo "missing $SRC" >&2; exit 1; }
[ -f "$RULE" ] || { echo "missing $RULE" >&2; exit 1; }

echo "Password for each unit is the AMU console password (all clones share it)."
echo

for n in $UNITS; do
  echo "=== amu$n ==="
  if ! scp -q "$SRC" "amu$n:/tmp/network.py"; then
    echo "amu$n: COPY FAILED - skipped"; echo; continue
  fi
  if ! scp -q "$RULE" "amu$n:/tmp/omega_fix_recovery_sudo.sh"; then
    echo "amu$n: COPY FAILED - skipped"; echo; continue
  fi
  ssh -t "amu$n" 'set -e
    sudo sh /tmp/omega_fix_recovery_sudo.sh
    D=$(systemctl cat omega-amu | grep -m1 WorkingDirectory | cut -d= -f2)
    cp /tmp/network.py "$D/network.py"
    sudo systemctl restart omega-amu
    sleep 2
    printf "service=%s  network.py=%s\n" "$(systemctl is-active omega-amu)" "$(md5sum "$D/network.py" | cut -c1-12)"
    rm -f /tmp/network.py /tmp/omega_fix_recovery_sudo.sh'
  echo
done

echo "Done. Every unit above should read RULE_OK, service=active,"
echo "and the SAME network.py fingerprint."
