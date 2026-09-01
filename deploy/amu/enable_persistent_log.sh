#!/bin/sh
# Run ONCE from this PC. Gives every AMU a log that survives its own reboot.
#
# Why this is not cosmetic: the recovery ladder's last rung REBOOTS the unit,
# and with a RAM-only journal that single most interesting event erases its own
# evidence. It happened for real on 2026-08-27 - AMU_15 rebooted itself unaided
# for the first time ever, and the rung-1 and rung-2 lines leading up to it were
# already gone by the time anyone could read them.
#
# /var/log/journal existed on these cards but was EMPTY, so Storage=auto quietly
# meant volatile. This says persistent and means it, capped at 200M because an
# SD card that fills up is its own outage.
#
# ssh -t is required: without a terminal on the far end sudo has nowhere to ask
# for its password. sudo caches the answer, so each unit asks exactly once.
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
UNITS="${OMEGA_AMU_UNITS:-10 11 12 13 14 15 16 17}"

echo "Password for each unit is the AMU console password (all clones share it)."
echo

for n in $UNITS; do
  echo "=== amu$n ==="
  ssh -t "amu$n" 'sudo sh -c "
    mkdir -p /var/log/journal
    systemd-tmpfiles --create --prefix /var/log/journal >/dev/null 2>&1 || true
    grep -qE \"^Storage=persistent\" /etc/systemd/journald.conf \
      || sed -i \"s/^#\\?Storage=.*/Storage=persistent/\" /etc/systemd/journald.conf
    grep -qE \"^SystemMaxUse=\" /etc/systemd/journald.conf \
      || echo SystemMaxUse=200M >> /etc/systemd/journald.conf
    systemctl restart systemd-journald
    sleep 1
    printf \"storage=%s  journal_dir=%s\n\" \
      \"$(grep -E ^Storage /etc/systemd/journald.conf)\" \
      \"$(ls /var/log/journal | head -1)\"
  "'
  echo
done

echo "Each unit should print Storage=persistent and a machine-id directory."
echo "From now on, journalctl -b -1 will show the boot BEFORE a self-reboot."
