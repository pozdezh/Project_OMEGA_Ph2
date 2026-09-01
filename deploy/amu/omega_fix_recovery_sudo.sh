#!/bin/sh
# Runs ON the AMU, as root, via sudo. Installs the recovery ladder's sudo rule.
#
# Rung 1 of the ladder brings WiFi back up with nmcli; rung 3 reboots the unit.
# Both need root, and the "amu" account has neither by default. Before
# 2026-08-26 the installer granted only /sbin/reboot, so rung 1 could never
# run at all - sudo simply asked for a password no unattended process could
# supply, and every real outage climbed straight past the cheap fix to a full
# reboot. Scoped to exactly these commands, never a blanket ALL.
set -e

target=/etc/sudoers.d/omega-amu-reboot
user=${SUDO_USER:-amu}
tmp=$(mktemp)

{
  echo "$user ALL=(root) NOPASSWD: /sbin/reboot"
  echo "$user ALL=(root) NOPASSWD: /usr/bin/nmcli connection up *"
  echo "$user ALL=(root) NOPASSWD: /usr/bin/nmcli dev wifi connect *"
} > "$tmp"

if visudo -c -f "$tmp" >/dev/null 2>&1; then
  install -m 0440 -o root -g root "$tmp" "$target"
  rm -f "$tmp"
  echo "RULE_OK ($target)"
else
  rm -f "$tmp"
  echo "RULE_FAILED_VALIDATION" >&2
  exit 1
fi
