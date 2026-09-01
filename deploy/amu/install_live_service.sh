#!/usr/bin/env bash
# One-shot: install the AMU inbound live-query agent as a systemd service and
# restart the telemetry service onto the new trigger-only code.
#
# Run with sudo. Everything else in this feature needed no root; this step
# does, because it writes a unit file into /etc/systemd/system.

set -u

TARGET=/home/amu10/omega_amu
LIVE_PY="$TARGET/venv311/bin/python"
UNIT=/etc/systemd/system/omega-amu-live.service

echo "== checking prerequisites =="
[ -x "$LIVE_PY" ] || { echo "MISSING: $LIVE_PY (run build_live_py.sh first)"; exit 1; }
[ -f "$TARGET/live_agent.py" ] || { echo "MISSING: $TARGET/live_agent.py"; exit 1; }
"$LIVE_PY" -c "import wolfssl" || { echo "MISSING: wolfssl in the agent venv"; exit 1; }
echo "ok"

echo "== stopping the temporary test supervisor =="
pkill -f "bash /tmp/agent_loop.sh" 2>/dev/null
pkill -f "venv311/bin/python -u /home/amu10/omega_amu/live_agent.py" 2>/dev/null
sleep 1

echo "== writing $UNIT =="
# Restart=always is load-bearing, not boilerplate. The wolfSSL Python binding
# segfaults in its DTLS server role after a handful of sessions (FINDINGS
# #35, exit code 139 observed on real hardware). The agent holds no state -
# it answers from a cache file another process writes - so a restart costs
# nothing and takes about two seconds. StartLimitIntervalSec=0 stops systemd
# from ever giving up permanently, which must never change.
#
# RestartSteps/RestartMaxDelaySec (systemd >=254) back off a RUN of failures
# 2s -> 4s -> 8s -> capped at 30s, then reset to 2s once the service stays up
# a while. Added 2026-08-23 after a real toggling session produced 69 restarts
# in under an hour (all self-healed, zero data lost - FINDINGS - but that many
# restart/log lines for one interrupted-handshake pattern is noise worth
# damping). A single stuck handshake still recovers in ~2s as before; only
# REPEATED failures in quick succession now space out.
cat > "$UNIT" <<EOF
[Unit]
Description=Omega Brick 4 AMU inbound live-query agent
After=network-online.target omega-amu.service
Wants=network-online.target

StartLimitIntervalSec=0

[Service]
Type=simple
User=amu10
WorkingDirectory=$TARGET
Environment=PYTHONUNBUFFERED=1
ExecStart=$LIVE_PY $TARGET/live_agent.py
Restart=always
RestartSec=2
RestartSteps=4
RestartMaxDelaySec=30

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable omega-amu-live >/dev/null 2>&1
systemctl restart omega-amu-live

echo "== restarting telemetry onto the trigger-only code =="
systemctl restart omega-amu

sleep 6
echo
echo "== omega-amu (telemetry) =="
systemctl is-active omega-amu
echo "== omega-amu-live (inbound agent) =="
systemctl is-active omega-amu-live
echo
echo "== interpreters in use =="
echo "telemetry : $(systemctl show -p ExecStart --value omega-amu | grep -o '/[^ ]*python[^ ]*' | head -1)"
echo "live agent: $LIVE_PY"
echo
echo "== listener present? =="
ss -ulnp 2>/dev/null | grep 5001 || echo "no listener yet (may still be starting)"
echo
echo "DONE"
