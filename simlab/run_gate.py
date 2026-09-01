"""Brick 4 verification gate - run before any deploy handoff.

Three steps, each a standalone script so failures are isolated and readable:
  1. proof_handshake - the true-DTLS known-answer: mutual auth, forward-secret
     ECDHE cipher, cert-bound identity, untrusted device refused;
  2. config_store/acks/storage unit tests - transport-independent server
     behaviour (config/ACK piggyback, dedup, identity forcing, routing);
  3. sim_lab - full-stack in-memory: client -> DTLS -> server modules -> ACK,
     plus the attack suite (eavesdrop, spoof, cross-CA forgery, revocation).

Exit 0 only if all three exit 0. Mirrors the discipline of the Brick 1/2 gate;
live UDP DTLS is validated on the Linux server/AMU, not on this Windows laptop.
"""

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STEPS = (
    ("true-DTLS handshake known-answer", os.path.join(HERE, "proof_handshake.py")),
    ("server config store + acks unit tests", os.path.join(ROOT, "server", "test_config_store.py")),
    ("NMU mailbox (queued questions)", os.path.join(ROOT, "server", "test_nmu_mailbox.py")),
    ("server storage unit tests", os.path.join(ROOT, "server", "test_storage.py")),
    ("duplicate device identities refused", os.path.join(ROOT, "server", "test_identity_guard.py")),
    ("device addresses learned, not configured", os.path.join(ROOT, "server", "test_device_addresses.py")),
    ("cause field is a closed vocabulary", os.path.join(ROOT, "server", "test_cause_validation.py")),
    ("dashboard survives a 40-unit fleet", os.path.join(ROOT, "server", "test_dashboard_scale.py")),
    ("db retention + daily stats", os.path.join(ROOT, "server", "test_maintenance.py")),
    ("device API + MCP wrapper", os.path.join(ROOT, "server", "test_device_api.py")),
    ("live AMU command handler + MCP", os.path.join(ROOT, "server", "test_live.py")),
    ("server discovery (no hardcoded address)", os.path.join(HERE, "test_discovery.py")),
    ("AMU buffer atomicity + alarm hysteresis", os.path.join(ROOT, "amu", "test_buffer_triggers.py")),
    ("AMU self-recovery ladder + clock trust", os.path.join(ROOT, "amu", "test_recovery_clock.py")),
    ("AMU learned heartbeat survives a restart", os.path.join(ROOT, "amu", "test_heartbeat_persistence.py")),
    ("rogue discovery responder refused", os.path.join(ROOT, "amu", "test_rogue_server.py")),
    ("AMU dates a buffered reading by capture, not arrival",
     os.path.join(ROOT, "amu", "test_stamp_resolution.py")),
    ("AMU log survives the reboot the ladder performs",
     os.path.join(ROOT, "amu", "test_applog.py")),
    ("liveness: newest first, backlog on its own clock", os.path.join(ROOT, "amu", "test_liveness.py")),
    ("late backlog fills the gap it left", os.path.join(HERE, "test_gap_fill.py")),
    ("undated backlog lands at its true position, not the reconnect",
     os.path.join(HERE, "test_undated_backlog.py")),
    ("NMU dates a buffered record by capture, not arrival",
     os.path.join(HERE, "test_nmu_time_wrapper.py")),
    ("discovery ranking + retry budget + backlog order", os.path.join(HERE, "test_fleet_scenarios.py")),
    ("installers copy every module they import", os.path.join(ROOT, "deploy", "test_installer_payload.py")),
    ("full-stack in-memory SimLab", os.path.join(HERE, "sim_lab.py")),
    ("shipped payload byte-sync", os.path.join(HERE, "test_payload_sync.py")),
)

# amu_config reads one ini at import. The fixture carries thresholds only, no
# credentials, so the AMU's logic is testable on any machine. The heartbeat
# state file goes to a scratch path so a gate run never writes into the repo.
_AMU_FIXTURE = {
    "OMEGA_AMU_CONFIG": os.path.join(ROOT, "amu", "test_fixture.ini"),
    "OMEGA_HEARTBEAT_STATE": os.path.join(
        tempfile.gettempdir(), "omega_gate_heartbeat_state.json"),
    "OMEGA_CONTACT_STATE": os.path.join(
        tempfile.gettempdir(), "omega_gate_contact_state.json"),
}
STEP_ENV = {
    os.path.join(ROOT, "amu", "test_buffer_triggers.py"): _AMU_FIXTURE,
    os.path.join(ROOT, "amu", "test_rogue_server.py"): _AMU_FIXTURE,
    os.path.join(ROOT, "amu", "test_liveness.py"): _AMU_FIXTURE,
    os.path.join(ROOT, "amu", "test_stamp_resolution.py"): _AMU_FIXTURE,
    os.path.join(HERE, "test_undated_backlog.py"): _AMU_FIXTURE,
    os.path.join(HERE, "test_fleet_scenarios.py"): _AMU_FIXTURE,
}


def _run(label, script):
    print("=== " + label + " ===")
    env = dict(os.environ)
    env.update(STEP_ENV.get(script, {}))
    result = subprocess.run([sys.executable, script], env=env)
    ok = result.returncode == 0
    print("--- " + label + ": " + ("PASS" if ok else "FAIL") + "\n")
    return ok


SCOPE_NOTE = """
SCOPE OF THIS GATE - read before quoting it as proof:
  * It runs IN MEMORY on this machine. Green means the logic is sound, not
    that it works on the desk.
  * The in-memory channel is DTLS 1.2, not 1.3: OpenSSL 3.0.13 here predates
    DTLS 1.3 (added in 3.5) and wolfSSL has no Windows wheel. What is proven
    version-independently is mutual auth, forward-secret ECDHE, cert-bound
    identity, cross-CA refusal and revocation.
  * DTLS 1.3 ITSELF is proven only on hardware - the live server and the
    ESP32 both negotiate DTLSv1.3 / TLS_AES_128_GCM_SHA256.
  * It cannot see file descriptors, task starvation, radio or power faults.
"""


def main():
    passed = all(_run(label, script) for label, script in STEPS)
    print("GATE: " + ("PASS - brick4 ready for deploy handoff" if passed else "FAIL - do not deploy"))
    print(SCOPE_NOTE)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
