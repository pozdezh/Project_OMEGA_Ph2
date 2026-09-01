"""Gate check: the shipped deploy payloads are byte-identical to the tested
source modules. The gate exercises the source; deployment copies the payload -
this guarantees they are the same bytes, so a green gate really covers what
ships. If you edit a source module, re-copy it into the payload or this fails.
"""

import hashlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PAIRS = [
    ("server/listener.py", "deploy/server/payload/listener.py"),
    ("server/session.py", "deploy/server/payload/session.py"),
    ("server/storage.py", "deploy/server/payload/storage.py"),
    ("server/identity_guard.py", "deploy/server/payload/identity_guard.py"),
    ("server/cause_validation.py", "deploy/server/payload/cause_validation.py"),
    ("server/config_store.py", "deploy/server/payload/config_store.py"),
    ("server/device_addresses.py", "deploy/server/payload/device_addresses.py"),
    ("server/acks.py", "deploy/server/payload/acks.py"),
    ("server/nmu_mailbox.py", "deploy/server/payload/nmu_mailbox.py"),
    ("server/db_retention.py", "deploy/server/payload/db_retention.py"),
    ("server/clear_database.py", "deploy/server/payload/clear_database.py"),
    ("server/daily_stats.py", "deploy/server/payload/daily_stats.py"),
    ("server/device_api.py", "deploy/server/payload/device_api.py"),
    ("server/mcp_server.py", "deploy/server/payload/mcp_server.py"),
    ("server/device_live.py", "deploy/server/payload/device_live.py"),
    ("server/discovery.py", "deploy/server/payload/discovery.py"),
    ("server/app.py", "deploy/server/payload/app.py"),
    ("server/templates/index.html", "deploy/server/payload/templates/index.html"),
    ("amu/main.py", "deploy/amu/payload/main.py"),
    ("amu/amu_config.py", "deploy/amu/payload/amu_config.py"),
    ("amu/sensors.py", "deploy/amu/payload/sensors.py"),
    ("amu/triggers.py", "deploy/amu/payload/triggers.py"),
    ("amu/buffer.py", "deploy/amu/payload/buffer.py"),
    ("amu/recovery.py", "deploy/amu/payload/recovery.py"),
    ("amu/retry_schedule.py", "deploy/amu/payload/retry_schedule.py"),
    ("nmu/omega_audio.cpp", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_audio.cpp"),
    ("nmu/omega_audio.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_audio.h"),
    ("nmu/omega_buffer.cpp", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_buffer.cpp"),
    ("nmu/omega_buffer.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_buffer.h"),
    ("nmu/omega_config.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_config.h"),
    ("nmu/omega_discovery.cpp", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_discovery.cpp"),
    ("nmu/omega_discovery.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_discovery.h"),
    ("nmu/omega_dtls.cpp", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_dtls.cpp"),
    ("nmu/omega_dtls.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_dtls.h"),
    ("nmu/omega_net.cpp", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_net.cpp"),
    ("nmu/omega_net.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_net.h"),
    ("nmu/omega_tasks.cpp", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_tasks.cpp"),
    ("nmu/omega_tasks.h", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/omega_tasks.h"),
    ("nmu/partitions.csv", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/partitions.csv"),
    ("nmu/stage3_beta_2_dtls.ino", "deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls/stage3_beta_2_dtls.ino"),
    ("nmu/omega_audio.cpp", "provisioning/pendrive/firmware/omega_audio.cpp"),
    ("nmu/omega_audio.h", "provisioning/pendrive/firmware/omega_audio.h"),
    ("nmu/omega_buffer.cpp", "provisioning/pendrive/firmware/omega_buffer.cpp"),
    ("nmu/omega_buffer.h", "provisioning/pendrive/firmware/omega_buffer.h"),
    ("nmu/omega_config.h", "provisioning/pendrive/firmware/omega_config.h"),
    ("nmu/omega_discovery.cpp", "provisioning/pendrive/firmware/omega_discovery.cpp"),
    ("nmu/omega_discovery.h", "provisioning/pendrive/firmware/omega_discovery.h"),
    ("nmu/omega_dtls.cpp", "provisioning/pendrive/firmware/omega_dtls.cpp"),
    ("nmu/omega_dtls.h", "provisioning/pendrive/firmware/omega_dtls.h"),
    ("nmu/omega_net.cpp", "provisioning/pendrive/firmware/omega_net.cpp"),
    ("nmu/omega_net.h", "provisioning/pendrive/firmware/omega_net.h"),
    ("nmu/omega_tasks.cpp", "provisioning/pendrive/firmware/omega_tasks.cpp"),
    ("nmu/omega_tasks.h", "provisioning/pendrive/firmware/omega_tasks.h"),
    ("nmu/partitions.csv", "provisioning/pendrive/firmware/partitions.csv"),
    ("nmu/stage3_beta_2_dtls.ino", "provisioning/pendrive/firmware/stage3_beta_2_dtls.ino"),
    ("amu/applog.py", "deploy/amu/payload/applog.py"),
    ("amu/clock.py", "deploy/amu/payload/clock.py"),
    ("amu/network.py", "deploy/amu/payload/network.py"),
    ("amu/dtls_client.py", "deploy/amu/payload/dtls_client.py"),
    ("amu/wolfssl_guard.py", "deploy/amu/payload/wolfssl_guard.py"),
    ("amu/live_server.py", "deploy/amu/payload/live_server.py"),
    ("amu/live_agent.py", "deploy/amu/payload/live_agent.py"),
    ("amu/live_cache.py", "deploy/amu/payload/live_cache.py"),
    ("amu/server_discovery.py", "deploy/amu/payload/server_discovery.py"),
    ("nmu/omega_dtls.h", "deploy/esp32/stage3_beta_2_dtls/omega_dtls.h"),
    ("nmu/omega_dtls.cpp", "deploy/esp32/stage3_beta_2_dtls/omega_dtls.cpp"),
    ("nmu/omega_discovery.h", "deploy/esp32/stage3_beta_2_dtls/omega_discovery.h"),
    ("nmu/omega_discovery.cpp", "deploy/esp32/stage3_beta_2_dtls/omega_discovery.cpp"),
    ("nmu/omega_config.h", "deploy/esp32/stage3_beta_2_dtls/omega_config.h"),
    ("nmu/omega_audio.h", "deploy/esp32/stage3_beta_2_dtls/omega_audio.h"),
    ("nmu/omega_audio.cpp", "deploy/esp32/stage3_beta_2_dtls/omega_audio.cpp"),
    ("nmu/omega_buffer.h", "deploy/esp32/stage3_beta_2_dtls/omega_buffer.h"),
    ("nmu/omega_buffer.cpp", "deploy/esp32/stage3_beta_2_dtls/omega_buffer.cpp"),
    ("nmu/omega_net.h", "deploy/esp32/stage3_beta_2_dtls/omega_net.h"),
    ("nmu/omega_net.cpp", "deploy/esp32/stage3_beta_2_dtls/omega_net.cpp"),
    ("nmu/omega_tasks.h", "deploy/esp32/stage3_beta_2_dtls/omega_tasks.h"),
    ("nmu/omega_tasks.cpp", "deploy/esp32/stage3_beta_2_dtls/omega_tasks.cpp"),
    ("nmu/stage3_beta_2_dtls.ino", "deploy/esp32/stage3_beta_2_dtls/stage3_beta_2_dtls.ino"),
    ("nmu/partitions.csv", "deploy/esp32/stage3_beta_2_dtls/partitions.csv"),
]


def _digest(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def _uncovered_installed_files():
    """Anything an installer SHIPS but this list never checks for drift.

    PAIRS is hand-maintained, which is the same weakness that let two modules
    go missing from the installers themselves (FINDINGS #44). Deriving the
    expected set from the installers closes the loop: if a file is shipped, it
    must be drift-checked."""
    sys.path.insert(0, os.path.join(ROOT, "deploy"))
    try:
        import test_installer_payload as guard
    except ImportError:
        return []
    checked = {os.path.basename(payload) for _, payload in PAIRS}
    missing = []
    for label, installer, payload_dir in guard.TARGETS:
        if not os.path.exists(installer):
            continue
        for name in sorted(guard.copied_files(installer)):
            if not name.endswith(".py"):
                continue
            if name in guard.OPTIONAL:
                continue
            if not os.path.exists(os.path.join(payload_dir, name)):
                continue
            if name not in checked:
                missing.append("%s ships %s but payload-sync never checks it"
                               % (label, name))
    return missing


def main():
    failures = 0
    for source_rel, payload_rel in PAIRS:
        source = os.path.join(ROOT, source_rel)
        payload = os.path.join(ROOT, payload_rel)
        if not os.path.exists(payload):
            print("  MISSING payload: " + payload_rel)
            failures += 1
            continue
        if _digest(source) != _digest(payload):
            print("  DRIFT: " + payload_rel + " differs from " + source_rel)
            failures += 1
        else:
            print("  OK " + payload_rel)
    print("payload sync: %d/%d in sync" % (len(PAIRS) - failures, len(PAIRS)))
    for gap in _uncovered_installed_files():
        print("  GAP " + gap)
        failures += 1
    if failures:
        print("RESULT: FAIL - re-copy changed modules into the deploy payloads")
        return 1
    print("RESULT: PASS - shipped payloads match tested source")
    return 0


if __name__ == "__main__":
    sys.exit(main())
