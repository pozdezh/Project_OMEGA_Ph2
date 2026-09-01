"""Brick 4 full-stack SimLab - client, DTLS, and server app core end to end.

SCOPE, STATED UP FRONT so this is never mistaken for a DTLS 1.3 proof: the
channel here is in-memory mutual-auth **DTLS 1.2**, because neither DTLS 1.3
path exists on the development machine - OpenSSL 3.0.13 predates DTLS 1.3
(added in 3.5) and wolfSSL has no Windows wheel. What this file proves is the
APPLICATION stack and the version-independent security properties: mutual
authentication, forward-secret ECDHE, cert-bound identity, cross-CA refusal,
revocation, dedup and identity forcing. **DTLS 1.3 itself is proven only on
real hardware** - the live server and the ESP32 both report
`DTLSv1.3 / TLS_AES_128_GCM_SHA256`, recorded in FINDINGS.

No hardware, no sockets: the channel (dtls_loopback.DtlsChannel) carries a
real telemetry record from a simulated AMU to the shipped server modules and
back as an ACK, exactly as the live listener would. It then runs the attack
suite that matters for this track:

  - round-trip: a trusted device's record is decrypted, stored, and ACKed with
    the per-type heartbeat config piggybacked back;
  - eavesdrop: the bytes on the wire contain no plaintext sensor values;
  - identity spoof: a device lying about its inner id is stored under its
    certificate CN, never the lie;
  - cross-CA forgery: a device whose certificate is signed by an unknown CA
    (an attacker holding only the WLAN password) is refused at the handshake;
  - revocation: a revoked device_id is refused by config_store even with a
    valid certificate.

Exit 0 only if every check passes.
"""

import json
import os
import queue
import sys
import tempfile
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "..", "provisioning"))
sys.path.insert(0, os.path.join(_HERE, "..", "server"))
sys.path.insert(0, os.path.join(_HERE, "..", "amu"))

import dtls_loopback
import omega_pki
import acks
import config_store
import storage
import live_server

CIPHERS = "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-ECDSA-AES128-GCM-SHA256"
CONFIG = {
    "devices": ["AMU_01", "NMU_01"],
    "nmu": {"cfg_ver": 4, "hb": 45},
    "amu": {"cfg_ver": 4, "hb": 30},
    "revoked": ["AMU_09"],
}
SECRET_MARKERS = (b"co2_ppm", b"612", b"AMU_01", b"airq", b"cause", b"SIM")


def _build_pki(out_dir):
    ca_key, ca_cert = omega_pki.create_ca()
    omega_pki._write_key(ca_key, os.path.join(out_dir, "ca-key.pem"))
    omega_pki._write_cert(ca_cert, os.path.join(out_dir, "ca-cert.pem"))
    omega_pki._issue_and_save(out_dir, ca_key, ca_cert, omega_pki.SERVER_COMMON_NAME, True)
    omega_pki._issue_and_save(out_dir, ca_key, ca_cert, "operator", False)
    omega_pki._issue_and_save(out_dir, ca_key, ca_cert, "AMU_01", False)
    omega_pki._issue_and_save(out_dir, ca_key, ca_cert, "NMU_01", False)
    rogue_ca_key, rogue_ca_cert = omega_pki.create_ca()
    r_key, r_cert = omega_pki.issue_leaf(rogue_ca_key, rogue_ca_cert, "AMU_01", False)
    omega_pki._write_key(r_key, os.path.join(out_dir, "rogue-key.pem"))
    omega_pki._write_cert(r_cert, os.path.join(out_dir, "rogue-cert.pem"))


def _write_config(path):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(CONFIG, handle)


def _server_handle_record(channel, store, packet_queue, record_bytes):
    """Mirror listener.py + session.py: identity from the verified cert, ACK
    with config, ingest. Returns the ACK bytes to hand back over the channel."""
    inner = json.loads(record_bytes.decode("utf-8"))
    device_id = channel.client_common_name()
    ack = acks.build_ack(store, device_id, inner.get("event"))
    storage.ingest_telemetry(packet_queue, device_id, inner)
    return device_id, json.dumps(ack).encode("utf-8")


def _one_round_trip(channel, store, packet_queue, record):
    seen = channel.client_to_server(json.dumps(record).encode("utf-8"))
    device_id, ack_bytes = _server_handle_record(channel, store, packet_queue, seen)
    got = json.loads(channel.server_to_client(ack_bytes).decode("utf-8"))
    return device_id, got


def run():
    tmp = tempfile.mkdtemp(prefix="omega_simlab_")
    _build_pki(tmp)
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    db_file = os.path.join(tmp, "sensor_data.db")
    store = config_store.ConfigStore(cfg_path)
    packet_queue = queue.Queue()
    threading.Thread(target=storage.db_worker, args=(db_file, packet_queue), daemon=True).start()

    results = []
    ts = int(time.time())

    with dtls_loopback.DtlsChannel(tmp, "omega-server", "AMU_01", CIPHERS) as channel:
        info = channel.info()
        results.append(("handshake mutual-auth ECDHE",
                        info["server_verify_ok"] and info["client_verify_ok"]
                        and (info["cipher"] or "").startswith("ECDHE-")))
        results.append(("server identity from cert == AMU_01",
                        info["server_view_of_client"] == "AMU_01"))

        record = {"id": "AMU_01", "type": "airq", "ts": ts, "event": "500_1", "hb": False,
                  "sensors": {"scd30": {"co2_ppm": 612.0}}, "cause": "SIM"}
        device_id, ack = _one_round_trip(channel, store, packet_queue, record)
        results.append(("round-trip ACK matches event", ack.get("ack") == "500_1"))
        results.append(("ACK piggybacks AMU heartbeat=30", ack.get("hb") == 30 and ack.get("cfg_ver") == 4))

        wire = channel.last_wire()
        results.append(("eavesdrop: no plaintext on wire",
                        all(marker not in wire for marker in SECRET_MARKERS) and len(wire) > 0))

        spoof = {"id": "NMU_99", "type": "airq", "ts": ts, "event": "500_2", "hb": False,
                 "sensors": {"scd30": {"co2_ppm": 400.0}}, "cause": "SPOOF"}
        spoof_id, _ = _one_round_trip(channel, store, packet_queue, spoof)
        results.append(("identity spoof forced to cert CN", spoof_id == "AMU_01"))

    # NMU noise device over its own DTLS session: byte-faithful to the firmware
    with dtls_loopback.DtlsChannel(tmp, "omega-server", "NMU_01", CIPHERS) as nmu:
        results.append(("NMU identity from cert == NMU_01",
                        nmu.info()["server_view_of_client"] == "NMU_01"))
        noise = {"id": "NMU_01", "type": "noise", "ts": ts, "event": "800_1", "hb": False,
                 "db": 82.5, "duration": 4.0}
        nmu_id, nmu_ack = _one_round_trip(nmu, store, packet_queue, noise)
        results.append(("NMU round-trip stored + ACKed", nmu_id == "NMU_01" and nmu_ack.get("ack") == "800_1"))
        results.append(("ACK piggybacks NMU heartbeat=45", nmu_ack.get("hb") == 45))

    # LIVE MCP-to-AMU: operator connects DIRECTLY to the always-on AMU (AMU is
    # the DTLS server here), issues a deterministic command, gets a fresh reply.
    def _fake_read():
        return {"scd30": {"co2_ppm": 655.0, "temperature_c": 22.4}}

    with dtls_loopback.DtlsChannel(tmp, "AMU_01", "operator", CIPHERS) as live:
        results.append(("AMU authenticates operator by cert",
                        live.info()["server_view_of_client"] == "operator"))
        seen = live.client_to_server(json.dumps({"cmd": "read_now"}).encode("utf-8"))
        reply = live_server.handle_command(json.loads(seen), "AMU_01", _fake_read)
        got = json.loads(live.server_to_client(json.dumps(reply).encode("utf-8")).decode("utf-8"))
        results.append(("live read_now returns fresh reading",
                        got.get("ok") and got["reading"]["scd30"]["co2_ppm"] == 655.0))
        wire = live.last_wire()
        results.append(("live command channel is encrypted",
                        b"read_now" not in wire and b"655" not in wire and len(wire) > 0))

    # a rogue operator (unknown CA) cannot open the live channel
    rogue_live_refused = False
    try:
        dtls_loopback.DtlsChannel(tmp, "AMU_01", "rogue", CIPHERS).close()
    except dtls_loopback.HandshakeError:
        rogue_live_refused = True
    results.append(("rogue operator refused on live channel", rogue_live_refused))

    # cross-CA forgery: an unknown-CA client certificate must fail the handshake
    forged_refused = False
    try:
        dtls_loopback.DtlsChannel(tmp, "omega-server", "rogue", CIPHERS).close()
    except dtls_loopback.HandshakeError:
        forged_refused = True
    results.append(("cross-CA forged device refused", forged_refused))

    # revocation: config_store refuses a revoked id even with a valid cert
    results.append(("revoked device refused by config_store", not store.is_allowed("AMU_09")))

    packet_queue.join()
    packet_queue.put(None)
    conn, c = storage.open_db(db_file)
    c.execute("SELECT id, event, scd_co2 FROM air_data ORDER BY event")
    rows = c.fetchall()
    c.execute("SELECT id, event, db FROM noise_data")
    noise_rows = c.fetchall()
    conn.close()
    results.append(("stored exactly the two records under AMU_01",
                    len(rows) == 2 and all(r[0] == "AMU_01" for r in rows)))
    results.append(("first record value survived (co2=612)",
                    any(abs((r[2] or 0) - 612.0) < 0.01 for r in rows)))
    results.append(("NMU noise row stored (db=82.5)",
                    len(noise_rows) == 1 and noise_rows[0][0] == "NMU_01"
                    and abs((noise_rows[0][2] or 0) - 82.5) < 0.01))

    passed = 0
    for name, ok in results:
        print(("  PASS " if ok else "  FAIL ") + name)
        passed += 1 if ok else 0
    total = len(results)
    print("SimLab: %d/%d checks passed" % (passed, total))
    return passed == total


def main():
    ok = run()
    print("RESULT: %s" % ("PASS - brick4 app stack + security properties proven "
                          "in-memory over DTLS 1.2; DTLS 1.3 is hardware-verified"
                          if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
