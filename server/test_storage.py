"""Unit tests for storage.py (no sockets, no crypto).

Proves dedup, identity forced to the authenticated device_id, and correct
routing of noise vs air rows.
"""

import os
import queue
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import storage


def _drain(packet_queue):
    packet_queue.join()
    packet_queue.put(None)


def test_dedup_identity_and_routing():
    tmp = tempfile.mkdtemp(prefix="omega_storage_")
    db_file = os.path.join(tmp, "sensor_data.db")
    pq = queue.Queue()
    worker = threading.Thread(target=storage.db_worker, args=(db_file, pq), daemon=True)
    worker.start()

    ts = int(time.time())
    # a device that lies about its inner id must still be stored under its real id
    storage.ingest_telemetry(pq, "AMU_01", {
        "id": "NMU_99", "type": "airq", "ts": ts, "event": "500_1",
        "sensors": {"scd30": {"co2_ppm": 600.0}}, "cause": "TEST"})
    # duplicate (id, event) must not create a second row
    storage.ingest_telemetry(pq, "AMU_01", {
        "id": "AMU_01", "type": "airq", "ts": ts, "event": "500_1",
        "sensors": {"scd30": {"co2_ppm": 999.0}}, "cause": "DUP"})
    # a noise row routes to the other table
    storage.ingest_telemetry(pq, "NMU_01", {
        "id": "NMU_01", "type": "noise", "ts": ts, "event": "700_3",
        "db": 82.5, "duration": 4.0})
    _drain(pq)

    conn, c = storage.open_db(db_file)
    c.execute("SELECT id, event, scd_co2, cause FROM air_data")
    air = c.fetchall()
    c.execute("SELECT id, event, db FROM noise_data")
    noise = c.fetchall()
    conn.close()

    assert len(air) == 1, "dedup kept one air row, got " + str(air)
    assert air[0][0] == "AMU_01", "identity forced to authenticated id, got " + air[0][0]
    assert air[0][2] == 600.0, "first write survived dedup"
    assert len(noise) == 1 and noise[0][0] == "NMU_01" and noise[0][2] == 82.5, "noise routed"
    print("PASS dedup + identity + routing")


def main():
    test_dedup_identity_and_routing()
    print("RESULT: PASS - storage behaviour verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
