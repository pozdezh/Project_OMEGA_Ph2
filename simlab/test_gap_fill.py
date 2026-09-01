"""Gate check: a backlog delivered late must land in the PAST, not the present.

This is the property that makes an outage survivable rather than merely
non-fatal. A unit that loses its link keeps measuring and keeps the readings;
when the link returns it sends them all at once. If the server filed them at
the moment they ARRIVED, the record would show a flat gap followed by an
impossible spike of simultaneous events - the outage would be visible in the
data as a hole that never fills, and every statistic over that window would
be wrong.

What must be true instead: each record carries the time it was MEASURED, the
server stores that time, and ordering by it reconstructs the true sequence
with the gap filled in.

    py -3.12 simlab/test_gap_fill.py
"""

import json
import os
import queue
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "server"))

import config_store
import dtls_loopback
import storage
from sim_lab import CIPHERS, _build_pki, _one_round_trip, _write_config


def test_late_backlog_lands_at_its_measured_time():
    tmp = tempfile.mkdtemp(prefix="omega_gap_")
    _build_pki(tmp)
    cfg = os.path.join(tmp, "device_config.json")
    _write_config(cfg)
    db_file = os.path.join(tmp, "sensor_data.db")
    store = config_store.ConfigStore(cfg)
    packet_queue = queue.Queue()
    threading.Thread(target=storage.db_worker, args=(db_file, packet_queue),
                     daemon=True).start()

    base = int(time.time()) - 3600

    # A unit measuring once a minute. The link dies after minute 2 and comes
    # back at minute 8, so minutes 3-7 are measured but cannot be sent.
    before = [(base + 60 * m, 400 + m) for m in (1, 2)]
    during_outage = [(base + 60 * m, 400 + m) for m in (3, 4, 5, 6, 7)]
    after = [(base + 60 * 8, 408)]

    def record(ts, co2, counter):
        return {"id": "AMU_01", "type": "airq", "ts": ts,
                "event": "900_%d" % counter, "hb": False,
                "sensors": {"scd30": {"co2_ppm": float(co2)}}, "cause": "SIM"}

    with dtls_loopback.DtlsChannel(tmp, "omega-server", "AMU_01", CIPHERS) as ch:
        counter = 0
        # Normal running.
        for ts, co2 in before:
            counter += 1
            _one_round_trip(ch, store, packet_queue, record(ts, co2, counter))

        # The link returns. The unit sends its NEWEST reading first - that is
        # the liveness rule - and only then drains the backlog oldest-first.
        # So the order on the wire is deliberately NOT chronological.
        counter += 1
        _one_round_trip(ch, store, packet_queue, record(after[0][0], after[0][1], counter))
        for ts, co2 in during_outage:
            counter += 1
            _one_round_trip(ch, store, packet_queue, record(ts, co2, counter))

    packet_queue.join()
    packet_queue.put(None)

    conn, cur = storage.open_db(db_file)
    cur.execute("SELECT timestamp, scd_co2, event FROM air_data "
                "WHERE id='AMU_01' ORDER BY timestamp")
    rows = cur.fetchall()
    conn.close()

    expected_times = [ts for ts, _ in before + during_outage + after]
    stored_times = [int(r[0]) for r in rows]
    assert stored_times == expected_times, (
        "stored times do not reconstruct the real sequence\n  expected %s\n  got      %s"
        % (expected_times, stored_times))

    # The gap is filled: consecutive minutes, no hole where the outage was.
    gaps = [stored_times[i + 1] - stored_times[i] for i in range(len(stored_times) - 1)]
    assert set(gaps) == {60}, "a hole remains in the record: gaps were %s" % gaps

    # And the VALUES follow their own timestamps, not their arrival order.
    stored_values = [int(r[1]) for r in rows]
    assert stored_values == [401, 402, 403, 404, 405, 406, 407, 408], stored_values

    # Arrival order really was scrambled - otherwise this proves nothing.
    conn, cur = storage.open_db(db_file)
    cur.execute("SELECT event FROM air_data WHERE id='AMU_01' ORDER BY rowid")
    arrival = [r[0] for r in cur.fetchall()]
    conn.close()
    assert arrival[2] == "900_3", (
        "the newest reading should have arrived BEFORE the backlog, got %s" % arrival)

    print("PASS backlog delivered out of order is stored at its measured time")
    print("     arrival order : %s" % " ".join(arrival))
    print("     stored order  : %s" % " ".join(
        "%s" % t for t in [r[2] for r in rows]))
    print("     gaps between consecutive records: %s s (no hole)" % sorted(set(gaps)))


def test_a_unit_with_no_trusted_clock_is_stamped_on_arrival():
    """The one case where the server substitutes its own time.

    A device whose clock is not trusted sends ts=0 rather than a wrong time.
    Storing a wrong time would be worse than storing arrival time: it would
    silently place the reading somewhere it never happened.
    """
    tmp = tempfile.mkdtemp(prefix="omega_noclock_")
    _build_pki(tmp)
    cfg = os.path.join(tmp, "device_config.json")
    _write_config(cfg)
    db_file = os.path.join(tmp, "sensor_data.db")
    store = config_store.ConfigStore(cfg)
    packet_queue = queue.Queue()
    threading.Thread(target=storage.db_worker, args=(db_file, packet_queue),
                     daemon=True).start()

    sent_at = int(time.time())
    with dtls_loopback.DtlsChannel(tmp, "omega-server", "AMU_01", CIPHERS) as ch:
        _one_round_trip(ch, store, packet_queue,
                        {"id": "AMU_01", "type": "airq", "ts": 0,
                         "event": "901_1", "hb": False,
                         "sensors": {"scd30": {"co2_ppm": 500.0}},
                         "cause": "NOCLOCK"})
    packet_queue.join()
    packet_queue.put(None)

    conn, cur = storage.open_db(db_file)
    cur.execute("SELECT timestamp FROM air_data WHERE event='901_1'")
    stored = int(cur.fetchone()[0])
    conn.close()

    assert abs(stored - sent_at) < 30, (
        "a reading with no trusted clock should be stamped on arrival, got %d vs %d"
        % (stored, sent_at))
    print("PASS a reading with no trusted clock is stamped on arrival, not at 0")


def main():
    test_late_backlog_lands_at_its_measured_time()
    test_a_unit_with_no_trusted_clock_is_stamped_on_arrival()
    print("RESULT: PASS - a late backlog fills the gap it left")
    return 0


if __name__ == "__main__":
    sys.exit(main())
