"""Gate check: readings taken while the clock was UNKNOWN still land in the past.

test_gap_fill.py already proves a backlog is filed at its measured time - but
only for readings the device was able to date when it took them. This covers
the case that actually bit the fleet: a unit that does not know what time it
is, because it has just rebooted with no network and a Raspberry Pi has no
battery-backed clock.

Observed on AMU_15, 2026-08-27. It rebooted itself to recover, kept measuring
for 23 minutes while still cut off, and when the link returned all 15 of those
readings were filed at 02:39 - the moment they arrived. The user spotted it
from the shape of the data alone: "multiple alarms from amu15 at approx the
same time 2.39". The room had been steady, so the burst could not be real.

The device now records how long ago each reading was taken, measured against
the machine's boot rather than the wall clock, and converts that into a real
time at the moment of sending. This test drives the SHIPPED _to_record() and
the SHIPPED clock module, over the real DTLS channel, into the real server
storage, and checks where the readings actually end up in the database.

    py -3.12 simlab/test_undated_backlog.py
"""

import io
import os
import queue
import re
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "server"))
sys.path.insert(0, os.path.join(ROOT, "amu"))

import config_store
import dtls_loopback
import storage
from sim_lab import CIPHERS, _build_pki, _one_round_trip, _write_config

import clock


class _ConfigStub(object):
    DEVICE_ID = "AMU_01"


def _shipped_to_record():
    """Load the REAL _to_record out of the shipped network.py.

    Importing the module wholesale is impossible on this machine - it pulls in
    wolfSSL, which has no Windows build - so the function is read from the file
    and run against the real clock module. That still tests the code that
    ships; a reimplementation here would test nothing.
    """
    with io.open(os.path.join(ROOT, "amu", "network.py"), encoding="utf-8") as handle:
        source = handle.read()
    body = re.search(r"^def _to_record\(.*?(?=^def )", source, re.S | re.M).group(0)
    namespace = {"amu_config": _ConfigStub, "clock": clock, "_last_cfg_ver": None}
    exec(compile(body, "network.py", "exec"), namespace)
    return namespace["_to_record"]


def test_readings_taken_with_no_clock_land_where_they_belong():
    tmp = tempfile.mkdtemp(prefix="omega_undated_")
    _build_pki(tmp)
    cfg = os.path.join(tmp, "device_config.json")
    _write_config(cfg)
    db_file = os.path.join(tmp, "sensor_data.db")
    store = config_store.ConfigStore(cfg)
    packet_queue = queue.Queue()
    threading.Thread(target=storage.db_worker, args=(db_file, packet_queue),
                     daemon=True).start()

    to_record = _shipped_to_record()

    # The unit has just rebooted with no network: no NTP, no server, no idea
    # what time it is. Every reading it takes is stamped 0 and carries only
    # how long ago it happened.
    clock._server_synced = False
    clock._ntp_at_start = False
    clock._restored_across_planned_reboot = False
    assert not clock.trusted(), "the fixture must start with an unusable clock"

    now_up = clock._since_boot_s()
    boot = clock._boot_identity()

    # Twenty readings, one a minute, across the 20 minutes it was cut off.
    # Oldest first: 20 minutes ago ... 1 minute ago.
    captured = []
    for minutes_ago in range(20, 0, -1):
        payload = {"id": "AMU_01", "type": "airq", "ts": clock.stamp(),
                   "event": "700_%d" % (21 - minutes_ago), "hb": False,
                   "cause": "SIM",
                   "sensors": {"scd30": {"co2_ppm": float(400 + minutes_ago)}}}
        assert payload["ts"] == 0, "a unit with no clock must stamp 0, not a guess"
        payload["_cap"] = {"up": now_up - minutes_ago * 60.0, "boot": boot}
        captured.append((minutes_ago, payload))

    # The link returns. The server's reply teaches the unit the time, and the
    # whole backlog goes out in one burst - seconds, not twenty minutes.
    clock.apply_server_time(time.time())
    assert clock.trusted()
    drain_started = int(time.time())

    with dtls_loopback.DtlsChannel(tmp, "omega-server", "AMU_01", CIPHERS) as ch:
        for _, payload in captured:
            _one_round_trip(ch, store, packet_queue, to_record(payload))

    packet_queue.join()
    packet_queue.put(None)

    conn, cur = storage.open_db(db_file)
    cur.execute("SELECT timestamp, scd_co2, event FROM air_data "
                "WHERE id='AMU_01' ORDER BY timestamp")
    rows = cur.fetchall()
    conn.close()

    assert len(rows) == 20, "expected 20 readings, stored %d" % len(rows)

    stored = [int(r[0]) for r in rows]
    spread = stored[-1] - stored[0]

    # THE FAILURE THIS EXISTS FOR: without the fix every reading is filed at
    # the moment of delivery and the spread collapses to a couple of seconds.
    assert spread > 1000, (
        "the outage collapsed onto the reconnect: 20 readings spanning 20 "
        "minutes were filed across %d seconds" % spread)
    assert 1100 < spread < 1300, "expected roughly 1140s of history, got %d" % spread

    # Each reading sits where it belongs, not merely 'spread out'.
    for index, (minutes_ago, _) in enumerate(captured):
        expected = drain_started - minutes_ago * 60
        assert abs(stored[index] - expected) <= 5, (
            "reading from %d min ago filed %ds off its true position"
            % (minutes_ago, stored[index] - expected))

    # Consecutive, one a minute: the gap is filled, with no hole and no pile.
    gaps = sorted({stored[i + 1] - stored[i] for i in range(len(stored) - 1)})
    assert gaps == [60], "readings are not one minute apart: gaps %s" % gaps

    # Values track their own timestamps, so the sequence is genuinely restored.
    values = [int(r[1]) for r in rows]
    assert values == sorted(values, reverse=True), values

    # Nothing filed in the future, and nothing filed at the drain moment.
    assert stored[-1] <= drain_started + 5, "a reading was filed in the future"
    assert stored[0] < drain_started - 1000, "the oldest reading did not move back"

    print("PASS readings taken with no clock land at their true positions")
    print("     drained in seconds, filed across %d s of history" % spread)
    print("     oldest %+ds from drain, newest %+ds"
          % (stored[0] - drain_started, stored[-1] - drain_started))
    print("     one-minute spacing preserved, no cluster at the reconnect")


def test_a_reading_that_still_has_no_clock_is_not_invented():
    """The other half. If the time is STILL unknown when the reading is sent,
    the device must send 0 and let the server date it by arrival - which is
    honest - rather than fabricate a position on the timeline."""
    to_record = _shipped_to_record()
    clock._server_synced = False
    clock._ntp_at_start = False
    clock._restored_across_planned_reboot = False

    payload = {"id": "AMU_01", "type": "airq", "ts": 0, "event": "701_1",
               "hb": False, "cause": "SIM", "sensors": {},
               "_cap": {"up": clock._since_boot_s() - 600.0,
                        "boot": clock._boot_identity()}}
    record = to_record(payload)
    assert record["ts"] == 0, (
        "with no trustworthy clock the device must admit it cannot tell, "
        "got %r" % record["ts"])
    assert "_cap" not in record, "the internal marker must never reach the wire"
    print("PASS an unknowable time is still reported as unknown, never guessed")


if __name__ == "__main__":
    test_readings_taken_with_no_clock_land_where_they_belong()
    test_a_reading_that_still_has_no_clock_is_not_invented()
    print("\nOK")
