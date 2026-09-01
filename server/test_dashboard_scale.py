"""What the dashboard does when the fleet is 40 units, not 2.

The chart endpoints downsample once a window holds more rows than the browser
can draw. Downsampling is a GROUP BY, and a GROUP BY that forgets the device id
does not thin the data - it deletes devices. With one NMU on the desk that is
invisible; at fleet size it is the whole dashboard.

No network, no live server: a seeded database and the real query text.

    py -3.12 server/test_dashboard_scale.py
"""

import os
import sqlite3
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import storage
os.environ.setdefault("OMEGA_DB", os.path.join(tempfile.gettempdir(), "unused.db"))
import app

FLEET = 40
MINUTES = 90
NOISE_FIELDS = "id, timestamp, db, duration"


def _seed(db_file, devices, seconds, per_second):
    conn, cursor = storage.open_db(db_file)
    base = int(time.time()) - seconds
    rows = []
    for index in range(devices):
        device_id = "NMU_%02d" % (index + 1)
        for step in range(0, seconds, per_second):
            rows.append((device_id, base + step, "%d_%d" % (index, step),
                         60.0 + index, 3.0, 0))
    cursor.executemany("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)", rows)
    conn.commit()
    conn.close()
    return len(rows)


def _run(db_file, query, args):
    conn = sqlite3.connect(db_file)
    conn.row_factory = sqlite3.Row
    rows = conn.execute(query, args).fetchall()
    conn.close()
    return rows


def test_downsampling_keeps_every_device():
    """The defect: GROUP BY timestamp/bucket with no device id collapses all
    40 units into one row per time bucket, and SQLite's bare-column rule hands
    back an arbitrary one of them. 39 units silently disappear from the chart
    the moment the window crosses the row cap."""
    tmp = tempfile.mkdtemp(prefix="omega_scale_")
    db_file = os.path.join(tmp, "sensor_data.db")
    window_s = MINUTES * 60
    total = _seed(db_file, FLEET, window_s, 3)
    cutoff = int(time.time()) - window_s
    cap = app.WINDOW_ROW_CAP
    assert total > cap, "the fixture must actually cross the cap: %d" % total

    broken = ("SELECT " + NOISE_FIELDS + ", MAX(db) AS peak FROM noise_data "
              "WHERE heartbeat = 0 AND timestamp >= ? "
              "GROUP BY timestamp / ? ORDER BY timestamp ASC")
    seen_broken = {r["id"] for r in _run(db_file, broken,
                                        (cutoff, app.bucket_seconds(window_s)))}

    fixed = ("SELECT " + NOISE_FIELDS + ", MAX(db) AS peak FROM noise_data "
             "WHERE heartbeat = 0 AND timestamp >= ? "
             "GROUP BY id, timestamp / ? ORDER BY timestamp ASC")
    seen_fixed = {r["id"] for r in _run(db_file, fixed,
                                       (cutoff, app.bucket_seconds(window_s)))}

    assert len(seen_broken) < FLEET, (
        "this test is meaningless unless the un-grouped query really does lose "
        "devices; it returned %d of %d" % (len(seen_broken), FLEET))
    assert len(seen_fixed) == FLEET, (
        "grouping by device must keep all %d units, got %d"
        % (FLEET, len(seen_fixed)))
    print("PASS downsampling keeps every device "
          "(un-grouped query returned only %d of %d)"
          % (len(seen_broken), FLEET))


def test_point_budget_is_shared_across_the_fleet():
    """A per-window row cap is not a browser budget: the cap decides the bucket
    WIDTH, and every device then contributes its own point per bucket. With the
    device id correctly in the GROUP BY, 40 devices return 40x the points the
    cap implies unless the bucket is widened by the fleet size."""
    tmp = tempfile.mkdtemp(prefix="omega_budget_")
    db_file = os.path.join(tmp, "sensor_data.db")
    window_s = MINUTES * 60
    _seed(db_file, FLEET, window_s, 3)
    cutoff = int(time.time()) - window_s
    cap = app.WINDOW_ROW_CAP

    query = ("SELECT " + NOISE_FIELDS + ", MAX(db) AS peak FROM noise_data "
             "WHERE heartbeat = 0 AND timestamp >= ? "
             "GROUP BY id, timestamp / ? ORDER BY timestamp ASC")

    naive = len(_run(db_file, query, (cutoff, app.bucket_seconds(window_s))))
    assert naive > cap, (
        "with the fleet ignored, the cap is exceeded %.1fx - this is the "
        "browser-side crash" % (naive / float(cap)))

    devices = _run(db_file, "SELECT COUNT(DISTINCT id) AS n FROM noise_data "
                            "WHERE timestamp >= ?", (cutoff,))[0]["n"]
    fleet_bucket = app.bucket_seconds(window_s, devices)
    budgeted = len(_run(db_file, query, (cutoff, fleet_bucket)))
    assert budgeted <= cap, (
        "widening the bucket by fleet size must bring the total under the cap: "
        "%d > %d" % (budgeted, cap))
    print("PASS fleet-aware bucket holds the point budget "
          "(%d devices: %d points naive -> %d budgeted, cap %d)"
          % (devices, naive, budgeted, cap))


def main():
    test_downsampling_keeps_every_device()
    test_point_budget_is_shared_across_the_fleet()
    print("RESULT: PASS - dashboard queries survive a 40-unit fleet")
    return 0


if __name__ == "__main__":
    sys.exit(main())
