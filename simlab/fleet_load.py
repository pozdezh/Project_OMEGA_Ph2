"""Virtual fleet load test: what the dashboard costs at 40 units, measured.

Builds a database the size a real 40-unit fleet produces in a day and drives
the ACTUAL Flask endpoints against it - not a reimplementation of their
queries. Reports wall-clock per request, the number of devices each endpoint
returns, and the payload size the browser has to parse.

Runs entirely on this machine. No devices, no server, no network.

    py -3.12 simlab/fleet_load.py            40 NMU + 8 AMU, 24h
    py -3.12 simlab/fleet_load.py --quick    same shape, 2h (fast iteration)
"""

import json
import os
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "server"))

NMU_COUNT = 40
AMU_COUNT = 8
# Measured on the live fleet 2026-08-24: NMU median inter-record gap 3s,
# AMU 62s. These are not guesses about a busy site - they are what the
# hardware on the desk is doing right now.
NMU_PERIOD_S = 3
AMU_PERIOD_S = 62
WINDOWS = (900, 3600, 21600, 86400, 604800)


def build_db(db_path, seconds):
    import storage
    conn, cursor = storage.open_db(db_path)
    conn.execute("PRAGMA synchronous=OFF;")
    now = int(time.time())
    base = now - seconds

    noise = []
    for unit in range(NMU_COUNT):
        device_id = "NMU_%02d" % (unit + 1)
        for offset in range(0, seconds, NMU_PERIOD_S):
            ts = base + offset + (unit % NMU_PERIOD_S)
            noise.append((device_id, ts, "%d_%d" % (unit, offset),
                          55.0 + (offset % 40), 2.5, 0))
    cursor.executemany("INSERT OR IGNORE INTO noise_data VALUES (?,?,?,?,?,?)", noise)

    air = []
    for unit in range(AMU_COUNT):
        device_id = "AMU_%02d" % (unit + 1)
        for offset in range(0, seconds, AMU_PERIOD_S):
            ts = base + offset + unit
            cause = "Sustained Alarm: High Temp" if unit == 0 and offset % 600 == 0 \
                else "CO2 Step/Drift"
            air.append((device_id, ts, "%d_%d" % (unit, offset), 0,
                        700.0 + (offset % 300), 24.0, 55.0, 24.5, 56.0,
                        5.0, 8.0, 11.0, 24.2, 54.0, 1010.0, 120.0, cause))
    cursor.executemany(
        "INSERT OR IGNORE INTO air_data VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", air)
    conn.commit()
    conn.close()
    return len(noise), len(air)


def timed(client, path):
    start = time.time()
    response = client.get(path)
    elapsed = (time.time() - start) * 1000.0
    body = response.get_data()
    try:
        parsed = json.loads(body)
    except ValueError:
        parsed = []
    return elapsed, parsed, len(body)


def main():
    seconds = 2 * 3600 if "--quick" in sys.argv else 24 * 3600
    tmp = tempfile.mkdtemp(prefix="omega_fleet_")
    db_path = os.path.join(tmp, "sensor_data.db")

    print("building %d NMU + %d AMU over %dh ..." % (NMU_COUNT, AMU_COUNT, seconds // 3600))
    start = time.time()
    noise_rows, air_rows = build_db(db_path, seconds)
    size_mb = os.path.getsize(db_path) / (1024.0 * 1024.0)
    print("  %d noise + %d air rows, %.1f MB, built in %.1fs"
          % (noise_rows, air_rows, size_mb, time.time() - start))
    print()

    os.environ["OMEGA_DB"] = db_path
    os.environ["OMEGA_DEVICE_CONFIG"] = os.path.join(tmp, "device_config.json")
    with open(os.environ["OMEGA_DEVICE_CONFIG"], "w", encoding="utf-8") as handle:
        json.dump({"devices": [], "revoked": [], "nmu": {"hb": 20}, "amu": {"hb": 10}}, handle)

    import app
    app.app.config["TESTING"] = True
    client = app.app.test_client()

    worst = 0.0
    print("%-34s %9s %9s %9s" % ("endpoint", "time", "points", "payload"))
    print("-" * 64)
    for window in WINDOWS:
        for name, path in (("noise", "/api/noise/latest?window_s=%d" % window),
                           ("air", "/api/air/latest?window_s=%d" % window)):
            elapsed, rows, size = timed(client, path)
            devices = len({r.get("id") for r in rows}) if rows else 0
            expected = NMU_COUNT if name == "noise" else AMU_COUNT
            flag = "" if devices == expected else "  <-- LOST DEVICES (%d/%d)" % (devices, expected)
            worst = max(worst, elapsed)
            print("%-34s %7.0fms %9d %8.0fkB%s"
                  % ("%s %5ds" % (name, window), elapsed, len(rows), size / 1024.0, flag))

    print()
    print("second pass (repeat viewer / second browser tab):")
    repeat_worst = 0.0
    for window in WINDOWS:
        for name, path in (("noise", "/api/noise/latest?window_s=%d" % window),
                           ("air", "/api/air/latest?window_s=%d" % window)):
            elapsed, rows, size = timed(client, path)
            repeat_worst = max(repeat_worst, elapsed)
            print("%-34s %7.0fms %9d %8.0fkB"
                  % ("%s %5ds" % (name, window), elapsed, len(rows), size / 1024.0))
    print("  worst repeat request: %.0f ms" % repeat_worst)
    print()

    for name, path in (("/api/keys", "/api/keys"),
                       ("/api/mcp/devices", "/api/mcp/devices")):
        cold, rows, size = timed(client, path)
        warm, _rows2, _size2 = timed(client, path)
        worst = max(worst, cold)
        repeat_worst = max(repeat_worst, warm)
        print("%-34s %7.0fms %9d %8.0fkB   (repeat %.0fms)"
              % (name, cold, len(rows), size / 1024.0, warm))

    # Two budgets, because they are two different experiences. STEADY is what
    # the dashboard costs continuously - every reconcile, every extra browser,
    # every open tab - and it is the number that decides whether the server
    # copes with a fleet. COLD is the one-off cost of opening a wide historical
    # window for the first time in a cache period; it is a visible pause, not a
    # failure, and it does not block other requests (app.run threaded=True).
    STEADY_BUDGET_MS = 250.0
    COLD_BUDGET_MS = 3000.0
    print()
    print("cold  worst: %7.0f ms  (budget %.0f)" % (worst, COLD_BUDGET_MS))
    print("steady worst: %7.0f ms  (budget %.0f)" % (repeat_worst, STEADY_BUDGET_MS))
    failed = []
    if worst > COLD_BUDGET_MS:
        failed.append("cold %.0fms > %.0fms" % (worst, COLD_BUDGET_MS))
    if repeat_worst > STEADY_BUDGET_MS:
        failed.append("steady %.0fms > %.0fms" % (repeat_worst, STEADY_BUDGET_MS))
    if failed:
        print("RESULT: FAIL - " + "; ".join(failed))
        return 1
    print("RESULT: PASS - dashboard holds both budgets at %d NMU + %d AMU"
          % (NMU_COUNT, AMU_COUNT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
