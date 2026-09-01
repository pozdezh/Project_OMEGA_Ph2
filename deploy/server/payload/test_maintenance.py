"""Unit tests for the Brick 3 maintenance jobs (db retention + daily stats).

No network and no live server: retention is checked on a seeded db, the stats
maths is checked against hand-computed values, and the HTTPS push is checked
with an injected poster (success and total-failure paths).
"""

import os
import sqlite3
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import storage
import db_retention
import daily_stats


def _seed_noise(db_file, rows):
    conn, c = storage.open_db(db_file)
    for i in range(rows):
        c.execute("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
                  ("NMU_01", 1000 + i, "900_" + str(i), 80.0 + (i % 5) * 2.0, 3.0, 0))
    conn.commit()
    conn.close()


def test_retention_prunes_oldest():
    """prune_table() itself is unconditional (the caller already decided
    pruning is needed) - this proves the 10-20% oldest-first deletion math."""
    tmp = tempfile.mkdtemp(prefix="omega_ret_")
    db_file = os.path.join(tmp, "sensor_data.db")
    _seed_noise(db_file, 100)
    conn = sqlite3.connect(db_file)
    deleted = db_retention.prune_table(conn, "noise_data", fraction=0.15)
    conn.commit()
    c = conn.cursor()
    c.execute("SELECT COUNT(*), MIN(timestamp) FROM noise_data")
    count, min_ts = c.fetchone()
    conn.close()
    assert deleted == 15, "deleted oldest 15%%, got " + str(deleted)
    assert count == 85, "85 rows remain, got " + str(count)
    assert min_ts == 1015, "oldest 15 removed, min ts now " + str(min_ts)
    print("PASS retention prunes oldest 10-20%")


def test_retention_under_size_budget_noop():
    """run() checks the DATABASE FILE'S SIZE ON DISK, not a row count - a
    tiny seeded db must never trigger pruning against a generous MB budget."""
    tmp = tempfile.mkdtemp(prefix="omega_ret2_")
    db_file = os.path.join(tmp, "sensor_data.db")
    _seed_noise(db_file, 10)
    rc = db_retention.run(db_file, max_size_mb=500, fraction=0.15)
    conn = sqlite3.connect(db_file)
    count = conn.execute("SELECT COUNT(*) FROM noise_data").fetchone()[0]
    conn.close()
    assert rc == 0 and count == 10, "under budget deletes nothing, got " + str(count)
    print("PASS retention no-op under size budget")


def test_retention_run_triggers_on_size():
    """run() with a near-zero MB budget must prune every managed table, since
    any real sqlite file exceeds a 0 MB budget."""
    tmp = tempfile.mkdtemp(prefix="omega_ret3_")
    db_file = os.path.join(tmp, "sensor_data.db")
    _seed_noise(db_file, 100)
    rc = db_retention.run(db_file, max_size_mb=0, fraction=0.15)
    conn = sqlite3.connect(db_file)
    count = conn.execute("SELECT COUNT(*) FROM noise_data").fetchone()[0]
    conn.close()
    assert rc == 0 and count == 85, "oldest 15%% pruned when over size budget, got " + str(count)
    print("PASS retention triggers and prunes when over size budget")


def test_stats_math_and_report():
    tmp = tempfile.mkdtemp(prefix="omega_stats_")
    db_file = os.path.join(tmp, "sensor_data.db")
    conn, c = storage.open_db(db_file)
    ts = int(time.time())
    for value in (80.0, 82.0, 84.0, 86.0, 88.0):
        c.execute("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
                  ("NMU_01", ts, "901_" + str(int(value)), value, 3.0, 0))
    conn.commit()
    conn.close()

    end_ts = ts + 1
    report = daily_stats.build_report(db_file, cutoff_ts=ts - 3600, end_ts=end_ts)
    assert report["report_id"] == "OMEGA_" + time.strftime("%Y%m%d", time.gmtime(end_ts)), \
        "report id is date-stamped: " + report["report_id"]
    devices = {d["device_id"]: d for d in report["devices"]}
    assert "NMU_01" in devices, "device present"
    db_stat = devices["NMU_01"]["variables"]["db"]
    assert db_stat["n"] == 5 and db_stat["min"] == 80.0 and db_stat["max"] == 88.0, str(db_stat)
    assert db_stat["avg"] == 84.0 and db_stat["median"] == 84.0, str(db_stat)
    assert abs(db_stat["sd"] - 3.162) < 0.01, "sd correct: " + str(db_stat["sd"])
    print("PASS stats math + report shape")


def test_stats_excludes_missing_and_implausible_values():
    """A NULL humidity, a >100%% humidity (physically impossible), and a
    genuinely valid reading must each be counted correctly: the impossible
    and missing values must never enter min/max/avg/median/sd, but must
    still be visibly reported, not silently vanish."""
    tmp = tempfile.mkdtemp(prefix="omega_bad_")
    db_file = os.path.join(tmp, "sensor_data.db")
    conn, c = storage.open_db(db_file)
    ts = int(time.time())
    rows = [
        ("AMU_01", ts, "1_1", 0, 500.0, 22.0, 55.0, 22.0, 55.0, 5, 5, 5, 22.0, 55.0, 1000.0, 10.0, "trigger"),
        ("AMU_01", ts + 1, "1_2", 0, 520.0, 22.5, None, 22.5, 55.5, 5, 5, 5, 22.5, 55.5, 1001.0, 10.0, "trigger"),
        ("AMU_01", ts + 2, "1_3", 0, 540.0, 23.0, 115.0, 23.0, 56.0, 5, 5, 5, 23.0, 56.0, 654.0, 10.0, "trigger"),
    ]
    c.executemany(
        "INSERT INTO air_data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        rows)
    conn.commit()
    conn.close()

    report = daily_stats.build_report(db_file, cutoff_ts=ts - 10, end_ts=ts + 10)
    variables = report["devices"][0]["variables"]

    hum_stat = variables["scd_hum"]
    assert hum_stat["n"] == 1 and hum_stat["n_missing"] == 1 and hum_stat["n_implausible"] == 1, \
        "1 valid (55.0), 1 missing (None), 1 implausible (115.0 > 100): " + str(hum_stat)
    assert hum_stat["avg"] == 55.0, "avg computed from the ONE valid value only: " + str(hum_stat)

    press_stat = variables["env_press"]
    assert press_stat["n"] == 2 and press_stat["n_implausible"] == 1, \
        "654 hPa (below 700 floor) excluded, two real readings kept: " + str(press_stat)
    assert press_stat["min"] == 1000.0 and press_stat["max"] == 1001.0, str(press_stat)

    co2_stat = variables["scd_co2"]
    assert co2_stat["n"] == 3 and co2_stat["n_missing"] == 0 and co2_stat["n_implausible"] == 0, \
        "all three CO2 readings are plausible: " + str(co2_stat)
    print("PASS stats exclude missing/implausible values, report their counts")


def test_stats_all_values_bad_returns_none():
    """A variable where every reading is missing or implausible must return
    None, not a fabricated stat over zero real values."""
    tmp = tempfile.mkdtemp(prefix="omega_allbad_")
    db_file = os.path.join(tmp, "sensor_data.db")
    conn, c = storage.open_db(db_file)
    ts = int(time.time())
    c.execute(
        "INSERT INTO air_data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        ("AMU_02", ts, "2_1", 0, 500.0, 22.0, 250.0, 22.0, 55.0, 5, 5, 5, 22.0, 55.0, 1000.0, 10.0, "trigger"))
    conn.commit()
    conn.close()

    report = daily_stats.build_report(db_file, cutoff_ts=ts - 10, end_ts=ts + 10)
    variables = report["devices"][0]["variables"]
    assert "scd_hum" not in variables, \
        "the only scd_hum reading (250%%) is implausible - must be omitted, not fabricated"
    print("PASS all-implausible variable is omitted, not fabricated as a zero-value stat")


def test_stats_since_last_report_no_gap_no_overlap():
    """The core requirement: each run covers exactly [checkpoint, now) - a
    record inserted between two runs must be picked up by the SECOND run
    exactly once, never zero times (a gap) and never twice (an overlap)."""
    tmp = tempfile.mkdtemp(prefix="omega_since_")
    db_file = os.path.join(tmp, "sensor_data.db")
    state_file = os.path.join(tmp, "state.json")

    conn, c = storage.open_db(db_file)
    ts1 = int(time.time()) - 10
    c.execute("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
              ("NMU_01", ts1, "1_1", 70.0, 1.0, 0))
    conn.commit()
    conn.close()

    sent = []
    ok_poster = lambda url, token, report: sent.append(report) or True

    rc1 = daily_stats.run(db_file, "https://x/y", "tok",
                          poster=ok_poster, state_path=state_file)
    assert rc1 == 0 and len(sent) == 1
    first_devices = {d["device_id"] for d in sent[0]["devices"]}
    assert "NMU_01" in first_devices, "first record covered by first run"

    time.sleep(1.1)  # cross a whole second: ts2 must differ from the first
                     # run's checkpoint (its own end_ts, saved above)
    conn, c = storage.open_db(db_file)
    ts2 = int(time.time())
    c.execute("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
              ("NMU_01", ts2, "1_2", 75.0, 1.0, 0))
    conn.commit()
    conn.close()

    time.sleep(1.1)  # cross another whole second: the second run's OWN
                     # end_ts (sampled fresh inside run()) must differ from
                     # ts2 too, or the half-open upper bound excludes it
    rc2 = daily_stats.run(db_file, "https://x/y", "tok",
                          poster=ok_poster, state_path=state_file)
    assert rc2 == 0 and len(sent) == 2
    second_db_stat = sent[1]["devices"][0]["variables"]["db"]
    assert second_db_stat["n"] == 1 and second_db_stat["min"] == 75.0, \
        "second run covers only the NEW record, got " + str(second_db_stat)
    print("PASS since-last-report window has no gap and no overlap")


def test_stats_failed_push_does_not_advance_checkpoint():
    tmp = tempfile.mkdtemp(prefix="omega_noadv_")
    db_file = os.path.join(tmp, "sensor_data.db")
    state_file = os.path.join(tmp, "state.json")
    conn, c = storage.open_db(db_file)
    c.execute("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
              ("NMU_01", int(time.time()), "2_1", 70.0, 1.0, 0))
    conn.commit()
    conn.close()

    daily_stats.POST_BACKOFF_S = 0.0
    bad_poster = lambda url, token, report: (_ for _ in ()).throw(OSError("refused"))
    rc = daily_stats.run(db_file, "https://x/y", "tok",
                         poster=bad_poster, state_path=state_file)
    assert rc == 1, "failed push returns non-zero"
    assert daily_stats._load_checkpoint(state_file) is None, \
        "checkpoint must not advance on failed delivery"
    print("PASS failed push leaves checkpoint untouched (no data silently skipped)")


def test_stats_push_paths():
    daily_stats.POST_BACKOFF_S = 0.0  # keep the failure path fast
    captured = {}

    def ok_poster(url, token, report):
        captured["report"] = report
        captured["token"] = token
        return True

    def bad_poster(url, token, report):
        raise OSError("connection refused")

    report = {"report_id": "OMEGA_20260817", "devices": []}
    assert daily_stats.send_report(report, "https://x/y", "tok", ok_poster), "success path"
    assert captured["token"] == "tok" and captured["report"]["report_id"] == "OMEGA_20260817"
    assert not daily_stats.send_report(report, "https://x/y", "tok", bad_poster), "failure path"
    print("PASS stats push success + failure")


def test_https_is_required_for_the_stats_push():
    ok, msg = daily_stats.require_secure_url("https://boss.example/ingest")
    assert ok and not msg, "https must be accepted silently"
    ok, msg = daily_stats.require_secure_url("http://boss.example/ingest")
    assert not ok, "plaintext http must be refused - it leaks the bearer token"
    assert "REFUSED" in msg
    ok, msg = daily_stats.require_secure_url("http://127.0.0.1:9443/x", allow_insecure=True)
    assert ok and "WARNING" in msg, "override must work but must warn"
    ok, msg = daily_stats.require_secure_url("")
    assert ok, "an unset url is the print-only mode, not an error"
    print("PASS https required for the stats push (override warns)")



def main():
    test_retention_prunes_oldest()
    test_retention_under_size_budget_noop()
    test_retention_run_triggers_on_size()
    test_stats_math_and_report()
    test_stats_excludes_missing_and_implausible_values()
    test_stats_all_values_bad_returns_none()
    test_stats_since_last_report_no_gap_no_overlap()
    test_stats_failed_push_does_not_advance_checkpoint()
    test_stats_push_paths()
    test_https_is_required_for_the_stats_push()
    print("RESULT: PASS - maintenance jobs verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
