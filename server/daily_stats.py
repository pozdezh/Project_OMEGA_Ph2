"""Brick 4 reporting - daily descriptive statistics push (Phase 2 goal 4).

Each run covers records from the END of the last SUCCESSFULLY DELIVERED
report up to now - never a fixed lookback window. A fixed window either
skips data (if a run is ever missed - server down, etc.) or double-counts it
(overlapping windows); a rolling checkpoint does neither. A device that only
started sending after the last report is naturally covered from its own
first record onward, since there is nothing earlier than that to miss.

For each device and each numeric variable it reports count, min, max, mean,
median and standard deviation. The report id is date-stamped
(OMEGA_YYYYMMDD, UTC) rather than random, so reports sort and file
predictably on the receiving end.

Read-only over the database (never disturbs the live listener). Network send
is injectable so the maths and the payload shape are unit-testable with no
server. The rolling checkpoint lives in a small sibling JSON file next to the
database, never inside it - this job must not touch the live write schema.

Cron:
    python3 daily_stats.py /path/to/sensor_data.db
Env:
    OMEGA_STATS_URL       https endpoint to POST to (required to actually send)
    OMEGA_STATS_TOKEN     bearer token sent as Authorization header
    OMEGA_STATS_WINDOW_H  bootstrap window in hours, first run only (default 24)
"""

import json
import math
import os
import statistics
import sqlite3
import sys
import time
import urllib.request

NOISE_COLUMNS = ("db", "duration")
AIR_COLUMNS = ("scd_co2", "scd_temp", "scd_hum", "dht_temp", "dht_hum",
               "pms_pm1", "pms_pm25", "pms_pm10", "env_temp", "env_hum",
               "env_press", "env_lux")
TABLES = (("noise_data", NOISE_COLUMNS), ("air_data", AIR_COLUMNS))
DEFAULT_WINDOW_HOURS = 24
SECONDS_PER_HOUR = 3600
STATE_FILE_SUFFIX = ".stats_state.json"
POST_RETRIES = 3
POST_BACKOFF_S = 2.0
POST_TIMEOUT_S = 15.0

# Physical plausibility bounds, inclusive - a value outside its column's
# range cannot be a real reading regardless of what the database holds.
# Sourced where a real sensor limit is known (see ARCHITECTURE.md, "AMU
# sensor calibration" / FINDINGS #25/#27/#29); a wide, documented sanity
# margin otherwise - never tight enough to reject a genuine reading.
# NMU: MAX_CHUNKS=80 chunks x 125ms in omega_config.h caps a real event at
# exactly 10s; dB SPL below 20 or above 130 is outside any plausible room.
# AMU: 0-100 is RH's hard physical ceiling, not a sensor spec (CMA-4544PF-W,
# DHT22, BME280 datasheets); CO2 400-10000ppm is the SCD30's own documented
# measurement range (Sensirion interface description) - the lower bound read
# 0.0 until 2026-08-21 despite this comment, which let 18.3% of stored
# readings through as "valid" while sitting below the sensor's own floor and
# below outdoor ambient (~425ppm), a level indoor air cannot reach. See
# FINDINGS #42; temperature range is
# the union of the DHT22/BME280/SCD30 datasheets' own stated operating
# range (-40 to 85C); pressure 700-1100hPa matches the SCD30's own accepted
# compensation range (this project's real observed range is far narrower).
PLAUSIBLE_RANGES = {
    "db": (20.0, 130.0),
    "duration": (0.0, 10.0),
    "scd_co2": (400.0, 10000.0),
    "scd_temp": (-40.0, 85.0),
    "scd_hum": (0.0, 100.0),
    "dht_temp": (-40.0, 85.0),
    "dht_hum": (0.0, 100.0),
    "env_temp": (-40.0, 85.0),
    "env_hum": (0.0, 100.0),
    "env_press": (700.0, 1100.0),
    "env_lux": (0.0, 100000.0),
    "pms_pm1": (0.0, 1000.0),
    "pms_pm25": (0.0, 1000.0),
    "pms_pm10": (0.0, 1000.0),
}


def _classify(column, value):
    if value is None:
        return "missing"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return "malformed"
    if math.isnan(value) or math.isinf(value):
        return "malformed"
    bounds = PLAUSIBLE_RANGES.get(column)
    if bounds is not None and not (bounds[0] <= value <= bounds[1]):
        return "implausible"
    return "valid"


def _describe(column, values):
    """Only 'valid' values (see _classify) feed min/max/avg/median/sd - a
    missing, malformed, or physically-impossible reading skews every one of
    those the same way whether it is silently dropped or silently kept, so
    it must never be silently anything: n_missing/n_malformed/n_implausible
    are always reported alongside the real numbers, not just excluded from
    them."""
    valid = []
    missing = malformed = implausible = 0
    for value in values:
        kind = _classify(column, value)
        if kind == "missing":
            missing += 1
        elif kind == "malformed":
            malformed += 1
        elif kind == "implausible":
            implausible += 1
        else:
            valid.append(value)
    if not valid:
        return None
    return {
        "n": len(valid),
        "n_missing": missing,
        "n_malformed": malformed,
        "n_implausible": implausible,
        "min": round(min(valid), 3),
        "max": round(max(valid), 3),
        "avg": round(statistics.fmean(valid), 3),
        "median": round(statistics.median(valid), 3),
        "sd": round(statistics.stdev(valid), 3) if len(valid) > 1 else 0.0,
    }


def _stats_for_table(conn, table, columns, cutoff_ts, end_ts):
    cursor = conn.cursor()
    # heartbeat = 0: keep-alive records carry no measured event. On the NMU a
    # heartbeat is stored with db = 0.0 and duration = 0.0 (omega_tasks.cpp),
    # so counting them would pin every duration statistic to zero and flag
    # every db value implausible; on the AMU they are periodic, which biases
    # the distribution toward quiet periods. Every other stats consumer
    # (app.py) already excludes them; this path must match.
    # Inclusive on both timestamp ends: the natural meaning for ANY caller,
    # including device_api.py's on-demand "last N hours as of right now"
    # queries, which have no later run to catch a record landing in this same
    # second. The cron path avoids double-counting not by excluding this
    # boundary, but by advancing its OWN next cutoff past it - see run()'s
    # checkpoint save.
    cursor.execute(
        "SELECT id, " + ", ".join(columns) + " FROM " + table +
        " WHERE heartbeat = 0 AND timestamp >= ? AND timestamp <= ?",
        (cutoff_ts, end_ts))
    per_device = {}
    for row in cursor.fetchall():
        device_id = row[0]
        bucket = per_device.setdefault(device_id, {c: [] for c in columns})
        for index, column in enumerate(columns, start=1):
            bucket[column].append(row[index])
    summarised = {}
    for device_id, series in per_device.items():
        variables = {}
        for column in columns:
            described = _describe(column, series[column])
            if described is not None:
                variables[column] = described
        if variables:
            summarised[device_id] = variables
    return summarised


def _new_report_id(end_ts):
    return "OMEGA_" + time.strftime("%Y%m%d", time.gmtime(end_ts))


def build_report(db_path, cutoff_ts, end_ts):
    conn = sqlite3.connect(db_path, timeout=30)
    conn.execute("PRAGMA journal_mode=WAL;")
    devices = {}
    for table, columns in TABLES:
        try:
            table_stats = _stats_for_table(conn, table, columns, cutoff_ts, end_ts)
        except sqlite3.OperationalError:
            continue
        for device_id, variables in table_stats.items():
            entry = devices.setdefault(device_id, {"device_id": device_id, "variables": {}})
            entry["variables"].update(variables)
    conn.close()
    return {
        "report_id": _new_report_id(end_ts),
        "generated_at": end_ts,
        "period_start": cutoff_ts,
        "period_end": end_ts,
        "devices": list(devices.values()),
    }


def _state_path(db_path):
    return db_path + STATE_FILE_SUFFIX


def _load_checkpoint(state_path):
    if not os.path.exists(state_path):
        return None
    try:
        with open(state_path, "r", encoding="utf-8") as handle:
            return json.load(handle).get("last_report_end_ts")
    except (OSError, ValueError):
        return None


def _save_checkpoint(state_path, end_ts):
    """Atomic write: a power cut mid-write must never corrupt the checkpoint
    into something that silently re-widens or skips the next report window."""
    tmp = state_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump({"last_report_end_ts": end_ts}, handle)
    os.replace(tmp, state_path)


def require_secure_url(url, allow_insecure=False):
    """Reject a plaintext endpoint. See ARCHITECTURE.md 18."""
    if not url:
        return True, ""
    if url.lower().startswith("https://"):
        return True, ""
    if allow_insecure:
        return True, ("WARNING: pushing over plaintext %s - the report AND the "
                      "bearer token are readable by anyone on the path"
                      % url.split("://")[0])
    return False, ("REFUSED: OMEGA_STATS_URL is %s, not https. The report "
                   "carries per-device statistics and the Authorization "
                   "header carries the bearer token; both would travel in "
                   "clear text. Use an https:// endpoint, or set "
                   "OMEGA_STATS_ALLOW_INSECURE=1 to override deliberately "
                   "(local simulator only)." % url.split("://")[0])


def _http_post(url, token, report):
    body = json.dumps(report).encode("utf-8")
    request = urllib.request.Request(url, data=body, method="POST")
    request.add_header("Content-Type", "application/json")
    if token:
        request.add_header("Authorization", "Bearer " + token)
    with urllib.request.urlopen(request, timeout=POST_TIMEOUT_S) as response:
        return 200 <= response.status < 300


def send_report(report, url, token, poster=_http_post):
    """POST the report with bounded retries and backoff. Returns True on the
    first success; False if all attempts fail (the report is also printed so a
    failed push is never silently lost)."""
    for attempt in range(POST_RETRIES):
        try:
            if poster(url, token, report):
                return True
        except Exception as error:
            print("stats push attempt %d failed: %s" % (attempt + 1, error))
        time.sleep(POST_BACKOFF_S * (attempt + 1))
    return False


def run(db_path, url, token, bootstrap_window_hours=DEFAULT_WINDOW_HOURS,
        poster=_http_post, state_path=None, allow_insecure=False):
    secure, message = require_secure_url(url, allow_insecure)
    if message:
        print(message)
    if not secure:
        return 2
    state_path = state_path or _state_path(db_path)
    end_ts = int(time.time())
    cutoff_ts = _load_checkpoint(state_path)
    if cutoff_ts is None:
        cutoff_ts = end_ts - bootstrap_window_hours * SECONDS_PER_HOUR
        print("stats: no checkpoint found, bootstrapping with a %dh window"
              % bootstrap_window_hours)

    report = build_report(db_path, cutoff_ts, end_ts)
    print("stats report %s: %d devices, period %s -> %s UTC"
          % (report["report_id"], len(report["devices"]),
             time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(cutoff_ts)),
             time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(end_ts))))

    if not url:
        print(json.dumps(report))
        print("stats: OMEGA_STATS_URL unset - printed report, not sent, "
              "checkpoint NOT advanced")
        return 0

    ok = send_report(report, url, token, poster)
    print("stats push: " + ("delivered" if ok else "FAILED (report printed above)"))
    if not ok:
        print(json.dumps(report))
        return 1

    # Only advance the checkpoint on confirmed delivery - a failed send must
    # be retried with the SAME start point next run, not silently skipped.
    # +1: this run already covered end_ts inclusively, so the next run's
    # cutoff starts one second later - otherwise the boundary second would
    # be reported twice, once by each of two consecutive runs.
    _save_checkpoint(state_path, end_ts + 1)
    return 0


def main(argv):
    if len(argv) < 2:
        print("usage: daily_stats.py <db_path>")
        return 2
    url = os.environ.get("OMEGA_STATS_URL", "")
    token = os.environ.get("OMEGA_STATS_TOKEN", "")
    allow_insecure = os.environ.get("OMEGA_STATS_ALLOW_INSECURE", "") == "1"
    bootstrap_hours = int(os.environ.get("OMEGA_STATS_WINDOW_H", DEFAULT_WINDOW_HOURS))
    return run(argv[1], url, token, bootstrap_window_hours=bootstrap_hours,
               allow_insecure=allow_insecure)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
