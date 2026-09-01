"""Brick 3 device API - the query logic behind the MCP demo (Phase 2 goal 5).

These functions let a caller inspect the fleet directly from the server's own
database and push a heartbeat change, with no dependency on the Flask REST API.
They are deliberately plain (no MCP imports) so they unit-test on any machine;
mcp_server.py wraps them as MCP tools for Claude to call directly.
"""

import json
import os
import sqlite3
import time

import config_store

NOISE_FIELDS = ("id", "timestamp", "event", "db", "duration", "heartbeat")
AIR_FIELDS = ("id", "timestamp", "event", "heartbeat", "scd_co2", "scd_temp",
              "scd_hum", "dht_temp", "dht_hum", "pms_pm1", "pms_pm25", "pms_pm10",
              "env_temp", "env_hum", "env_press", "env_lux", "cause")
NMU_PREFIX = "NMU"
SECONDS_PER_HOUR = 3600
DEFAULT_STATS_WINDOW_H = 24


def _connect(db_path):
    conn = sqlite3.connect(db_path, timeout=15)
    conn.row_factory = sqlite3.Row
    return conn


def list_devices(db_path):
    """Every device that has ever reported, with its type, last-seen time and
    row count. Sorted by device id."""
    conn = _connect(db_path)
    seen = {}
    for table, kind in (("noise_data", "noise"), ("air_data", "airq")):
        try:
            rows = conn.execute(
                "SELECT id, MAX(timestamp) AS last_ts, COUNT(*) AS n FROM "
                + table + " GROUP BY id").fetchall()
        except sqlite3.OperationalError:
            continue
        for row in rows:
            seen[row["id"]] = {
                "device_id": row["id"], "type": kind,
                "last_seen": row["last_ts"], "rows": row["n"],
            }
    conn.close()
    return [seen[k] for k in sorted(seen)]


def _table_for(device_id):
    return "noise_data" if device_id.startswith(NMU_PREFIX) else "air_data"


def latest_reading(db_path, device_id):
    """The most recent stored row for a device as a plain dict, or None."""
    table = _table_for(device_id)
    fields = NOISE_FIELDS if table == "noise_data" else AIR_FIELDS
    conn = _connect(db_path)
    try:
        row = conn.execute(
            "SELECT " + ", ".join(fields) + " FROM " + table
            + " WHERE id = ? ORDER BY timestamp DESC, rowid DESC LIMIT 1",
            (device_id,)).fetchone()
    except sqlite3.OperationalError:
        row = None
    conn.close()
    return dict(row) if row is not None else None


def device_stats(db_path, device_id, hours=DEFAULT_STATS_WINDOW_H):
    """Descriptive stats for one device over a window, reusing the same maths
    as the daily push."""
    import daily_stats
    table = _table_for(device_id)
    columns = daily_stats.NOISE_COLUMNS if table == "noise_data" else daily_stats.AIR_COLUMNS
    end_ts = int(time.time())
    cutoff_ts = end_ts - hours * SECONDS_PER_HOUR
    conn = sqlite3.connect(db_path, timeout=15)
    try:
        table_stats = daily_stats._stats_for_table(conn, table, columns, cutoff_ts, end_ts)
    except sqlite3.OperationalError:
        table_stats = {}
    conn.close()
    return {"device_id": device_id, "window_hours": hours,
            "variables": table_stats.get(device_id, {})}


def set_heartbeat(config_path, device_type, minutes):
    """Push a new heartbeat (minutes) for a device type ('nmu' or 'amu') by
    updating device_config.json and bumping its cfg_ver - the same change the
    dashboard makes, delivered on each device's next ACK. Returns the new
    config block."""
    device_type = device_type.lower()
    if device_type not in ("nmu", "amu"):
        raise ValueError("device_type must be 'nmu' or 'amu'")
    if int(minutes) < 1:
        raise ValueError("minutes must be >= 1")
    with open(config_path, "r", encoding="utf-8") as handle:
        doc = json.load(handle)
    block = dict(doc.get(device_type, {}))
    block["hb"] = int(minutes)
    block["cfg_ver"] = int(block.get("cfg_ver", 0)) + 1
    doc[device_type] = block
    config_store.write_atomic(config_path, doc)
    return block


def activity_report(db_path, device_id, hours=DEFAULT_STATS_WINDOW_H):
    """Why a device transmitted over a window, and which way its readings moved.

    device_stats answers "what were the values". This answers "what happened
    and why" - the question an operator actually asks. Three parts:

      causes   - how many transmissions each trigger reason produced, so a
                 chatty device can be explained rather than guessed at;
      alarms   - each alarm episode with when it started, when it cleared and
                 how long it lasted, since a sustained alarm is the clinically
                 meaningful event in a care setting, not an instantaneous one;
      trends   - first value, last value and direction per variable, computed
                 from the stored records themselves.

    Read-only. Returns plain dicts so mcp_server can hand it straight to the
    model without post-processing.
    """
    table = _table_for(device_id)
    end_ts = int(time.time())
    cutoff_ts = end_ts - hours * SECONDS_PER_HOUR
    conn = _connect(db_path)

    causes = {}
    alarm_episodes = []
    trends = {}
    total = 0

    try:
        if table == "air_data":
            rows = conn.execute(
                "SELECT timestamp, cause, scd_co2, dht_temp, dht_hum, pms_pm25, env_lux "
                "FROM air_data WHERE id = ? AND timestamp >= ? ORDER BY timestamp ASC",
                (device_id, cutoff_ts)).fetchall()
            tracked = ("scd_co2", "dht_temp", "dht_hum", "pms_pm25", "env_lux")
        else:
            rows = conn.execute(
                "SELECT timestamp, db, duration FROM noise_data "
                "WHERE id = ? AND timestamp >= ? ORDER BY timestamp ASC",
                (device_id, cutoff_ts)).fetchall()
            tracked = ("db", "duration")
    except sqlite3.OperationalError:
        rows = []
        tracked = ()
    conn.close()

    total = len(rows)
    open_alarms = {}
    series = {name: [] for name in tracked}

    for row in rows:
        cause = row["cause"] if table == "air_data" else "noise event"
        causes[cause] = causes.get(cause, 0) + 1

        for name in tracked:
            value = row[name]
            if isinstance(value, (int, float)):
                series[name].append((row["timestamp"], value))

        if table != "air_data" or not cause:
            continue
        if cause.startswith("New Alarm: ") or cause.startswith("Sustained Alarm: "):
            label = cause.split(": ", 1)[1].split(" (+")[0]
            if label not in open_alarms:
                open_alarms[label] = row["timestamp"]
        elif cause == "All Alarms Cleared":
            for label, started in open_alarms.items():
                alarm_episodes.append({
                    "alarm": label, "started": started, "cleared": row["timestamp"],
                    "duration_s": row["timestamp"] - started, "ongoing": False})
            open_alarms = {}

    for label, started in open_alarms.items():
        alarm_episodes.append({
            "alarm": label, "started": started, "cleared": None,
            "duration_s": end_ts - started, "ongoing": True})

    for name, points in series.items():
        if len(points) < 2:
            continue
        first_value, last_value = points[0][1], points[-1][1]
        change = last_value - first_value
        spread = max(p[1] for p in points) - min(p[1] for p in points)
        if spread and abs(change) > spread * 0.25:
            direction = "rising" if change > 0 else "falling"
        else:
            direction = "steady"
        trends[name] = {
            "first": round(first_value, 2), "last": round(last_value, 2),
            "change": round(change, 2), "direction": direction,
            "min": round(min(p[1] for p in points), 2),
            "max": round(max(p[1] for p in points), 2),
            "samples": len(points),
        }

    ranked = sorted(causes.items(), key=lambda item: item[1], reverse=True)
    return {
        "device_id": device_id,
        "window_hours": hours,
        "window_start": cutoff_ts,
        "window_end": end_ts,
        "total_transmissions": total,
        "transmissions_per_hour": round(total / float(hours), 2) if hours else 0,
        "causes": [{"cause": name, "count": count} for name, count in ranked],
        "alarm_episodes": alarm_episodes,
        "trends": trends,
    }
