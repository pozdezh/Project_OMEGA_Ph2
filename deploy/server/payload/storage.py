"""Schema, writes, dedup, and the db worker thread. Must never know about
sockets - session.py hands this module an authenticated device_id and the
decrypted inner JSON; this module never touches crypto and can therefore be
unit-tested on any platform with no sockets.
"""

import sqlite3
import time

import cause_validation

NOISE_TYPE = "noise"
AIR_TYPES = ("air", "airq")


def open_db(db_file):
    conn = sqlite3.connect(db_file, timeout=10)
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute(
        "CREATE TABLE IF NOT EXISTS noise_data ("
        "id TEXT, timestamp INTEGER, event TEXT, db REAL, duration REAL, heartbeat INTEGER)")
    conn.execute("CREATE UNIQUE INDEX IF NOT EXISTS idx_noise_event_unique ON noise_data (id, event)")
    conn.execute(
        "CREATE TABLE IF NOT EXISTS air_data ("
        "id TEXT, timestamp INTEGER, event TEXT, heartbeat INTEGER, "
        "scd_co2 REAL, scd_temp REAL, scd_hum REAL, dht_temp REAL, dht_hum REAL, "
        "pms_pm1 REAL, pms_pm25 REAL, pms_pm10 REAL, env_temp REAL, env_hum REAL, "
        "env_press REAL, env_lux REAL, cause TEXT)")
    conn.execute("CREATE UNIQUE INDEX IF NOT EXISTS idx_air_event_unique ON air_data (id, event)")
    # See ARCHITECTURE.md 21.
    conn.execute("CREATE INDEX IF NOT EXISTS idx_noise_hb_ts ON noise_data (heartbeat, timestamp)")
    conn.execute("CREATE INDEX IF NOT EXISTS idx_noise_id_ts ON noise_data (id, timestamp)")
    conn.execute("CREATE INDEX IF NOT EXISTS idx_air_hb_ts ON air_data (heartbeat, timestamp)")
    conn.execute("CREATE INDEX IF NOT EXISTS idx_air_id_ts ON air_data (id, timestamp)")
    conn.commit()
    return conn, conn.cursor()


def is_duplicate(cursor, table, device_id, event_id):
    if not event_id:
        return False
    cursor.execute("SELECT 1 FROM " + table + " WHERE id = ? AND event = ?", (device_id, event_id))
    return cursor.fetchone() is not None


def store_noise_row(cursor, payload, device_id, ts, event_id, heartbeat_val, is_heartbeat):
    if is_duplicate(cursor, "noise_data", device_id, event_id):
        if not is_heartbeat:
            print("DUPLICATE noise " + str(event_id) + " " + device_id)
        return
    cursor.execute(
        "INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
        (device_id, ts, event_id, payload.get("db", 0.0), payload.get("duration", 0.0), heartbeat_val))
    if not is_heartbeat:
        print("NOISE logged " + device_id + " event " + str(event_id))


def store_air_row(cursor, payload, device_id, ts, event_id, heartbeat_val, is_heartbeat):
    if is_duplicate(cursor, "air_data", device_id, event_id):
        if not is_heartbeat:
            print("DUPLICATE airq " + str(event_id) + " " + device_id)
        return
    sensors = payload.get("sensors", {})
    scd = sensors.get("scd30", {})
    dht = sensors.get("dht22", {})
    pms = sensors.get("pms5003", {})
    envs = sensors.get("enviro_plus", {})
    cursor.execute(
        "INSERT INTO air_data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (device_id, ts, event_id, heartbeat_val,
         scd.get("co2_ppm"), scd.get("temperature_c"), scd.get("humidity_pct"),
         dht.get("temperature_c"), dht.get("humidity_pct"),
         pms.get("pm1_0"), pms.get("pm2_5"), pms.get("pm10_0"),
         envs.get("temperature_c"), envs.get("humidity_pct"),
         envs.get("pressure_hpa"), envs.get("light_lux"),
         cause_validation.sanitize_cause(payload.get("cause", "Unknown"))))
    if not is_heartbeat:
        print("AIRQ logged " + device_id + " event " + str(event_id))


def db_worker(db_file, packet_queue):
    conn, c = open_db(db_file)
    while True:
        payload = packet_queue.get()
        if payload is None:
            packet_queue.task_done()
            break
        device_id = payload.get("id", "UNKNOWN")
        ev_type = payload.get("type", "unknown")
        ts = payload.get("ts", 0) or int(time.time())
        is_heartbeat = payload.get("hb", False)
        heartbeat_val = 1 if is_heartbeat else 0
        event_id = str(payload.get("event")) if payload.get("event") else None
        try:
            if ev_type == NOISE_TYPE:
                store_noise_row(c, payload, device_id, ts, event_id, heartbeat_val, is_heartbeat)
            elif ev_type in AIR_TYPES:
                store_air_row(c, payload, device_id, ts, event_id, heartbeat_val, is_heartbeat)
            conn.commit()
        except sqlite3.OperationalError as error:
            print("DB worker reconnecting after: " + str(error))
            try:
                conn.close()
            except Exception:
                pass
            conn, c = open_db(db_file)
        except Exception as error:
            print("DB worker error: " + str(error))
            try:
                conn.rollback()
            except Exception:
                pass
        packet_queue.task_done()


def ingest_telemetry(packet_queue, device_id, inner):
    """Queue one authenticated telemetry record for storage. The identity is
    forced to the DTLS-verified device_id, so a device cannot claim to be
    another even if it lies about the inner id field."""
    payload = dict(inner)
    payload["id"] = device_id
    payload["hb"] = bool(inner.get("hb", False))
    packet_queue.put(payload)
