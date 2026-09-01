"""A pending-question mailbox for devices that never accept inbound
connections (the NMU - see SYSTEM_VISION.md's session model).

The AMU is always-on and can be called directly (device_live.py). The NMU
cannot: calling it would mean keeping a listening socket open around the
clock for a question that might arrive once a week, which is the opposite of
"a device that only speaks when it has something to say". Instead the
question rides the channel that already exists in both directions on every
contact - the ACK going out, and the device's own next record coming back:

    1. An operator queues a question (queue_query) - nothing is sent yet.
    2. The NMU's NEXT ACK (whenever it next transmits - could be seconds or
       hours away) piggybacks that question (acks.py pops it via
       pop_pending_query, exactly once).
    3. The firmware answers on ITS OWN next outgoing record - not a new
       connection, the very next thing it was already going to send.
    4. session.py notices the reply field and stores it (deliver_reply).
    5. The operator's tool call polls (poll_reply) up to a bounded timeout.

Backed by SQLite, not a JSON file: this mailbox is written from TWO different
OS processes (the listener, and whatever runs the MCP tool) plus multiple
per-device threads within the listener itself. SQLite's own locking makes
that safe for free; a hand-rolled JSON read-modify-write would not be.
"""

import json
import sqlite3
import time

POLL_INTERVAL_S = 1.0


def _connect(db_path):
    conn = sqlite3.connect(db_path, timeout=30)
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute(
        "CREATE TABLE IF NOT EXISTS mailbox ("
        "device_id TEXT PRIMARY KEY, "
        "pending_cmd TEXT, pending_queued_at INTEGER, "
        "reply_json TEXT, reply_at INTEGER)")
    return conn


def queue_query(db_path, device_id, command):
    """Stage a question for device_id's next contact. Replaces any question
    already waiting (an unanswered old question is stale, not queued twice)
    and clears any leftover reply from a previous round."""
    conn = _connect(db_path)
    try:
        conn.execute(
            "INSERT INTO mailbox (device_id, pending_cmd, pending_queued_at, "
            "reply_json, reply_at) VALUES (?, ?, ?, NULL, NULL) "
            "ON CONFLICT(device_id) DO UPDATE SET "
            "pending_cmd=excluded.pending_cmd, "
            "pending_queued_at=excluded.pending_queued_at, "
            "reply_json=NULL, reply_at=NULL",
            (device_id, json.dumps(command), int(time.time())))
        conn.commit()
    finally:
        conn.close()


def pop_pending_query(db_path, device_id):
    """Called from acks.py while building one ACK. Claims and clears the
    pending question so it rides on this ACK exactly once - a device that
    sends two records before the operator notices a reply must not be asked
    the same question twice."""
    conn = _connect(db_path)
    try:
        row = conn.execute(
            "SELECT pending_cmd FROM mailbox WHERE device_id = ? AND pending_cmd IS NOT NULL",
            (device_id,)).fetchone()
        if row is None:
            return None
        conn.execute(
            "UPDATE mailbox SET pending_cmd = NULL WHERE device_id = ?", (device_id,))
        conn.commit()
        return json.loads(row[0])
    finally:
        conn.close()


def deliver_reply(db_path, device_id, reply):
    """Called from session.py when a device's record carries an answer."""
    conn = _connect(db_path)
    try:
        conn.execute(
            "INSERT INTO mailbox (device_id, pending_cmd, pending_queued_at, "
            "reply_json, reply_at) VALUES (?, NULL, NULL, ?, ?) "
            "ON CONFLICT(device_id) DO UPDATE SET "
            "reply_json=excluded.reply_json, reply_at=excluded.reply_at",
            (device_id, json.dumps(reply), int(time.time())))
        conn.commit()
    finally:
        conn.close()


def poll_reply(db_path, device_id, timeout_s, interval_s=POLL_INTERVAL_S, sleep=time.sleep):
    """Wait up to timeout_s for a reply to appear, checking every interval_s.
    Claims (clears) the reply on the way out, so a second poll never re-reads
    a stale answer from a previous question. Returns the reply dict, or None
    if the device has not answered within the window - the device may still
    answer later; the question itself is not cancelled by a poll timing out."""
    deadline = time.time() + timeout_s
    while True:
        conn = _connect(db_path)
        try:
            row = conn.execute(
                "SELECT reply_json, reply_at FROM mailbox WHERE device_id = ? AND reply_json IS NOT NULL",
                (device_id,)).fetchone()
            if row is not None:
                conn.execute(
                    "UPDATE mailbox SET reply_json = NULL, reply_at = NULL "
                    "WHERE device_id = ?", (device_id,))
                conn.commit()
                reply = json.loads(row[0])
                reply["answered_at"] = row[1]
                return reply
        finally:
            conn.close()
        if time.time() >= deadline:
            return None
        sleep(interval_s)
