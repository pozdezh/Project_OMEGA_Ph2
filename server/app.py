"""Brick 4 dashboard - read-only telemetry views plus device revoke/restore.

Owns the dashboard and its API only - never business rules. Identity for the
revoke/restore actions comes from nginx's mutual-TLS gate in front of this
app (only an operator holding a CA-signed certificate reaches it at all), so
this module does not re-implement its own auth layer on top of that.

Same sqlite schema as storage.py (this is the read side of the same
database), same device_config.json format as config_store.py (this is the
write side of the same file - the listener hot-reloads on the mtime change,
so a revoke here takes effect on the device's next record with no restart).
"""

import json
import os
import sqlite3
import threading
import time
from flask import Flask, render_template, jsonify, request, Response

import config_store
import device_addresses
import device_api
import device_live
import nmu_mailbox

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(BASE_DIR, "templates")
DB_FILE = os.environ.get("OMEGA_DB", os.path.join(BASE_DIR, "sensor_data.db"))
# Addresses the listener learned from each device's own handshakes.
# Must match listener.py's OMEGA_ADDRESS_DB - they share one file, one
# writing it and the other reading it.
ADDRESS_DB_PATH = os.environ.get(
    "OMEGA_ADDRESS_DB", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "device_addresses.json"))
DEVICE_CONFIG_FILE = os.environ.get(
    "OMEGA_DEVICE_CONFIG", os.path.join(BASE_DIR, "device_config.json"))
MAILBOX_DB_FILE = os.environ.get(
    "OMEGA_MAILBOX_DB", os.path.join(BASE_DIR, "nmu_mailbox.db"))
NMU_QUERY_TIMEOUT_S = float(os.environ.get("OMEGA_NMU_QUERY_TIMEOUT_S", "90"))
# Reaching an AMU directly. Restored 2026-08-21 after the sponsors'
# event-driven requirement was confirmed: the AMU transmits only when a
# trigger fires, so on a quiet site there is no ACK for a question to ride
# down on and the only way to reach it is to call it. The AMU now runs its
# inbound listener as its OWN PROCESS (live_agent.py), which removes the
# two-wolfSSL-objects-in-one-process defect that made the previous inbound
# attempt segfault - see FINDINGS #32 and #35.
LIVE_PKI_DIR = os.environ.get(
    "OMEGA_LIVE_PKI_DIR", os.path.join(BASE_DIR, "pki_provisioning"))
# The server calls devices as ITSELF ("omega-server"), not as the operator.
# Both certificates verify against the same CA, so either would connect - but
# the device's allowlist names this one specifically, so a leaked operator
# credential cannot reach devices directly. It can only reach this server,
# where authorisation and logging already are.
LIVE_CALLER_CN = os.environ.get("OMEGA_LIVE_CALLER_CN", "omega-server")
LIVE_PORT = int(os.environ.get("OMEGA_LIVE_PORT", "5001"))
# Hard wall-clock budget for the direct attempt, enforced by abandoning the
# worker thread - NOT by device_live's own timeout_s, which cannot bound the
# handshake. wolfSSL overwrites SO_RCVTIMEO during wrap_socket() (FINDINGS
# #3), so a handshake against a device whose listener has just died can block
# for over two minutes; measured at 130s on 2026-08-21. A live query must
# never inherit that.
LIVE_DIRECT_TIMEOUT_S = float(os.environ.get("OMEGA_LIVE_DIRECT_TIMEOUT_S", "5"))
# Budget for the queued fallback: how long to wait for the device to transmit
# something the answer can ride back on. Most of the time an AMU triggers
# within seconds; the guaranteed bound is its heartbeat.
LIVE_QUEUED_TIMEOUT_S = float(os.environ.get("OMEGA_LIVE_QUEUED_TIMEOUT_S", "30"))
# Window the graphs show is client-selectable (see _window_param) - the NMU
# can trigger several times a minute, so a fixed 24h default packed the
# chart with overlapping points. Clamped so a bad/missing param can't turn
# into an unbounded query.
DEFAULT_WINDOW_S = 60 * 60
MIN_WINDOW_S = 5 * 60
MAX_WINDOW_S = 7 * 24 * 60 * 60
WINDOW_ROW_CAP = 5000
STREAM_POLL_S = 1.0

app = Flask(__name__, template_folder=TEMPLATE_DIR)


def _window_param():
    raw = request.args.get("window_s", DEFAULT_WINDOW_S)
    try:
        window_s = int(raw)
    except (TypeError, ValueError):
        window_s = DEFAULT_WINDOW_S
    return max(MIN_WINDOW_S, min(MAX_WINDOW_S, window_s))


def _row_count_since(table, cutoff):
    row = query_db("SELECT COUNT(*) AS c FROM " + table +
                   " WHERE heartbeat = 0 AND timestamp >= ?", (cutoff,), one=True)
    return row["c"] if row else 0


def _distinct_devices_since(table, cutoff):
    row = query_db("SELECT COUNT(DISTINCT id) AS n FROM " + table +
                   " WHERE heartbeat = 0 AND timestamp >= ?", (cutoff,), one=True)
    return max(1, row["n"] if row and row["n"] else 1)


def bucket_seconds(window_s, device_count=1):
    """Seconds per downsample bucket, sized so the WHOLE fleet fits the budget.

    WINDOW_ROW_CAP is a browser budget, not a per-device one: every device
    contributes one point per bucket, so a bucket chosen for a single unit
    returns device_count times too many points at fleet size. Ceiling division
    both ways - flooring either one overshoots the cap by a bucket per device,
    which at 40 devices is 40 points past a limit that exists to stop the tab
    from dying.
    """
    per_device = max(1, WINDOW_ROW_CAP // max(1, device_count))
    return max(1, -(-window_s // per_device))


# The downsampled chart query must read every row in the window to find each
# bucket's peak - measured 2.0s for a 24h window over a 40-unit fleet's
# 1.15M rows (simlab/fleet_load.py). Nothing indexable removes that: the cost
# is the reading, not the finding. So it is computed at most once per
# CHART_CACHE_TTL_S no matter how many browsers ask, and liveness comes from
# /api/stream, which is rowid-keyed and cheap. The cache therefore only ever
# ages the LEFT edge of a long window, which is history and does not move.
CHART_CACHE_TTL_S = 30.0
# Wide windows are mostly history, and history does not change - so the wider
# the window, the longer its result stays good. This is what keeps the cost
# flat as the database grows: a 7-day window over a full 500 MB database reads
# several million rows, and it must not do that every thirty seconds.
CHART_CACHE_WIDE_WINDOW_S = 6 * 60 * 60
CHART_CACHE_WIDE_TTL_S = 300.0
_chart_cache = {}
_chart_cache_lock = threading.Lock()


def _chart_cache_ttl(window_s):
    if window_s >= CHART_CACHE_WIDE_WINDOW_S:
        return CHART_CACHE_WIDE_TTL_S
    return CHART_CACHE_TTL_S


def _cached_rows(key, builder):
    window_s = key[1]
    now = time.time()
    with _chart_cache_lock:
        entry = _chart_cache.get(key)
        if entry is not None and now - entry[0] < _chart_cache_ttl(window_s):
            return entry[1]
    rows = builder()
    with _chart_cache_lock:
        _chart_cache[key] = (now, rows)
    return rows


KEYS_CACHE_TTL_S = 10.0


def _last_seen_by_device():
    seen = {}
    for table in ("noise_data", "air_data"):
        for row in query_db("SELECT id, MAX(timestamp) AS last_seen FROM " +
                            table + " GROUP BY id"):
            device_id = row["id"]
            last = row["last_seen"] or 0
            if last > seen.get(device_id, 0):
                seen[device_id] = last
    return seen


def load_device_config():
    if not os.path.exists(DEVICE_CONFIG_FILE):
        return {}
    try:
        with open(DEVICE_CONFIG_FILE, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError) as error:
        print("Device config read error: " + str(error))
        return {}


DEVICE_CONFIG_LOCK = threading.Lock()


def save_device_config(config):
    config_store.write_atomic(DEVICE_CONFIG_FILE, config)


def query_db(query, args=(), one=False):
    if not os.path.exists(DB_FILE):
        return []
    try:
        conn = sqlite3.connect(DB_FILE, timeout=10)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        cursor.execute(query, args)
        rows = cursor.fetchall()
        conn.close()
        return (rows[0] if rows else None) if one else rows
    except Exception as error:
        print("DB error: " + str(error))
        return []


@app.route("/")
def index():
    return render_template("index.html")


def row_to_noise(row):
    # rowid rides along so the browser can tell "already have this one" apart
    # from "new" - see the dashboard's dedup note for why that matters even
    # though the database itself never stores a true duplicate row.
    return {"rowid": row["rowid"], "id": row["id"], "timestamp": row["timestamp"],
            "db_level": row["db"], "duration": row["duration"]}


def row_to_air(row):
    item = dict(row)
    return {
        "rowid": item.get("rowid"),
        "id": item.get("id"), "timestamp": item.get("timestamp"),
        "co2": item.get("scd_co2"),
        "scd_temp": item.get("scd_temp"), "scd_hum": item.get("scd_hum"),
        "dht_temp": item.get("dht_temp"), "dht_hum": item.get("dht_hum"),
        "pm1": item.get("pms_pm1"), "pm25": item.get("pms_pm25"),
        "pm10": item.get("pms_pm10"),
        "env_temp": item.get("env_temp"), "env_hum": item.get("env_hum"),
        "env_press": item.get("env_press"), "lux": item.get("env_lux"),
        "cause": item.get("cause", "Unknown"),
    }


def _max_rowid(table):
    row = query_db("SELECT MAX(rowid) AS m FROM " + table, one=True)
    return row["m"] if row and row["m"] is not None else 0


@app.route("/api/noise/latest")
def get_noise_data():
    # duration included so the dashboard tooltip can show it alongside dB -
    # both were already stored, the API just wasn't asked for the second one.
    # A busy window holds far more rows than a browser can plot, so past the
    # cap it is downsampled to the LOUDEST real row per time bucket rather
    # than truncated - see ARCHITECTURE.md 15.
    window_s = _window_param()
    fields = "rowid, id, timestamp, db, duration"

    def build():
        cutoff = int(time.time()) - window_s
        if _row_count_since("noise_data", cutoff) <= WINDOW_ROW_CAP:
            return query_db("SELECT " + fields + " FROM noise_data "
                            "WHERE heartbeat = 0 AND timestamp >= ? "
                            "ORDER BY timestamp ASC", (cutoff,))
        # id in the GROUP BY: without it the fleet collapses into one row per
        # bucket and SQLite returns an arbitrary device, so 39 of 40 units
        # vanish from the chart. See server/test_dashboard_scale.py.
        bucket = bucket_seconds(window_s, _distinct_devices_since("noise_data", cutoff))
        return query_db("SELECT " + fields + ", MAX(db) AS peak FROM noise_data "
                        "WHERE heartbeat = 0 AND timestamp >= ? "
                        "GROUP BY id, timestamp / ? ORDER BY timestamp ASC",
                        (cutoff, bucket))

    rows = _cached_rows(("noise", window_s), build)
    return jsonify([row_to_noise(row) for row in rows])


@app.route("/api/air/latest")
def get_air_data():
    # Every stored variable, not a subset - the tooltip is meant to show
    # everything a record carries, not just the ones charted at a glance.
    # Downsampled past the cap on peak CO2, the safety-relevant value, the
    # same way the noise chart uses peak dB - see ARCHITECTURE.md 15.
    window_s = _window_param()
    fields = ("rowid, id, timestamp, scd_co2, scd_temp, scd_hum, dht_temp, dht_hum, "
              "pms_pm1, pms_pm25, pms_pm10, env_temp, env_hum, env_press, env_lux, cause")

    def build():
        cutoff = int(time.time()) - window_s
        if _row_count_since("air_data", cutoff) <= WINDOW_ROW_CAP:
            return query_db("SELECT " + fields + " FROM air_data "
                            "WHERE heartbeat = 0 AND timestamp >= ? "
                            "ORDER BY timestamp ASC", (cutoff,))
        bucket = bucket_seconds(window_s, _distinct_devices_since("air_data", cutoff))
        return query_db("SELECT " + fields + ", MAX(scd_co2) AS peak FROM air_data "
                        "WHERE heartbeat = 0 AND timestamp >= ? "
                        "GROUP BY id, timestamp / ? ORDER BY timestamp ASC",
                        (cutoff, bucket))

    rows = _cached_rows(("air", window_s), build)
    return jsonify([row_to_air(row) for row in rows])


@app.route("/api/stream")
def stream():
    # Push, not poll: the browser holds one open connection and gets each
    # new row the moment it's committed, instead of re-asking on a timer.
    # This process (app.py) never talks to the listener directly - it has
    # no reach into that process's memory - so "push" here means this
    # generator polls its OWN sqlite connection on a short internal timer
    # and only forwards a message when rowid actually advances. That inner
    # poll is invisible to the browser, which just sees an event arrive.
    def generate():
        last_noise = _max_rowid("noise_data")
        last_air = _max_rowid("air_data")
        while True:
            time.sleep(STREAM_POLL_S)
            try:
                new_noise = query_db(
                    "SELECT rowid, id, timestamp, db, duration FROM noise_data "
                    "WHERE heartbeat = 0 AND rowid > ? ORDER BY rowid ASC",
                    (last_noise,))
                if new_noise:
                    last_noise = new_noise[-1]["rowid"]
                    yield "event: noise\ndata: " + json.dumps(
                        [row_to_noise(r) for r in new_noise]) + "\n\n"

                new_air = query_db(
                    "SELECT rowid, id, timestamp, scd_co2, scd_temp, scd_hum, "
                    "dht_temp, dht_hum, pms_pm1, pms_pm25, pms_pm10, env_temp, "
                    "env_hum, env_press, env_lux, cause FROM air_data "
                    "WHERE heartbeat = 0 AND rowid > ? ORDER BY rowid ASC",
                    (last_air,))
                if new_air:
                    last_air = new_air[-1]["rowid"]
                    yield "event: air\ndata: " + json.dumps(
                        [row_to_air(r) for r in new_air]) + "\n\n"

                yield ": keep-alive\n\n"
            except GeneratorExit:
                raise
            except Exception as error:
                print("SSE stream error: " + str(error))
                break

    return Response(generate(), mimetype="text/event-stream", headers={
        "Cache-Control": "no-cache",
        # nginx buffers proxied responses by default, which would hold every
        # event until the buffer filled - defeating the entire point of a
        # push stream. This header is the documented nginx opt-out; the
        # matching proxy_buffering off is also set server-side in
        # install_server.sh's /api/stream location block (belt and braces -
        # either alone is enough, but a future nginx config change should
        # not silently reintroduce buffering).
        "X-Accel-Buffering": "no",
    })


@app.route("/api/keys")
def get_keys():
    # last_seen is a MAX(timestamp) GROUP BY id over both whole tables, and the
    # panel polls every 5s per open browser - 156ms per call at fleet volume.
    # Cached; the revocation list is NOT, so clicking Revoke still shows its
    # effect immediately while a last-seen clock may lag by a few seconds.
    seen = _cached_rows(("keys", KEYS_CACHE_TTL_S), _last_seen_by_device)
    revoked = set(load_device_config().get("revoked", []))
    devices = sorted(set(seen) | revoked)
    return jsonify([{"id": device_id, "last_seen": seen.get(device_id),
                     "revoked": device_id in revoked} for device_id in devices])


@app.route("/api/keys/revoke", methods=["POST"])
@app.route("/api/keys/restore", methods=["POST"])
def set_revocation():
    device_id = (request.get_json(silent=True) or {}).get("id", "")
    if not device_id:
        return jsonify({"error": "missing device id"}), 400
    with DEVICE_CONFIG_LOCK:
        config = load_device_config()
        revoked = set(config.get("revoked", []))
        if request.path.endswith("/revoke"):
            revoked.add(device_id)
        else:
            revoked.discard(device_id)
        config["revoked"] = sorted(revoked)
        save_device_config(config)
    print("Key admin: " + device_id + " revoked=" + str(device_id in revoked))
    return jsonify({"id": device_id, "revoked": device_id in revoked})


@app.route("/api/config/heartbeat", methods=["GET"])
def get_heartbeat():
    config = load_device_config()
    return jsonify({"nmu_hb": config.get("nmu", {}).get("hb"),
                    "amu_hb": config.get("amu", {}).get("hb")})


@app.route("/api/config/heartbeat", methods=["POST"])
def set_heartbeat():
    # Written into device_config.json; the listener hot-reloads it and
    # delivers the new value to each device inside its next authenticated
    # ACK (piggyback) - no restart, no per-device push. Blank field leaves
    # that device type unchanged.
    body = request.get_json(silent=True) or {}
    changed = {}
    with DEVICE_CONFIG_LOCK:
        config = load_device_config()
        for kind in ("nmu", "amu"):
            raw = body.get(kind + "_hb")
            if raw is None or str(raw).strip() == "":
                continue
            try:
                hb = int(raw)
            except (TypeError, ValueError):
                return jsonify({"error": kind + "_hb must be an integer number of minutes"}), 400
            if hb < 1:
                return jsonify({"error": kind + "_hb must be >= 1 minute"}), 400
            block = dict(config.get(kind, {}))
            block["hb"] = hb
            block["cfg_ver"] = int(block.get("cfg_ver", 0)) + 1
            config[kind] = block
            changed[kind + "_hb"] = hb
        if not changed:
            return jsonify({"error": "provide nmu_hb and/or amu_hb"}), 400
        save_device_config(config)
    print("Heartbeat push queued: " + str(changed) + " (delivered on each device's next ACK)")
    return jsonify({"applied": changed})


@app.route("/api/mcp/devices")
def mcp_list_devices():
    return jsonify(device_api.list_devices(DB_FILE))


@app.route("/api/mcp/latest/<device_id>")
def mcp_latest_reading(device_id):
    return jsonify(device_api.latest_reading(DB_FILE, device_id) or {})


@app.route("/api/mcp/stats/<device_id>")
def mcp_device_stats(device_id):
    hours = request.args.get("hours", device_api.DEFAULT_STATS_WINDOW_H, type=int)
    return jsonify(device_api.device_stats(DB_FILE, device_id, hours))


@app.route("/api/mcp/activity/<device_id>")
def mcp_activity_report(device_id):
    hours = request.args.get("hours", device_api.DEFAULT_STATS_WINDOW_H, type=int)
    return jsonify(device_api.activity_report(DB_FILE, device_id, hours))


def _try_direct(device_id, host, cmd):
    """Attempt one direct call to the device, abandoned after a hard deadline.

    Returns (reply, error_text); reply is None if the attempt did not produce
    an answer in time.

    The deadline is enforced HERE, by not waiting on the worker past
    LIVE_DIRECT_TIMEOUT_S, rather than by device_live's own timeout_s. That
    parameter cannot bound the handshake: wolfSSL overwrites the socket
    receive timeout during wrap_socket() (FINDINGS #3), so a handshake against
    a device whose live agent has just segfaulted blocks far longer - measured
    at 130s. Waiting on it would make the fallback pointless, because the
    caller would already have given up.

    An overrunning worker is left to finish on its own. It is a daemon thread
    that closes its own socket in a finally block, it holds no lock and
    touches no shared state, and live queries are rare enough that at most a
    couple can ever be in flight."""
    if not host:
        return None, ("no live endpoint known for %s - add it to "
                      "device_config.json under \"endpoints\", or wait for it "
                      "to report once so its address is learned" % device_id)

    outcome = {}

    def attempt():
        try:
            outcome["reply"] = device_live.live_query(
                LIVE_PKI_DIR, LIVE_CALLER_CN, host, LIVE_PORT, {"cmd": cmd},
                timeout_s=LIVE_DIRECT_TIMEOUT_S)
        except Exception as error:
            outcome["error"] = str(error)

    worker = threading.Thread(target=attempt, daemon=True)
    worker.start()
    worker.join(LIVE_DIRECT_TIMEOUT_S)

    if worker.is_alive():
        return None, ("direct call to %s did not complete within %.0fs"
                      % (host, LIVE_DIRECT_TIMEOUT_S))
    if "reply" in outcome:
        return outcome["reply"], None
    return None, outcome.get("error", "direct call failed")


@app.route("/api/mcp/live/<device_id>", methods=["POST"])
def mcp_live_query(device_id):
    """Call an always-on AMU directly and return its most recent reading.

    This opens a fresh mutual-authenticated DTLS 1.3 session straight to the
    device's live agent, bypassing the store-and-forward path entirely. It is
    the only way to reach an AMU that has nothing to report: the device is
    event-driven by requirement, so a quiet unit transmits nothing and there
    is no ACK for a queued question to travel on.

    The device answers from the sample it refreshes every 2s rather than
    sampling on demand, and reports that sample's age, so a wedged sampling
    loop shows up as stale data instead of masquerading as a live reading.

    The endpoint is a HOSTNAME resolved fresh on every call (mDNS - see
    FINDINGS #20), so a DHCP lease change on the AMU needs no reconfiguration
    here."""
    cmd = (request.get_json(silent=True) or {}).get("cmd", "")
    if cmd not in ("read_now", "status"):
        return jsonify({"ok": False, "error": "cmd must be read_now or status"}), 400

    # A hand-written endpoint still wins, so an operator can pin an address
    # deliberately. Otherwise use the one the device itself last called from -
    # which means a newly installed unit is reachable the moment it reports,
    # with nothing to configure, and a DHCP move fixes itself.
    host = load_device_config().get("endpoints", {}).get(device_id)
    if not host:
        host = device_addresses.lookup(ADDRESS_DB_PATH, device_id)
    direct_reply, direct_error = _try_direct(device_id, host, cmd)
    if direct_reply is not None:
        direct_reply["ok"] = direct_reply.get("ok", True)
        direct_reply["answered"] = True
        direct_reply["device_id"] = device_id
        direct_reply["via"] = "direct"
        return jsonify(direct_reply)

    # Fall through to the mechanism proven on the NMU: leave the question in
    # the mailbox, the server attaches it to the next ACK the device is
    # already waiting for, and the answer rides back on the device's next
    # transmission. Slower, but it uses only code that has been running in
    # production, and it cannot be broken by the live agent being down.
    nmu_mailbox.queue_query(MAILBOX_DB_FILE, device_id, {"cmd": cmd})
    reply = nmu_mailbox.poll_reply(MAILBOX_DB_FILE, device_id,
                                   LIVE_QUEUED_TIMEOUT_S)
    if reply is not None:
        reply["ok"] = reply.get("ok", True)
        reply["answered"] = True
        reply["device_id"] = device_id
        reply["via"] = "queued"
        reply["direct_error"] = direct_error
        return jsonify(reply)

    return jsonify({
        "ok": False,
        "answered": False,
        "queued": True,
        "device_id": device_id,
        "via": "queued",
        "direct_error": direct_error,
        "error": ("%s has not transmitted within %.0fs, so the question is "
                  "still waiting. It is queued and will be answered on the "
                  "device's next transmission - a trigger, or its heartbeat "
                  "at the latest. Ask again shortly to collect the answer."
                  % (device_id, LIVE_QUEUED_TIMEOUT_S)),
    })


@app.route("/api/mcp/nmu_ask/<device_id>", methods=["POST"])
def mcp_nmu_ask(device_id):
    cmd = (request.get_json(silent=True) or {}).get("cmd", "")
    # "reboot" restarts the unit remotely. It rides the same mailbox as the
    # read commands, so it reaches the device on its next contact and needs no
    # inbound port on the NMU. The device answers BEFORE it restarts, so an
    # answered reboot is a confirmed reboot rather than a hopeful one.
    if cmd not in ("read_now", "status", "reboot"):
        return jsonify({"error": "cmd must be read_now, status or reboot"}), 400
    nmu_mailbox.queue_query(MAILBOX_DB_FILE, device_id, {"cmd": cmd})
    reply = nmu_mailbox.poll_reply(MAILBOX_DB_FILE, device_id, NMU_QUERY_TIMEOUT_S)
    if reply is None:
        return jsonify({"answered": False})
    reply["answered"] = True
    return jsonify(reply)


if __name__ == "__main__":
    # 8081, not 8080: brick1's dashboard (a separate, untouched fallback
    # track per project policy) already owns 8080 on this same server.
    # Default host is all-interfaces so http://<server-ip>:8081 works out of
    # the box. Behind nginx (the normal deployment), set
    # OMEGA_WEB_HOST=127.0.0.1 so :8081 is reachable only through the
    # mTLS-gated proxy.
    host = os.environ.get("OMEGA_WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("OMEGA_WEB_PORT", "8081"))
    # threaded=True is required, not optional, now that /api/stream holds a
    # connection open indefinitely - without it this dev server handles one
    # request at a time and a single open dashboard tab would freeze every
    # other request (API calls, page loads) behind it.
    # debug=False means Jinja compiles every template ONCE and caches it in
    # memory, so editing templates/index.html on disk changes nothing until
    # this service restarts:
    #
    #     sudo systemctl restart omega-web
    #
    # Safe at any time - this process only serves the dashboard. The listener
    # is a separate service, so no device session drops and no reading is
    # lost. (Debug mode would re-read templates per request, but it also
    # exposes an interactive debugger on the network, which is not a trade
    # worth making on a machine holding the fleet's certificates.)
    app.run(host=host, port=port, debug=False, threaded=True)
