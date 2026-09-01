"""Simulated 'boss website' stats-receiving endpoint (Phase 2 goal 4, agreed
2026-07-30: simulate the URL until the real research-unit endpoint exists).

Stands in for the external, third-party HTTPS endpoint daily_stats.py posts
to. Not part of Omega's own server - this represents somebody else's website,
so it lives in simlab/ rather than server/ and is never part of a deploy
payload. Requires the same Bearer token daily_stats.py sends, matching the
real endpoint's expected auth shape, and appends every accepted report to a
JSON-lines file so a field test has something to inspect afterward.

Run:
    python3 boss_endpoint_sim.py [port]
Env:
    OMEGA_BOSS_SIM_TOKEN   required bearer token (default: "sim-token")
"""

import json
import os
import sys
import time

from flask import Flask, jsonify, request

HERE = os.path.dirname(os.path.abspath(__file__))
RECEIVED_LOG = os.path.join(HERE, "boss_endpoint_sim_received.jsonl")
DEFAULT_PORT = 9443
REQUIRED_FIELDS = ("report_id", "generated_at", "period_start", "period_end", "devices")

app = Flask(__name__)


def _token_ok(req):
    expected = os.environ.get("OMEGA_BOSS_SIM_TOKEN", "sim-token")
    header = req.headers.get("Authorization", "")
    return header == "Bearer " + expected


def _shape_ok(report):
    return isinstance(report, dict) and all(field in report for field in REQUIRED_FIELDS)


@app.route("/stats/ingest", methods=["POST"])
def ingest():
    if not _token_ok(request):
        return jsonify({"error": "bad or missing bearer token"}), 401

    report = request.get_json(silent=True)
    if not _shape_ok(report):
        return jsonify({"error": "malformed report - expected " + str(REQUIRED_FIELDS)}), 400

    record = {"received_at": int(time.time()), "report": report}
    with open(RECEIVED_LOG, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(record) + "\n")

    span_h = (report["period_end"] - report["period_start"]) / 3600.0
    print("boss-sim: accepted %s (%d devices, %.1fh span)"
          % (report["report_id"], len(report["devices"]), span_h))
    return jsonify({"ok": True, "report_id": report["report_id"]}), 200


@app.route("/stats/received")
def received():
    """Inspection helper for the field test - not part of the real contract."""
    if not os.path.exists(RECEIVED_LOG):
        return jsonify([])
    with open(RECEIVED_LOG, "r", encoding="utf-8") as handle:
        return jsonify([json.loads(line) for line in handle if line.strip()])


def main(argv):
    port = int(argv[1]) if len(argv) > 1 else DEFAULT_PORT
    print("boss-sim: listening on :%d (POST /stats/ingest, GET /stats/received)" % port)
    app.run(host="127.0.0.1", port=port, debug=False)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
