"""Manual CLI for one live DTLS query to an AMU - proves live_read/live_status
work against real hardware, outside the MCP client. Same call mcp_server.py's
live_read/live_status tools make; this just exposes it directly for testing.

Usage (run on the server, where the operator cert + device_config.json live):

    python3 manual_live_query.py AMU_01 read_now
    python3 manual_live_query.py AMU_01 status

Endpoint is resolved the same way mcp_server.py does: device_config.json's
"endpoints" map. Override with --host/--port to bypass that lookup.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import device_live

DEFAULT_CONFIG = os.environ.get(
    "OMEGA_DEVICE_CONFIG", os.path.join(os.path.dirname(__file__), "device_config.json"))
DEFAULT_PKI_DIR = os.environ.get("OMEGA_PKI_DIR", os.path.join(os.path.dirname(__file__), "pki"))
DEFAULT_OPERATOR_CN = os.environ.get("OMEGA_OPERATOR_CN", "operator")


def _endpoint(config_path, device_id):
    with open(config_path, "r", encoding="utf-8") as handle:
        doc = json.load(handle)
    return doc.get("endpoints", {}).get(device_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device_id", help="e.g. AMU_01")
    parser.add_argument("cmd", choices=["read_now", "status"])
    parser.add_argument("--host", help="skip device_config.json lookup, query this IP directly")
    parser.add_argument("--port", type=int, default=device_live.LIVE_PORT)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--pki-dir", default=DEFAULT_PKI_DIR)
    parser.add_argument("--operator-cn", default=DEFAULT_OPERATOR_CN)
    parser.add_argument("--timeout", type=float, default=device_live.DEFAULT_TIMEOUT_S)
    args = parser.parse_args()

    host = args.host or _endpoint(args.config, args.device_id)
    if not host:
        print("ERROR: no live endpoint for %s in %s (use --host to override)"
              % (args.device_id, args.config))
        return 1

    reply = device_live.live_query(
        args.pki_dir, args.operator_cn, host, args.port,
        {"cmd": args.cmd}, timeout_s=args.timeout)
    print(json.dumps(reply, indent=2))
    return 0 if reply.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
