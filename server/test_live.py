"""Unit tests for the AMU live command handler (deterministic dispatch).

Proves the fixed command vocabulary and the MCP endpoint resolution without any
sockets. The encrypted operator<->AMU round-trip and rogue-operator refusal are
proven in the in-memory SimLab.
"""

import os
import sys
import tempfile
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "amu"))

import live_server


def _fake_read():
    return {"scd30": {"co2_ppm": 640.0, "temperature_c": 22.0, "humidity_pct": 41.0}}


def test_read_now():
    reply = live_server.handle_command({"cmd": "read_now"}, "AMU_01", _fake_read)
    assert reply["ok"] and reply["device_id"] == "AMU_01", str(reply)
    assert reply["reading"]["scd30"]["co2_ppm"] == 640.0, "returns the live reading"
    print("PASS live read_now")


def test_status():
    reply = live_server.handle_command({"cmd": "status"}, "AMU_02", _fake_read)
    assert reply["ok"] and reply["status"] == "online" and reply["device_id"] == "AMU_02"
    print("PASS live status")


def test_unknown_command_is_explicit_error():
    reply = live_server.handle_command({"cmd": "reboot"}, "AMU_01", _fake_read)
    assert reply["ok"] is False and "unknown command" in reply["error"], str(reply)
    # a deterministic vocabulary: an out-of-band instruction is refused, not guessed
    reply2 = live_server.handle_command({"cmd": "please turn everything off"}, "AMU_01", _fake_read)
    assert reply2["ok"] is False
    print("PASS unknown command refused (fixed vocabulary)")


def test_mcp_endpoint_resolution():
    # The AMU-hostname lookup lives server-side now (app.py's /api/mcp/live
    # route calls load_device_config() directly) - see FINDINGS #24. Proves
    # the same resolution behaviour at its new home.
    import app
    tmp = tempfile.mkdtemp(prefix="omega_live_")
    cfg = os.path.join(tmp, "device_config.json")
    with open(cfg, "w", encoding="utf-8") as handle:
        json.dump({"endpoints": {"AMU_01": "10.0.0.5"}}, handle)
    app.DEVICE_CONFIG_FILE = cfg
    endpoints = app.load_device_config().get("endpoints", {})
    assert endpoints.get("AMU_01") == "10.0.0.5", "endpoint resolves"
    assert endpoints.get("AMU_99") is None, "unknown endpoint -> None"
    print("PASS MCP endpoint resolution")


def main():
    test_read_now()
    test_status()
    test_unknown_command_is_explicit_error()
    test_mcp_endpoint_resolution()
    print("RESULT: PASS - live command handler + MCP resolution verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
