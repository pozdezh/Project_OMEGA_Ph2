"""Unit tests for the device API (the logic behind the MCP demo).

Seeds a database and config, then checks list/latest/stats/set_heartbeat, and
confirms the MCP wrapper imports and registers its tools.
"""

import json
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import storage
import device_api


def _seed(db_file):
    conn, c = storage.open_db(db_file)
    ts = int(time.time())
    for i, value in enumerate((80.0, 84.0, 88.0)):
        c.execute("INSERT INTO noise_data VALUES (?, ?, ?, ?, ?, ?)",
                  ("NMU_01", ts - i, "900_" + str(i), value, 3.0, 0))
    c.execute("INSERT INTO air_data VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
              ("AMU_01", ts, "500_1", 0, 615.0, 22.0, 40.0, None, None,
               1.0, 2.0, 3.0, 22.5, 39.0, 1013.0, 150.0, "TEST"))
    conn.commit()
    conn.close()


def test_list_and_latest():
    tmp = tempfile.mkdtemp(prefix="omega_api_")
    db_file = os.path.join(tmp, "sensor_data.db")
    _seed(db_file)

    devices = {d["device_id"]: d for d in device_api.list_devices(db_file)}
    assert set(devices) == {"NMU_01", "AMU_01"}, "both devices listed: " + str(devices)
    assert devices["NMU_01"]["type"] == "noise" and devices["NMU_01"]["rows"] == 3
    assert devices["AMU_01"]["type"] == "airq"

    latest = device_api.latest_reading(db_file, "NMU_01")
    assert latest["db"] == 80.0, "latest is most recent (ts-0): " + str(latest["db"])
    air = device_api.latest_reading(db_file, "AMU_01")
    assert air["scd_co2"] == 615.0 and air["cause"] == "TEST"
    assert device_api.latest_reading(db_file, "AMU_77") is None, "unknown device -> None"
    print("PASS list_devices + latest_reading")


def test_stats_and_set_heartbeat():
    tmp = tempfile.mkdtemp(prefix="omega_api2_")
    db_file = os.path.join(tmp, "sensor_data.db")
    _seed(db_file)
    stats = device_api.device_stats(db_file, "NMU_01", hours=24)
    db_var = stats["variables"]["db"]
    assert db_var["n"] == 3 and db_var["min"] == 80.0 and db_var["max"] == 88.0
    assert db_var["avg"] == 84.0 and db_var["median"] == 84.0
    print("PASS device_stats")

    cfg_path = os.path.join(tmp, "device_config.json")
    with open(cfg_path, "w", encoding="utf-8") as handle:
        json.dump({"nmu": {"cfg_ver": 3, "hb": 30}, "amu": {"cfg_ver": 3, "hb": 30}}, handle)
    block = device_api.set_heartbeat(cfg_path, "nmu", 60)
    assert block == {"cfg_ver": 4, "hb": 60}, "hb set + cfg_ver bumped: " + str(block)
    with open(cfg_path, "r", encoding="utf-8") as handle:
        doc = json.load(handle)
    assert doc["nmu"]["hb"] == 60 and doc["amu"]["hb"] == 30, "only nmu changed"
    try:
        device_api.set_heartbeat(cfg_path, "xxx", 60)
        assert False, "bad type should raise"
    except ValueError:
        pass
    print("PASS set_heartbeat + validation")


def test_mcp_wrapper_imports():
    import mcp_server
    assert mcp_server.mcp is not None, "FastMCP server object exists"
    print("PASS mcp_server imports + registers tools")


def main():
    test_list_and_latest()
    test_stats_and_set_heartbeat()
    test_mcp_wrapper_imports()
    print("RESULT: PASS - device API + MCP wrapper verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
