# Superseded files - not part of the running system

Nothing here is imported, deployed, or executed. Kept only because these
files are not in git history and deleting them would lose them.

## sensor_manager_dtls.py

The **Brick 3** AMU monolith (its own docstring says so). Brick 4 replaced it
with a module split, and every function it contains has a live equivalent:

| in the monolith | now lives in |
|---|---|
| `TriggerEngine`, alarm/heartbeat policy | `amu/triggers.py` |
| `read_dht22`, `read_all_sensors`, `get_cpu_temperature` | `amu/sensors.py` |
| `load_buffer`, `save_buffer`, `append_to_buffer` | `amu/buffer.py` |
| `ensure_session`, `deliver`, `apply_config`, `to_record`, `mark_ack_failure`, `connect_wifi`, `hard_network_reset` | `amu/network.py` |
| `network_worker`, `send_data`, `enqueue_newest_wins` | `amu/main.py` |
| `_live_read` | `amu/live_agent.py` + `amu/live_cache.py` |

It was removed from `amu/` on 2026-08-21 because it sat beside the real
entry point and read like one - see FINDINGS #40.7. The split versions carry
fixes this file never received (atomic buffer operations, alarm hysteresis,
the wolfSSL lock discipline), so it must never be run.
