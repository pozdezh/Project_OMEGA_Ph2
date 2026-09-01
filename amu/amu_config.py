import json
import os
import sys
import configparser

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.environ.get("OMEGA_AMU_CONFIG",
                             os.path.join(BASE_DIR, "config", "global.ini"))

if not os.path.exists(CONFIG_PATH):
    print("CRITICAL: Config file not found at %s" % CONFIG_PATH)
    sys.exit(1)

_config = configparser.ConfigParser()
_config.read(CONFIG_PATH)

try:
    DEVICE_ID = _config["NETWORK"].get("device_id", "AMU_UNKNOWN")
    WIFI_SSID = _config["NETWORK"]["wifi_ssid"]
    WIFI_PASS = _config["NETWORK"]["wifi_pass"]
    UDP_PORT = int(_config["NETWORK"].get("target_port", 5000))
    SERVER_HOST = _config["NETWORK"].get("server_host", "").strip()
    PKI_DIR = _config["SECURITY"].get("pki_dir", os.path.join(BASE_DIR, "pki"))
    heartbeat_interval = int(_config["TRIGGER_THRESHOLDS"].get("alarm_heartbeat_interval", 3600))
    BUFFER_FILE = _config["PATHS"].get("buffer_file", os.path.join(BASE_DIR, "offline_buffer.json"))
    LIVE_PORT_CONFIGURED = _config["NETWORK"].get("live_port", None)
    TRIGGER_THRESHOLDS = _config["TRIGGER_THRESHOLDS"]
except KeyError as error:
    print("CRITICAL: Missing config key - %s" % error)
    sys.exit(1)

STATIC_SERVER = (SERVER_HOST, UDP_PORT) if SERVER_HOST else None

# Persisted so the LEARNED heartbeat survives a process restart. Without
# this, a crash (or the recovery ladder's OWN restart rung) wipes
# heartbeat_interval back to global.ini's static default - which is exactly
# how AMU_15 was found stuck on a stale 30-minute value left over from
# before the fleet moved to a 15-minute server heartbeat, making its
# recovery ladder run at half the intended speed for the one outage it
# exists to survive. Write-once-per-change, same reasoning as
# device_addresses.py on the server: this is not a credential, only a
# cached number the server re-confirms on every ACK.
HEARTBEAT_STATE_PATH = os.environ.get(
    "OMEGA_HEARTBEAT_STATE", os.path.join(BASE_DIR, "heartbeat_state.json"))


def _load_persisted_heartbeat_s():
    try:
        with open(HEARTBEAT_STATE_PATH, "r", encoding="utf-8") as handle:
            return int(json.load(handle)["heartbeat_s"])
    except (OSError, ValueError, KeyError, TypeError):
        return None


def persist_heartbeat_s(value_s):
    tmp = HEARTBEAT_STATE_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump({"heartbeat_s": int(value_s)}, handle)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, HEARTBEAT_STATE_PATH)


_persisted_heartbeat_s = _load_persisted_heartbeat_s()
if _persisted_heartbeat_s is not None:
    heartbeat_interval = _persisted_heartbeat_s


def apply_learned_heartbeat_s(value_s):
    """Update the in-memory heartbeat from a real server ACK, persisting it
    only when it actually changes. Returns True if it changed.

    The persistence is what matters: without it, a process restart (a crash,
    or the recovery ladder's own restart rung) falls back to global.ini's
    static default, which found AMU_15 stuck on a stale 30-minute value
    during the one outage its recovery ladder existed to survive.
    """
    global heartbeat_interval
    if value_s == heartbeat_interval:
        return False
    heartbeat_interval = value_s
    # A card that has worn read-only must not take telemetry down with it:
    # this runs inside the ACK path, so an uncaught write error would skip
    # the rest of _apply_config and cost the delivery. The in-memory value
    # is already correct for THIS process either way; only the restart
    # protection is lost, which is exactly what the log line says.
    try:
        persist_heartbeat_s(value_s)
    except OSError as error:
        print("heartbeat %ds learned but NOT persisted (%s) - a restart will "
              "fall back to the config default" % (value_s, error), flush=True)
    return True

# How often the network worker wakes to advance the recovery ladder AND to
# check whether a buffered record can go out yet. This is the granularity of
# "how live is the system after a hiccup", so it is seconds, not minutes.
RECOVERY_TICK_S = 5.0

# Retry cadence for emptying the offline buffer. Starts at the floor so a
# brief WiFi glitch costs seconds, doubles while the link stays down so a
# dead server is not hammered by the whole fleet, and snaps back to the floor
# the moment anything is acknowledged.
BACKLOG_RETRY_MIN_S = 5.0
BACKLOG_RETRY_MAX_S = 60.0

# Latest-sample cache, shared with the inbound live-query agent
# (live_agent.py) - see ARCHITECTURE.md "AMU live query".
#
# On tmpfs (RAM), never on the SD card: main.py rewrites this every sampling
# cycle, which on flash would be ~43k writes/day of pure wear for data that
# is worthless two seconds later.
LIVE_CACHE_FILE = os.environ.get("OMEGA_LIVE_CACHE",
                                 "/dev/shm/omega_amu_latest.json")

# A cached sample older than this means the sampling loop has stopped or
# wedged. The live agent still returns the data - it is real, just old - but
# marks it stale, so an operator can never mistake a dead unit's last gasp
# for a current reading. Sampling runs every 2s, so this is 15 cycles of
# slack before anything is called stale.
LIVE_CACHE_STALE_S = 30.0

# Inbound live-query listener port (live_agent.py). The AMU is mains-powered
# and always on, so unlike the NMU it can afford to accept a connection.
LIVE_PORT_DEFAULT = 5001

# Who is allowed to call this device. Only the SERVER, by its own
# certificate - not the operator certificate, even though that one would
# also verify against the same CA. Two reasons the distinction is worth the
# extra config: every machine then speaks as itself in the logs, and an
# operator credential that leaks (a laptop, the .p12 bundle) cannot be used
# to reach devices directly - it can only talk to the server, which is the
# one place authorisation and auditing already live.
LIVE_ALLOWED_CALLERS = ("omega-server",)

MAX_BUFFER_SIZE = 100
MAX_QUEUE_DEPTH = 8
MAX_MISSED_ACKS = 5
RECONNECT_BACKOFF_S = 2.0

# Spread a fleet's reconnects. When a server returns after an outage every
# unit notices within the same second and would otherwise handshake in
# lockstep. See ARCHITECTURE.md "Fleet behaviour".
RECONNECT_JITTER_S = 3.0
BOOT_JITTER_S = 30.0
FLUSH_PACING_MIN_S = 0.03
FLUSH_PACING_MAX_S = 0.12

DHT_READ_ATTEMPTS = 3
DHT_RETRY_DELAY_S = 0.3

# Self-heating coupling factors (see ARCHITECTURE.md, "AMU sensor
# calibration"). DHT22 is a satellite sensor on a cable, more airflow
# separation from the Pi - kept at the original physically-motivated
# starting value, no live evidence yet that it needs re-deriving.
# Enviro+ is a HAT plugged directly onto the Pi's header (touching the
# board) - re-derived 2026-08-21 from a live comparison against the DHT22
# on the actual deployed unit (raw BME280 ~46.8C, CPU ~62C, DHT22
# corrected ~32.9C): 2.25 under-corrected it to ~40C, physically too hot
# for the room every other sensor agreed on. 1.15 is not an independent
# reference-thermometer calibration either - re-validate when one is
# available - but it is grounded in real, simultaneous cross-sensor data
# from this unit, not reused from a different sensor's mount type.
DHT_SELF_HEAT_FACTOR = 2.25
ENV_SELF_HEAT_FACTOR = 1.15

# Sensirion SCD30 interface description section 1.3.8: must be found
# empirically in continuous operation, not guessed - see ARCHITECTURE.md
# for the re-validation procedure. Unit: degrees C.
SCD30_TEMP_OFFSET_C = 9.0

# Sensirion SCD30 interface description section 1.3.1 / the installed
# scd30_i2c library's own validation: ambient pressure compensation only
# accepts 0 (off) or this range, in mBar.
SCD30_PRESSURE_MIN_MBAR = 700
SCD30_PRESSURE_MAX_MBAR = 1400
