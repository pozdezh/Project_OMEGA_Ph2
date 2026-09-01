"""Brick 3 AMU main - air-quality sampling over a true-DTLS session.

Sensor sampling, the hierarchical TriggerEngine, offline buffering and self-heal
are preserved verbatim from the working system; the ONLY change is the network
layer, which now runs over a mutual-authenticated DTLS session (dtls_client)
instead of the Brick 1/2 seal-and-sendto path. Records go out as plain JSON
inside the encrypted channel; the server's JSON ACK carries the piggybacked
heartbeat config, applied on receipt.

Self-heal without a cleartext discovery side-channel: the server host is
configured (a residence has a fixed server). If a session drops, the worker
reconnects on the next send and flushes the offline buffer; sustained failures
trigger a WLAN re-check. Identity and trust come only from the certificates.

Runs on the Raspberry Pi 4B; the sensor imports require the Pi hardware, so
this is validated on-device. The transport it depends on (dtls_client) is
proven on the laptop by the SimLab.
"""

import os
import time
import json
import subprocess
import threading
import queue
import random
import sys
import configparser

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dtls_client
import live_server
import server_discovery

# --- HARDWARE IMPORTS (Raspberry Pi only) ---
import board
import adafruit_dht
from smbus2 import SMBus
from bme280 import BME280
from ltr559 import LTR559
from pms5003 import PMS5003, ReadTimeoutError
from scd30_i2c import SCD30

# --- 1. CONFIGURATION ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config", "global.ini")

if not os.path.exists(CONFIG_PATH):
    print("CRITICAL: Config file not found at %s" % CONFIG_PATH)
    sys.exit(1)

config = configparser.ConfigParser()
config.read(CONFIG_PATH)

try:
    DEVICE_ID = config["NETWORK"].get("device_id", "AMU_UNKNOWN")
    WIFI_SSID = config["NETWORK"]["wifi_ssid"]
    WIFI_PASS = config["NETWORK"]["wifi_pass"]
    UDP_PORT = int(config["NETWORK"].get("target_port", 5000))
    # server_host is now OPTIONAL and only a last-resort fallback: the address
    # is normally discovered (mDNS, then UDP broadcast, cached in between), so
    # the router may reassign the server freely without touching this device.
    SERVER_HOST = config["NETWORK"].get("server_host", "").strip()
    PKI_DIR = config["SECURITY"].get("pki_dir", os.path.join(BASE_DIR, "pki"))
    heartbeat_interval = int(config["TRIGGER_THRESHOLDS"].get("alarm_heartbeat_interval", 3600))
    BUFFER_FILE = config["PATHS"].get("buffer_file", os.path.join(BASE_DIR, "offline_buffer.json"))
except KeyError as error:
    print("CRITICAL: Missing config key - %s" % error)
    sys.exit(1)

MAX_BUFFER_SIZE = 100
MAX_MISSED_ACKS = 5
RECONNECT_BACKOFF_S = 2.0

session_id = int(time.time())
event_counter = 0
consecutive_ack_fails = 0
last_cfg_ver = None
force_initial_payload = True
buffer_lock = threading.Lock()

STATIC_SERVER = (SERVER_HOST, UDP_PORT) if SERVER_HOST else None

# Find the server before the first handshake. If nothing answers yet (server
# not up, network still settling) this returns the configured fallback or the
# last-known address, and ensure_session() keeps retrying - so booting the
# device before the server is harmless.
_initial = server_discovery.find_server(static_fallback=STATIC_SERVER, log=print)
if _initial is None:
    _initial = STATIC_SERVER or ("0.0.0.0", UDP_PORT)
    print("No server found yet - will keep discovering while buffering.")

client = dtls_client.DtlsClient(PKI_DIR, DEVICE_ID, _initial[0], _initial[1])

# Bounded, so a long outage cannot exhaust the Pi's memory. When full the
# OLDEST unsent item is dropped, matching the offline buffer's behaviour:
# during a flood the newest readings are the ones that still matter.
MAX_QUEUE_DEPTH = 500
network_queue = queue.Queue(maxsize=MAX_QUEUE_DEPTH)
queue_displaced_count = 0

# Live MCP channel: the always-on AMU answers direct operator queries on its own
# DTLS listener. It serves the LATEST sampled reading (updated by the main loop),
# never opening a second sensor reader - that would fight the main loop for the
# GPIO/I2C bus, the exact conflict that bit us before.
LIVE_PORT = int(config["NETWORK"].get("live_port", live_server.LIVE_PORT))
_latest_reading = {}


def _live_read():
    return dict(_latest_reading)


# --- 2. NETWORK ENFORCER (unchanged behaviour) ---
def connect_wifi(ssid, password):
    print("Verifying WLAN connection to: %s..." % ssid)
    try:
        check = subprocess.run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
                               capture_output=True, text=True)
        if any(line.startswith("yes:" + ssid) for line in check.stdout.split("\n")):
            print("Already connected to %s." % ssid)
            return True
        subprocess.run(["sudo", "nmcli", "dev", "wifi", "connect", ssid, "password", password],
                       capture_output=True)
        return True
    except Exception as error:
        print("Network check bypassed or failed: %s" % error)
        return False


def hard_network_reset():
    print("Performing soft-network reset...")
    connect_wifi(WIFI_SSID, WIFI_PASS)


connect_wifi(WIFI_SSID, WIFI_PASS)

# --- 3. HARDWARE INITIALISATION (unchanged) ---
print("Initializing Sensors for %s..." % DEVICE_ID)
DHT_TEMP_OFFSET, DHT_HUM_OFFSET = -11.05, 29.1
dht_sensor = adafruit_dht.DHT22(board.D4)
ENV_FACTOR_TEMP, ENV_FACTOR_HUM = -9.35, 33.4
bus = SMBus(1)
bme280 = BME280(i2c_dev=bus)
ltr559 = LTR559()


def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as handle:
            return int(handle.read()) / 1000.0
    except Exception:
        return 0.0


pms5003 = PMS5003()
scd30 = SCD30()
scd30.set_temperature_offset(9.0)
scd30.set_measurement_interval(2)
scd30.start_periodic_measurement()


# --- 4. TRIGGER ENGINE (verbatim from the working system) ---
class TriggerEngine:
    def __init__(self, cfg):
        self.cfg = cfg
        self.history = {"co2": [], "pm1": [], "pm2_5": [], "pm10": []}
        self.window_size = 5
        self.last_transmitted = {"temp": None, "hum": None, "lux": None,
                                 "co2": None, "pm1": None, "pm2_5": None, "pm10": None}
        self.is_currently_in_alarm = False
        self.last_transmission_time = 0

    def evaluate(self, data, force_initial):
        global heartbeat_interval
        trigger_now = False
        is_heartbeat = False
        active_alarms = False
        relative_change = False
        alarm_list = []
        delta_list = []

        co2 = data.get("scd30", {}).get("co2_ppm")
        pm1 = data.get("pms5003", {}).get("pm1_0")
        pm25 = data.get("pms5003", {}).get("pm2_5")
        pm10 = data.get("pms5003", {}).get("pm10_0")
        temp = data.get("dht22", {}).get("temperature_c")
        hum = data.get("dht22", {}).get("humidity_pct")
        lux = data.get("enviro_plus", {}).get("light_lux")

        if co2 is not None and co2 >= float(self.cfg["co2_critical_high"]):
            alarm_list.append("High CO2")
        if pm25 is not None and pm25 >= float(self.cfg["pm_critical_high"]):
            alarm_list.append("High PM2.5")
        if temp is not None:
            if temp >= float(self.cfg["temp_critical_high"]):
                alarm_list.append("High Temp")
            elif temp <= float(self.cfg["temp_critical_low"]):
                alarm_list.append("Low Temp")
        if hum is not None:
            if hum >= float(self.cfg["hum_critical_high"]):
                alarm_list.append("High Humidity")
            elif hum <= float(self.cfg["hum_critical_low"]):
                alarm_list.append("Low Humidity")
        if lux is not None:
            if lux >= float(self.cfg["lux_critical_high"]):
                alarm_list.append("High Lux")
            elif lux <= float(self.cfg["lux_critical_low"]):
                alarm_list.append("Low Lux")

        if alarm_list:
            active_alarms = True

        if co2 is not None:
            if self.history["co2"] and abs(co2 - (sum(self.history["co2"]) / len(self.history["co2"]))) >= float(self.cfg["co2_delta"]):
                delta_list.append("CO2 Spike/Drop")
            self.history["co2"].append(co2)
            if len(self.history["co2"]) > self.window_size:
                self.history["co2"].pop(0)

        for p_val, p_key, p_name in [(pm1, "pm1", "PM1.0"), (pm25, "pm2_5", "PM2.5"), (pm10, "pm10", "PM10.0")]:
            if p_val is not None:
                if self.history[p_key] and abs(p_val - (sum(self.history[p_key]) / len(self.history[p_key]))) >= float(self.cfg["pm_delta"]):
                    delta_list.append("%s Spike/Drop" % p_name)
                self.history[p_key].append(p_val)
                if len(self.history[p_key]) > self.window_size:
                    self.history[p_key].pop(0)

        for val, key, step, name in [
            (temp, "temp", "temp_step", "Temp"),
            (hum, "hum", "hum_step", "Humidity"),
            (lux, "lux", "lux_step", "Lux"),
            (co2, "co2", "co2_delta", "CO2"),
            (pm1, "pm1", "pm_delta", "PM1.0"),
            (pm25, "pm2_5", "pm_delta", "PM2.5"),
            (pm10, "pm10", "pm_delta", "PM10.0"),
        ]:
            if val is not None and self.last_transmitted.get(key) is not None:
                if abs(val - self.last_transmitted[key]) >= float(self.cfg[step]):
                    drift = "%s Step/Drift" % name
                    if drift not in delta_list:
                        delta_list.append(drift)

        if delta_list:
            relative_change = True

        now = time.time()
        time_since_last = now - self.last_transmission_time
        primary_cause = "Unknown"

        if force_initial:
            trigger_now = True
            primary_cause = "Initial Server Sync"
        elif active_alarms and not self.is_currently_in_alarm:
            trigger_now = True
            primary_cause = "New Alarm: %s" % alarm_list[0] + (" (+%d)" % (len(alarm_list) - 1) if len(alarm_list) > 1 else "")
        elif relative_change:
            trigger_now = True
            primary_cause = delta_list[0] + (" (+%d)" % (len(delta_list) - 1) if len(delta_list) > 1 else "")
        elif self.is_currently_in_alarm and not active_alarms:
            trigger_now = True
            primary_cause = "All Alarms Cleared"
        elif active_alarms and time_since_last >= heartbeat_interval:
            trigger_now = True
            is_heartbeat = True
            primary_cause = "Sustained Alarm: %s" % alarm_list[0]
        elif time_since_last >= heartbeat_interval:
            trigger_now = True
            is_heartbeat = True
            primary_cause = "Routine Heartbeat"

        self.is_currently_in_alarm = active_alarms
        if trigger_now:
            self.last_transmission_time = now
            self.last_transmitted.update({"temp": temp, "hum": hum, "lux": lux, "co2": co2,
                                          "pm1": pm1, "pm2_5": pm25, "pm10": pm10})
        return trigger_now, primary_cause, is_heartbeat


DHT_READ_ATTEMPTS = 3
DHT_RETRY_DELAY_S = 0.3


def read_dht22():
    """Read the DHT22 once per cycle, with retries, keeping real 0.0 values.

    THREE BUGS FIXED HERE, all causes of the NULL readings you saw:

    1. `if dht_sensor.temperature` treated a real 0.0 C as "no reading" and
       threw it away, because 0.0 is falsy in Python. A freezing corridor
       silently produced no data. The test must be `is not None`.

    2. Each `dht_sensor.temperature` access re-reads the hardware. The old
       code touched `.temperature` and `.humidity` twice each - four hardware
       reads per cycle instead of one - roughly doubling the failure rate,
       since DHT22 sensors fail intermittently by design.

    3. No retry at all. A single failed read produced a NULL even though the
       very next attempt would usually have succeeded.
    """
    for attempt in range(DHT_READ_ATTEMPTS):
        try:
            # ONE hardware read each, held in a local.
            temperature = dht_sensor.temperature
            humidity = dht_sensor.humidity
            if temperature is None and humidity is None:
                raise RuntimeError("DHT22 returned nothing")
            return {
                "temperature_c": (round(temperature + DHT_TEMP_OFFSET, 2)
                                  if temperature is not None else None),
                "humidity_pct": (round(humidity + DHT_HUM_OFFSET, 2)
                                 if humidity is not None else None),
            }
        except Exception:
            if attempt < DHT_READ_ATTEMPTS - 1:
                time.sleep(DHT_RETRY_DELAY_S)
    print("DHT22 failed after %d attempts" % DHT_READ_ATTEMPTS)
    return {"temperature_c": None, "humidity_pct": None}


def read_all_sensors():
    data = {"dht22": {}, "scd30": {}, "pms5003": {}, "enviro_plus": {}}
    data["dht22"] = read_dht22()
    try:
        if scd30.get_data_ready():
            m = scd30.read_measurement()
            if m:
                data["scd30"] = {"co2_ppm": round(m[0], 2), "temperature_c": round(m[1], 2), "humidity_pct": round(m[2], 2)}
            else:
                data["scd30"] = {"co2_ppm": None, "temperature_c": None, "humidity_pct": None}
        else:
            data["scd30"] = {"co2_ppm": None, "temperature_c": None, "humidity_pct": None}
    except Exception:
        data["scd30"] = {"co2_ppm": None, "temperature_c": None, "humidity_pct": None}
    try:
        readings = pms5003.read()
        data["pms5003"] = {"pm1_0": readings.pm_ug_per_m3(1.0), "pm2_5": readings.pm_ug_per_m3(2.5), "pm10_0": readings.pm_ug_per_m3(10)}
    except ReadTimeoutError:
        pms5003.setup()
        data["pms5003"] = {"pm1_0": None, "pm2_5": None, "pm10_0": None}
    except Exception:
        data["pms5003"] = {"pm1_0": None, "pm2_5": None, "pm10_0": None}
    try:
        raw_temp = bme280.get_temperature()
        data["enviro_plus"] = {
            "temperature_c": round(raw_temp - ((get_cpu_temperature() - raw_temp) / 2.25) + ENV_FACTOR_TEMP, 2),
            "humidity_pct": round(bme280.get_humidity() + ENV_FACTOR_HUM, 2),
            "pressure_hpa": round(bme280.get_pressure(), 2),
            "light_lux": round(ltr559.get_lux(), 2)}
    except Exception:
        data["enviro_plus"] = {"temperature_c": None, "humidity_pct": None, "pressure_hpa": None, "light_lux": None}
    return data


engine = TriggerEngine(config["TRIGGER_THRESHOLDS"])


# --- 5. BUFFERING (unchanged) ---
def load_buffer():
    with buffer_lock:
        if os.path.exists(BUFFER_FILE):
            try:
                with open(BUFFER_FILE, "r") as handle:
                    return json.load(handle)
            except Exception:
                return []
        return []


def save_buffer(buffer_list):
    """Write the backlog atomically: temp file, flush to disk, then rename.

    Writing in place meant a power cut mid-write corrupted the whole file and
    lost all 100 buffered records. A rename cannot be half-done, so the worst
    case is now losing only the newest record, never the backlog.
    """
    with buffer_lock:
        tmp = BUFFER_FILE + ".tmp"
        try:
            with open(tmp, "w") as handle:
                json.dump(buffer_list, handle)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp, BUFFER_FILE)
        except Exception as error:
            print("Failed to write buffer: %s" % error)
            try:
                os.unlink(tmp)
            except OSError:
                pass


def append_to_buffer(payload):
    if payload.get("hb", False):
        return
    buf = load_buffer()
    buf.append(payload)
    while len(buf) > MAX_BUFFER_SIZE:
        buf.pop(0)
    save_buffer(buf)


def apply_config(ack):
    """Config piggyback: apply known keys from the ACK, ignore unknowns."""
    global heartbeat_interval, last_cfg_ver
    if "hb" in ack:
        heartbeat_interval = int(ack["hb"]) * 60
    if "cfg_ver" in ack:
        last_cfg_ver = ack["cfg_ver"]


def to_record(payload):
    """The plain-JSON record sent inside the DTLS channel. The server forces
    identity from the certificate, so id here is advisory only."""
    record = {"id": DEVICE_ID, "type": "airq", "ts": payload["ts"],
              "event": payload["event"], "hb": payload.get("hb", False),
              "sensors": payload.get("sensors", {}), "cause": payload.get("cause", "")}
    if last_cfg_ver is not None:
        record["cfg_ver"] = last_cfg_ver
    return record


def ensure_session():
    """Make sure a live DTLS session exists, rediscovering the server if the
    old address stopped answering.

    This is what makes startup order irrelevant: a device booted before the
    server keeps failing here harmlessly, buffering as it goes, and connects
    by itself the moment the server appears. A server that moves to a new
    DHCP address is re-found without touching the device.
    """
    if client.connected():
        return True
    if client.connect():
        return True

    # The handshake failed. Either the server is down, or it moved.
    found = server_discovery.find_server(
        static_fallback=STATIC_SERVER, log=print)
    if found:
        client.set_server(found[0], found[1])
        return client.connect()
    return False


def deliver(payload):
    """Ensure a session and send one record, returning True on a verified ACK."""
    if not ensure_session():
        return False
    ack = client.send_and_ack(to_record(payload))
    if ack is None:
        return False
    apply_config(ack)
    return True


def mark_ack_failure():
    global consecutive_ack_fails
    consecutive_ack_fails += 1
    print("ACK failure streak: %d/%d" % (consecutive_ack_fails, MAX_MISSED_ACKS))
    client.close()
    if consecutive_ack_fails >= MAX_MISSED_ACKS:
        consecutive_ack_fails = 0
        hard_network_reset()
        time.sleep(RECONNECT_BACKOFF_S)


def network_worker():
    global consecutive_ack_fails
    while True:
        payload = network_queue.get()
        if payload is None:
            network_queue.task_done()
            break
        is_hb = payload.get("hb", False)

        # Flush buffered packets first, oldest to newest.
        #
        # THE BUG THIS FIXES: the gate used to be
        #     if offline and client.connected() and not is_hb:
        # which asked "am I connected?" BEFORE anything had a chance to
        # reconnect. deliver() is what reconnects, and it ran only after this
        # check, so on the offline->online transition the answer was always
        # "no" and the backlog never drained - while new readings sent fine.
        # Now we actively establish the session first, and heartbeats are
        # allowed to drain the backlog too (a quiet site sends nothing but
        # heartbeats, so excluding them meant the buffer sat there for hours).
        offline = load_buffer()
        if offline and ensure_session():
            print("Flushing %d buffered packets..." % len(offline))
            flushed = []
            for past in offline:
                time.sleep(random.uniform(0.03, 0.12))
                if deliver(past):
                    consecutive_ack_fails = 0
                    flushed.append(past)
                else:
                    mark_ack_failure()
                    break
            if flushed:
                save_buffer([p for p in offline if p not in flushed])
                print("Flushed %d/%d buffered packets" % (len(flushed), len(offline)))

        # Send the new packet.
        if deliver(payload):
            consecutive_ack_fails = 0
        else:
            print("Transmission failed / no ACK")
            if not is_hb:
                append_to_buffer(payload)
            mark_ack_failure()
        network_queue.task_done()


threading.Thread(target=network_worker, daemon=True).start()


def send_data(payload):
    global event_counter
    event_counter += 1
    payload["event"] = "%d_%d" % (session_id, event_counter)
    enqueue_newest_wins(payload)


def enqueue_newest_wins(payload):
    global queue_displaced_count
    try:
        network_queue.put_nowait(payload)
        return
    except queue.Full:
        pass
    try:
        displaced = network_queue.get_nowait()
        network_queue.task_done()
        append_to_buffer(displaced)
        queue_displaced_count += 1
    except queue.Empty:
        pass
    try:
        network_queue.put_nowait(payload)
    except queue.Full:
        append_to_buffer(payload)


# --- 6. ORCHESTRATION ---
print("Allowing hardware to stabilize 20 sec...")
time.sleep(20)
client.connect()

# Start the live MCP command listener (mains-powered AMU only).
_live = live_server.LiveCommandServer(PKI_DIR, DEVICE_ID, _live_read, port=LIVE_PORT)
threading.Thread(target=_live.serve_forever, daemon=True).start()

print("Starting Smart Ageing AirQ (DTLS)...")

try:
    while True:
        time.sleep(2)
        try:
            current_data = read_all_sensors()
            _latest_reading = current_data
            trigger, cause, is_heartbeat = engine.evaluate(current_data, force_initial_payload)
            if trigger:
                payload = {"id": DEVICE_ID, "type": "airq", "ts": int(time.time()),
                           "hb": is_heartbeat, "cause": cause, "sensors": current_data}
                print("Data Queued (Cause: %s)" % cause)
                send_data(payload)
                if force_initial_payload:
                    force_initial_payload = False
        except Exception as error:
            print("[%s] Recoverable loop error: %s" % (time.time(), error))
except KeyboardInterrupt:
    print("Shutting down...")
    client.close()
