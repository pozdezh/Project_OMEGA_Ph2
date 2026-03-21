import os
import sys
import time
import json
import socket
import configparser
from datetime import datetime

# --- HARDWARE IMPORTS ---
import board
import adafruit_dht
from smbus2 import SMBus
from bme280 import BME280
from ltr559 import LTR559
from pms5003 import PMS5003, ReadTimeoutError
from scd30_i2c import SCD30

# --- 1. LOAD CONFIGURATION ---
config = configparser.ConfigParser()
config.read('/home/shootconstantin/DELTA/config/global.ini')

UDP_IP = config['NETWORK']['target_ip']
UDP_PORT = int(config['NETWORK']['target_port'])
HEARTBEAT_INTERVAL = int(config['TRIGGER_THRESHOLDS']['alarm_heartbeat_interval'])

# --- 2. HARDWARE INITIALIZATION & CALIBRATION OFFSETS ---
print("Initializing Sensors...")

# DHT22
DHT_TEMP_OFFSET = -11.05  
DHT_HUM_OFFSET = 29.1 
dht_sensor = adafruit_dht.DHT22(board.D4)

# Enviro+
ENV_FACTOR_TEMP = -9.35 
ENV_FACTOR_HUM = 33.4
bus = SMBus(1)
bme280 = BME280(i2c_dev=bus)
ltr559 = LTR559()

def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = int(f.read()) / 1000.0
        return temp
    except:
        return 0.0

# PMS5003
pms5003 = PMS5003()

# SCD30
scd30 = SCD30()
scd30.set_temperature_offset(9.0) 
scd30.set_measurement_interval(2)
scd30.start_periodic_measurement()

# --- 3. THE TRIGGER ENGINE ---
class TriggerEngine:
    def __init__(self, cfg):
        self.cfg = cfg
        self.history = {'co2': [], 'pm2_5': []}
        self.window_size = 5
        self.last_sent = {'temp': None, 'hum': None, 'lux': None}
        self.is_currently_in_alarm = False
        self.last_transmission_time = 0

    def evaluate(self, data):
        trigger_now = False
        active_alarms = False
        relative_change = False

        co2 = data['scd30']['co2_ppm']
        pm25 = data['pms5003']['pm2_5']
        temp = data['dht22']['temperature_c'] 
        hum = data['dht22']['humidity_pct']
        lux = data['enviro_plus']['light_lux']

        # TIER 1: ALARMS
        if any([
            co2 and co2 >= float(self.cfg['co2_critical_high']),
            pm25 and pm25 >= float(self.cfg['pm_critical_high']),
            temp and (temp <= float(self.cfg['temp_critical_low']) or temp >= float(self.cfg['temp_critical_high'])),
            hum and (hum <= float(self.cfg['hum_critical_low']) or hum >= float(self.cfg['hum_critical_high'])),
            lux and (lux <= float(self.cfg['lux_critical_low']) or lux >= float(self.cfg['lux_critical_high']))
        ]):
            active_alarms = True

        # TIER 2: RELATIVE CHANGES
        if co2 is not None:
            if not self.history['co2']: relative_change = True
            else:
                avg = sum(self.history['co2']) / len(self.history['co2'])
                if abs(co2 - avg) >= float(self.cfg['co2_delta']): relative_change = True
            self.history['co2'].append(co2)
            if len(self.history['co2']) > self.window_size: self.history['co2'].pop(0)

        if pm25 is not None:
            if not self.history['pm2_5']: relative_change = True
            else:
                avg = sum(self.history['pm2_5']) / len(self.history['pm2_5'])
                if abs(pm25 - avg) >= float(self.cfg['pm_delta']): relative_change = True
            self.history['pm2_5'].append(pm25)
            if len(self.history['pm2_5']) > self.window_size: self.history['pm2_5'].pop(0)

        if temp is not None:
            if self.last_sent['temp'] is None or abs(temp - self.last_sent['temp']) >= float(self.cfg['temp_step']):
                relative_change = True
                self.last_sent['temp'] = temp
                
        if hum is not None:
            if self.last_sent['hum'] is None or abs(hum - self.last_sent['hum']) >= float(self.cfg['hum_step']):
                relative_change = True
                self.last_sent['hum'] = hum
                
        if lux is not None:
            if self.last_sent['lux'] is None or abs(lux - self.last_sent['lux']) >= float(self.cfg['lux_step']):
                relative_change = True
                self.last_sent['lux'] = lux

        now = time.time()
        if relative_change: trigger_now = True
        if active_alarms:
            if not self.is_currently_in_alarm: trigger_now = True 
            elif (now - self.last_transmission_time) >= HEARTBEAT_INTERVAL: trigger_now = True 
        elif self.is_currently_in_alarm and not active_alarms: trigger_now = True 

        self.is_currently_in_alarm = active_alarms
        if trigger_now: self.last_transmission_time = now

        return trigger_now

# --- 4. DATA GATHERING LOOP ---
def read_all_sensors():
    data = {"dht22": {}, "scd30": {}, "pms5003": {}, "enviro_plus": {}}
    
    # 1. DHT22
    try:
        data["dht22"]["temperature_c"] = round(dht_sensor.temperature + DHT_TEMP_OFFSET, 2) if dht_sensor.temperature else None
        data["dht22"]["humidity_pct"] = round(dht_sensor.humidity + DHT_HUM_OFFSET, 2) if dht_sensor.humidity else None
    except Exception:
        data["dht22"]["temperature_c"] = None
        data["dht22"]["humidity_pct"] = None

    # 2. SCD30
    try:
        if scd30.get_data_ready():
            m = scd30.read_measurement()
            if m:
                data["scd30"]["co2_ppm"] = round(m[0], 2)
                data["scd30"]["temperature_c"] = round(m[1], 2)
                data["scd30"]["humidity_pct"] = round(m[2], 2)
    except Exception:
        data["scd30"] = {"co2_ppm": None, "temperature_c": None, "humidity_pct": None}

    # 3. PMS5003
    try:
        readings = pms5003.read()
        data["pms5003"]["pm1_0"] = readings.pm_ug_per_m3(1.0)
        data["pms5003"]["pm2_5"] = readings.pm_ug_per_m3(2.5)
        data["pms5003"]["pm10_0"] = readings.pm_ug_per_m3(10)
    except ReadTimeoutError:
        pms5003.setup() # Reset on timeout
        data["pms5003"] = {"pm1_0": None, "pm2_5": None, "pm10_0": None}

    # 4. Enviro+
    try:
        raw_temp = bme280.get_temperature()
        cpu_temp = get_cpu_temperature()
        comp_temp = raw_temp - ((cpu_temp - raw_temp) / 2.25) + ENV_FACTOR_TEMP
        
        data["enviro_plus"]["temperature_c"] = round(comp_temp, 2)
        data["enviro_plus"]["humidity_pct"] = round(bme280.get_humidity() + ENV_FACTOR_HUM, 2)
        data["enviro_plus"]["pressure_hpa"] = round(bme280.get_pressure(), 2)
        data["enviro_plus"]["light_lux"] = round(ltr559.get_lux(), 2)
    except Exception:
         data["enviro_plus"] = {"temperature_c": None, "humidity_pct": None, "pressure_hpa": None, "light_lux": None}

    return data

# --- 5. ORCHESTRATION ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
engine = TriggerEngine(config['TRIGGER_THRESHOLDS'])

print(f"Allowing hardware to stabilize 20 sec...")
time.sleep(20)

print(f"Starting Smart Ageing AirQ Phase 1. Target UDP: {UDP_IP}:{UDP_PORT}")

try:
    while True:
        time.sleep(2) # The 2-second tick
        current_data = read_all_sensors()
        
        if engine.evaluate(current_data):
            payload = {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "unit_id": "AMU_01",
                "sensors": current_data
            }
            sock.sendto(json.dumps(payload).encode('utf-8'), (UDP_IP, UDP_PORT))
            print(f"[{payload['timestamp']}] TRIGGER EVENT: UDP JSON Dispatched.")

except KeyboardInterrupt:
    print("Shutting down...")
finally:
    sock.close()
