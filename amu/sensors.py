import math
import time

import board
import adafruit_dht
from smbus2 import SMBus
from bme280 import BME280
from ltr559 import LTR559
from pms5003 import PMS5003, ReadTimeoutError
from scd30_i2c import SCD30

import amu_config

MAGNUS_A = 17.625
MAGNUS_B = 243.04
MAGNUS_ES0_HPA = 6.1094

_EMPTY_SCD30 = {"co2_ppm": None, "temperature_c": None,
                "humidity_pct": None, "dew_point_c": None}

dht_sensor = None
bus = None
bme280 = None
ltr559 = None
pms5003 = None
scd30 = None
_pressure_comp_seeded = False


def _maybe_seed_pressure_compensation(pressure_hpa):
    """One-shot: SCD30 starts with compensation off (init_hardware() cannot
    trust a BME280 reading taken before main.py's own 20s stabilization
    window). Called from read_all_sensors() once that window has passed and
    a real reading exists - re-sending start_periodic_measurement() is the
    documented way to set compensation while continuous mode is already
    running (Sensirion interface description 1.3.1)."""
    global _pressure_comp_seeded
    if _pressure_comp_seeded:
        return
    candidate_mbar = int(round(pressure_hpa))
    if amu_config.SCD30_PRESSURE_MIN_MBAR <= candidate_mbar <= amu_config.SCD30_PRESSURE_MAX_MBAR:
        scd30.start_periodic_measurement(candidate_mbar)
        _pressure_comp_seeded = True
        print("SCD30 ambient pressure compensation seeded at %d mBar" % candidate_mbar)


def init_hardware():
    global dht_sensor, bus, bme280, ltr559, pms5003, scd30
    print("Initializing Sensors for %s..." % amu_config.DEVICE_ID)
    dht_sensor = adafruit_dht.DHT22(board.D4)
    bus = SMBus(1)
    bme280 = BME280(i2c_dev=bus)
    bme280.get_temperature()
    ltr559 = LTR559()
    # /dev/serial0 is the GPIO-header UART symlink; on boards with onboard
    # Bluetooth (e.g. Pi 3/4/Zero W), /dev/ttyAMA0 is claimed by Bluetooth and
    # the header is exposed as the mini-UART (/dev/ttyS0) instead - hardcoding
    # ttyAMA0 fails on exactly those boards.
    pms5003 = PMS5003(device="/dev/serial0")
    scd30 = SCD30()
    scd30.set_temperature_offset(amu_config.SCD30_TEMP_OFFSET_C)
    scd30.set_measurement_interval(2)
    scd30.start_periodic_measurement(0)


def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as handle:
            return int(handle.read()) / 1000.0
    except Exception:
        return 0.0


def self_heat_compensated_temp(raw_temp_c, cpu_temp_c, coupling_factor):
    """Removes enclosure self-heating bias, proportional to how far the CPU
    runs above the sensor. Same form used community-wide for Pi-adjacent
    BME280s; applied here to any co-located sensor sharing the same heat
    source. See ARCHITECTURE.md for re-validating coupling_factor."""
    return raw_temp_c - ((cpu_temp_c - raw_temp_c) / coupling_factor)


def _saturation_vapor_pressure_hpa(temp_c):
    return MAGNUS_ES0_HPA * math.exp(MAGNUS_A * temp_c / (temp_c + MAGNUS_B))


def dew_point_c(temp_c, humidity_pct):
    """Dew point from temperature and relative humidity (Magnus-Tetens).

    Reported alongside each sensor's own T/RH because RELATIVE humidity is
    not comparable between sensors sitting at different temperatures, and
    comparing it anyway produces false alarms. Measured on this unit
    2026-08-21: SCD30 read 65.7%RH and DHT22 49.3%RH - a 16-point gap that
    looks like a broken sensor. Their dew points were 19.51C and 19.94C,
    0.43C apart. The sensors agreed almost exactly on how much water was in
    the air; they only disagreed on their own temperature.

    Dew point is the temperature at which that air would start to condense,
    so it depends only on water content, not on how warm the sensor is. Two
    sensors in the same room must agree on it whatever their self-heating.
    That makes it the honest cross-check, and the number to compare in the
    memo."""
    if temp_c is None or humidity_pct is None or humidity_pct <= 0:
        return None
    alpha = (math.log(humidity_pct / 100.0)
             + (MAGNUS_A * temp_c) / (MAGNUS_B + temp_c))
    if alpha >= MAGNUS_A:
        return None
    return (MAGNUS_B * alpha) / (MAGNUS_A - alpha)


def humidity_at_corrected_temp(raw_humidity_pct, raw_temp_c, corrected_temp_c):
    """Relative humidity is intrinsically temperature-dependent (RH = actual
    vapor pressure / saturation vapor pressure AT that temperature) - a flat
    additive %RH offset ignores this. Standard Magnus-Tetens approach:
    recover the actual vapor pressure from the raw (biased) reading, then
    re-express it as %RH at the self-heat-corrected temperature."""
    vapor_pressure_hpa = (raw_humidity_pct / 100.0) * _saturation_vapor_pressure_hpa(raw_temp_c)
    corrected = 100.0 * vapor_pressure_hpa / _saturation_vapor_pressure_hpa(corrected_temp_c)
    return max(0.0, min(100.0, corrected))


def read_dht22():
    """Read the DHT22 once per cycle, with retries, keeping real 0.0 values.

    Three fixes live here: `is not None` rather than a truthy check, because
    0.0 C is a real reading that Python's truthiness would discard; ONE
    hardware read per property per cycle, since each access re-reads the
    sensor and the sensor fails intermittently by design; and a bounded
    retry, since the very next read usually succeeds after a single failure.
    """
    for attempt in range(amu_config.DHT_READ_ATTEMPTS):
        try:
            temperature = dht_sensor.temperature
            humidity = dht_sensor.humidity
            if temperature is None and humidity is None:
                raise RuntimeError("DHT22 returned nothing")
            corrected_temp = (
                self_heat_compensated_temp(temperature, get_cpu_temperature(),
                                           amu_config.DHT_SELF_HEAT_FACTOR)
                if temperature is not None else None)
            corrected_hum = None
            if humidity is not None:
                corrected_hum = (
                    humidity_at_corrected_temp(humidity, temperature, corrected_temp)
                    if corrected_temp is not None else humidity)
            dew = dew_point_c(corrected_temp, corrected_hum)
            return {
                "temperature_c": round(corrected_temp, 2) if corrected_temp is not None else None,
                "humidity_pct": round(corrected_hum, 2) if corrected_hum is not None else None,
                "dew_point_c": round(dew, 2) if dew is not None else None,
            }
        except Exception:
            if attempt < amu_config.DHT_READ_ATTEMPTS - 1:
                time.sleep(amu_config.DHT_RETRY_DELAY_S)
    print("DHT22 failed after %d attempts" % amu_config.DHT_READ_ATTEMPTS)
    return {"temperature_c": None, "humidity_pct": None, "dew_point_c": None}


def read_all_sensors():
    data = {"dht22": {}, "scd30": {}, "pms5003": {}, "enviro_plus": {}}
    data["dht22"] = read_dht22()
    try:
        if scd30.get_data_ready():
            m = scd30.read_measurement()
            if m:
                scd_dew = dew_point_c(m[1], m[2])
                data["scd30"] = {"co2_ppm": round(m[0], 2), "temperature_c": round(m[1], 2),
                                 "humidity_pct": round(m[2], 2),
                                 "dew_point_c": round(scd_dew, 2) if scd_dew is not None else None}
            else:
                data["scd30"] = _EMPTY_SCD30.copy()
        else:
            # Not an error: the SCD30 produces a value every 2s on its own
            # clock, so a sampling tick can legitimately land between two of
            # them. This is the single largest source of the "missing" counts
            # in the daily statistics.
            data["scd30"] = _EMPTY_SCD30.copy()
    except Exception:
        data["scd30"] = _EMPTY_SCD30.copy()
    try:
        readings = pms5003.read()
        data["pms5003"] = {"pm1_0": readings.pm_ug_per_m3(1.0),
                           "pm2_5": readings.pm_ug_per_m3(2.5),
                           "pm10_0": readings.pm_ug_per_m3(10)}
    except ReadTimeoutError:
        pms5003.setup()
        data["pms5003"] = {"pm1_0": None, "pm2_5": None, "pm10_0": None}
    except Exception:
        data["pms5003"] = {"pm1_0": None, "pm2_5": None, "pm10_0": None}
    try:
        raw_temp = bme280.get_temperature()
        raw_hum = bme280.get_humidity()
        pressure_hpa = bme280.get_pressure()
        corrected_temp = self_heat_compensated_temp(
            raw_temp, get_cpu_temperature(), amu_config.ENV_SELF_HEAT_FACTOR)
        corrected_hum = humidity_at_corrected_temp(raw_hum, raw_temp, corrected_temp)
        env_dew = dew_point_c(corrected_temp, corrected_hum)
        data["enviro_plus"] = {
            "temperature_c": round(corrected_temp, 2),
            "humidity_pct": round(corrected_hum, 2),
            # Least trustworthy humidity of the three, and the dew point is
            # how you see that. This sensor sits ON the Pi header at ~47C,
            # where the air is only ~22%RH, so extrapolating its reading down
            # to room temperature multiplies any raw error by ~2.5x. A -3%RH
            # raw error - exactly the BME280's datasheet accuracy - becomes a
            # 2.3C dew-point error. Not a bug and not fixable by another
            # offset; a property of measuring humidity on a hot surface.
            "dew_point_c": round(env_dew, 2) if env_dew is not None else None,
            "pressure_hpa": round(pressure_hpa, 2),
            "light_lux": round(ltr559.get_lux(), 2)}
        _maybe_seed_pressure_compensation(pressure_hpa)
    except Exception:
        data["enviro_plus"] = {"temperature_c": None, "humidity_pct": None,
                               "dew_point_c": None,
                               "pressure_hpa": None, "light_lux": None}
    return data
