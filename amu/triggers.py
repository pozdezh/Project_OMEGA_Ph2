import time

import amu_config

ABOVE = "above"
BELOW = "below"

ALARM_RULES = (
    ("High CO2", "co2", "co2_critical_high", "co2_critical_clear", ABOVE),
    ("High PM2.5", "pm2_5", "pm_critical_high", "pm_critical_clear", ABOVE),
    ("High Temp", "temp", "temp_critical_high", "temp_critical_high_clear", ABOVE),
    ("Low Temp", "temp", "temp_critical_low", "temp_critical_low_clear", BELOW),
    ("High Humidity", "hum", "hum_critical_high", "hum_critical_high_clear", ABOVE),
    ("Low Humidity", "hum", "hum_critical_low", "hum_critical_low_clear", BELOW),
    ("High Lux", "lux", "lux_critical_high", "lux_critical_high_clear", ABOVE),
    ("Low Lux", "lux", "lux_critical_low", "lux_critical_low_clear", BELOW),
)

DELTA_RULES = (
    ("co2", "CO2", "co2_delta"),
    ("pm1", "PM1.0", "pm_delta"),
    ("pm2_5", "PM2.5", "pm_delta"),
    ("pm10", "PM10.0", "pm_delta"),
)

STEP_RULES = (
    ("temp", "Temp", "temp_step"),
    ("hum", "Humidity", "hum_step"),
    ("lux", "Lux", "lux_step"),
    ("co2", "CO2", "co2_delta"),
    ("pm1", "PM1.0", "pm_delta"),
    ("pm2_5", "PM2.5", "pm_delta"),
    ("pm10", "PM10.0", "pm_delta"),
)

HISTORY_KEYS = ("co2", "pm1", "pm2_5", "pm10")

# See ARCHITECTURE.md 22.
REDUNDANT_SOURCES = {
    "temp": (("dht22", "temperature_c"),
             ("scd30", "temperature_c"),
             ("enviro_plus", "temperature_c")),
    "hum": (("dht22", "humidity_pct"),
            ("scd30", "humidity_pct"),
            ("enviro_plus", "humidity_pct")),
}


class TriggerEngine:
    def __init__(self, cfg, clock=time.time):
        self.cfg = cfg
        self.clock = clock
        self.window_size = 5
        self.history = {key: [] for key in HISTORY_KEYS}
        self.last_transmitted = {"temp": None, "hum": None, "lux": None,
                                 "co2": None, "pm1": None, "pm2_5": None,
                                 "pm10": None}
        self.alarm_active = {rule[0]: False for rule in ALARM_RULES}
        self.alarm_pending_since = {rule[0]: None for rule in ALARM_RULES}
        self.is_currently_in_alarm = False
        self.last_transmission_time = 0
        self.sustain_s = self._number("alarm_sustain_s", 60.0)
        self.clear_s = self._number("alarm_clear_s", 120.0)
        self.min_send_interval_s = self._number("min_send_interval_s", 60.0)
        # How often a still-active alarm repeats itself. Independent of the
        # heartbeat so that raising the heartbeat to quieten a chatty site
        # cannot also quieten its alarms.
        self.alarm_repeat_s = self._number("alarm_repeat_s", 300.0)
        self.last_alarm_announced = 0

    def _number(self, key, default):
        try:
            return float(self.cfg[key])
        except (KeyError, TypeError, ValueError):
            return float(default)

    def _threshold(self, key, fallback_key):
        try:
            return float(self.cfg[key])
        except (KeyError, TypeError, ValueError):
            return float(self.cfg[fallback_key])

    @staticmethod
    def _breached(value, limit, direction):
        if direction == ABOVE:
            return value >= limit
        return value <= limit

    def _update_alarms(self, readings, now):
        for name, source, trip_key, clear_key, direction in ALARM_RULES:
            value = readings.get(source)
            if value is None:
                self.alarm_pending_since[name] = None
                continue
            active = self.alarm_active[name]
            limit = (self._threshold(clear_key, trip_key) if active
                     else float(self.cfg[trip_key]))
            breached = self._breached(value, limit, direction)
            if breached == active:
                self.alarm_pending_since[name] = None
                continue
            started = self.alarm_pending_since[name]
            if started is None:
                self.alarm_pending_since[name] = now
                continue
            needed = self.sustain_s if breached else self.clear_s
            if now - started >= needed:
                self.alarm_active[name] = breached
                self.alarm_pending_since[name] = None
        return [name for name in self.alarm_active if self.alarm_active[name]]

    def _spikes(self, readings):
        found = []
        for source, label, delta_key in DELTA_RULES:
            value = readings.get(source)
            if value is None:
                continue
            window = self.history[source]
            if window:
                mean = sum(window) / len(window)
                if abs(value - mean) >= float(self.cfg[delta_key]):
                    found.append("%s Spike/Drop" % label)
            window.append(value)
            if len(window) > self.window_size:
                window.pop(0)
        return found

    def _steps(self, readings):
        found = []
        for source, label, step_key in STEP_RULES:
            value = readings.get(source)
            previous = self.last_transmitted.get(source)
            if value is None or previous is None:
                continue
            if abs(value - previous) >= float(self.cfg[step_key]):
                drift = "%s Step/Drift" % label
                if drift not in found:
                    found.append(drift)
        return found

    @staticmethod
    def fuse(data, key):
        """Median of whichever redundant sensors reported a usable number.

        Median rather than mean: with three sensors, one failing sensor cannot
        move the answer at all, whereas a mean is dragged by exactly the reading
        least worth trusting. Two sensors fall back to their mean and one to
        itself, so the unit degrades instead of going blind.
        """
        values = []
        for block, field in REDUNDANT_SOURCES[key]:
            value = data.get(block, {}).get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                continue
            if value != value:
                continue
            values.append(float(value))
        if not values:
            return None
        values.sort()
        middle = len(values) // 2
        if len(values) % 2:
            return values[middle]
        return (values[middle - 1] + values[middle]) / 2.0

    @staticmethod
    def _summarise(items):
        if len(items) > 1:
            return items[0] + (" (+%d)" % (len(items) - 1))
        return items[0]

    def evaluate(self, data, force_initial):
        now = self.clock()
        readings = {
            "co2": data.get("scd30", {}).get("co2_ppm"),
            "pm1": data.get("pms5003", {}).get("pm1_0"),
            "pm2_5": data.get("pms5003", {}).get("pm2_5"),
            "pm10": data.get("pms5003", {}).get("pm10_0"),
            "temp": self.fuse(data, "temp"),
            "hum": self.fuse(data, "hum"),
            "lux": data.get("enviro_plus", {}).get("light_lux"),
        }

        previously_in_alarm = self.is_currently_in_alarm
        alarm_list = self._update_alarms(readings, now)
        active_alarms = bool(alarm_list)
        changes = self._spikes(readings) + self._steps(readings)
        time_since_last = now - self.last_transmission_time
        quiet_enough = time_since_last >= self.min_send_interval_s

        trigger_now = False
        is_heartbeat = False
        primary_cause = "Unknown"

        if force_initial:
            trigger_now = True
            primary_cause = "Initial Server Sync"
        elif active_alarms and not previously_in_alarm:
            trigger_now = True
            primary_cause = "New Alarm: " + self._summarise(alarm_list)
        elif previously_in_alarm and not active_alarms:
            trigger_now = True
            primary_cause = "All Alarms Cleared"
        elif active_alarms and now - self.last_alarm_announced >= self.alarm_repeat_s:
            # An active alarm re-announces on ITS OWN clock, before ordinary
            # change traffic is considered. Measured 2026-08-24: a latched High
            # Temp alarm went unmentioned for hours because every CO2 drift
            # reset the shared transmission timer, so the newest record - the
            # one the dashboard shows - always named the drift instead. In a
            # care setting an active alarm must not be maskable by unrelated
            # chatter. See FINDINGS #51.
            trigger_now = True
            primary_cause = "Sustained Alarm: " + self._summarise(alarm_list)
        elif changes and quiet_enough:
            trigger_now = True
            primary_cause = self._summarise(changes)
        elif time_since_last >= amu_config.heartbeat_interval:
            # A heartbeat stays a HEARTBEAT even while an alarm is latched.
            #
            # It used to be relabelled "Sustained Alarm" here, with
            # is_heartbeat left False - so it was stored and drawn on the
            # dashboard as a fresh alarm record. That made alarm traffic a
            # function of the HEARTBEAT rate: measured 2026-08-26, dropping
            # the heartbeat to 1 minute turned a single latched High Temp
            # into a visible alarm record every 61 seconds, twenty-four in
            # half an hour, instead of the six that alarm_repeat_s asks for.
            #
            # Reaching this branch at all means the alarm branch above did
            # NOT fire, which means alarm_repeat_s has not elapsed - the
            # alarm was already announced less than five minutes ago and
            # there is nothing new to say about it. Announcing it again here
            # is duplication, not diligence.
            #
            # Nothing is lost by staying quiet: the alarm keeps re-announcing
            # on its own clock at line ~204, the dashboard's banner is fed by
            # those records, and this heartbeat still carries the current
            # sensor readings and still collects an ACK - which is how a
            # quiet unit learns the time, a config change, or that it has
            # been revoked.
            #
            # The rule this restores: heartbeat rate is a LIVENESS decision,
            # alarm rate is a SAFETY decision, and changing one must not
            # silently change the other.
            trigger_now = True
            primary_cause = "Routine Heartbeat"
            is_heartbeat = True

        self.is_currently_in_alarm = active_alarms
        if not active_alarms:
            self.last_alarm_announced = 0
        elif trigger_now and primary_cause.startswith(("New Alarm", "Sustained Alarm")):
            self.last_alarm_announced = now
        if trigger_now:
            self.last_transmission_time = now
            self.last_transmitted.update(readings)
        return trigger_now, primary_cause, is_heartbeat
