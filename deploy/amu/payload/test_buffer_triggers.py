import os
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class _Cfg(dict):
    pass


BASE_CFG = _Cfg({
    "co2_critical_high": "1400", "co2_critical_clear": "1300",
    "pm_critical_high": "55", "pm_critical_clear": "45",
    "temp_critical_high": "30", "temp_critical_high_clear": "29",
    "temp_critical_low": "12", "temp_critical_low_clear": "13",
    "hum_critical_high": "70", "hum_critical_high_clear": "67",
    "hum_critical_low": "25", "hum_critical_low_clear": "28",
    "lux_critical_high": "800", "lux_critical_high_clear": "750",
    "lux_critical_low": "5", "lux_critical_low_clear": "8",
    "co2_delta": "150", "pm_delta": "15",
    "temp_step": "1.5", "hum_step": "5", "lux_step": "100",
    "alarm_sustain_s": "60", "alarm_clear_s": "120",
    "min_send_interval_s": "60",
})


def reading(co2=None, temp=None, hum=None, lux=None, pm25=None):
    return {"scd30": {"co2_ppm": co2},
            "pms5003": {"pm1_0": None, "pm2_5": pm25, "pm10_0": None},
            "dht22": {"temperature_c": temp, "humidity_pct": hum},
            "enviro_plus": {"light_lux": lux}}


class FakeClock(object):
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class TriggerEngineTests(unittest.TestCase):
    def setUp(self):
        import amu_config
        import triggers
        amu_config.heartbeat_interval = 300
        self.clock = FakeClock()
        self.engine = triggers.TriggerEngine(BASE_CFG, clock=self.clock)
        self.engine.evaluate(reading(co2=600), True)

    def _run(self, seconds, step, **kw):
        fired = []
        for _ in range(int(seconds / step)):
            self.clock.advance(step)
            trigger, cause, _hb = self.engine.evaluate(reading(**kw), False)
            if trigger:
                fired.append(cause)
        return fired

    def test_value_parked_on_the_threshold_does_not_flap(self):
        fired = []
        for index in range(300):
            self.clock.advance(2)
            co2 = 1400 + (40 if index % 2 else -40)
            trigger, cause, _hb = self.engine.evaluate(reading(co2=co2), False)
            if trigger:
                fired.append(cause)
        alarms = [c for c in fired if "Alarm" in c]
        self.assertEqual(alarms, [],
                         "a value oscillating across the trip point never holds "
                         "for alarm_sustain_s, so it must produce no alarm churn "
                         "at all; got %r" % (alarms,))

    def test_a_dip_inside_the_deadband_does_not_clear_the_alarm(self):
        for _ in range(40):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1500), False)
        self.assertTrue(self.engine.alarm_active["High CO2"])
        for _ in range(200):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1350), False)
        self.assertTrue(self.engine.alarm_active["High CO2"],
                        "1350 is between clear (1300) and trip (1400) - the "
                        "deadband must hold the alarm latched")

    def test_alarm_needs_to_be_sustained(self):
        self.clock.advance(2)
        trigger, cause, _hb = self.engine.evaluate(reading(co2=1500), False)
        self.assertFalse("Alarm" in cause and trigger,
                         "a single sample above the limit must not raise an alarm")
        self.clock.advance(30)
        self.engine.evaluate(reading(co2=1500), False)
        self.assertFalse(self.engine.alarm_active["High CO2"])
        self.clock.advance(31)
        trigger, cause, _hb = self.engine.evaluate(reading(co2=1500), False)
        self.assertTrue(self.engine.alarm_active["High CO2"])
        self.assertEqual(cause, "New Alarm: High CO2")

    def test_a_brief_excursion_never_becomes_an_alarm(self):
        for _ in range(10):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1500), False)
        self.assertFalse(self.engine.alarm_active["High CO2"])
        self.clock.advance(2)
        self.engine.evaluate(reading(co2=600), False)
        self.assertFalse(self.engine.alarm_active["High CO2"])

    def test_clearing_needs_the_longer_hold(self):
        for _ in range(40):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1500), False)
        self.assertTrue(self.engine.alarm_active["High CO2"])
        self.clock.advance(2)
        self.engine.evaluate(reading(co2=1000), False)
        self.clock.advance(100)
        self.engine.evaluate(reading(co2=1000), False)
        self.assertTrue(self.engine.alarm_active["High CO2"],
                        "must not clear before alarm_clear_s has elapsed")
        self.clock.advance(21)
        trigger, cause, _hb = self.engine.evaluate(reading(co2=1000), False)
        self.assertFalse(self.engine.alarm_active["High CO2"])
        self.assertEqual(cause, "All Alarms Cleared")

    def test_spikes_are_rate_limited(self):
        fired = []
        co2 = 600
        for index in range(150):
            self.clock.advance(2)
            co2 = 600 if index % 2 else 1000
            trigger, cause, _hb = self.engine.evaluate(reading(co2=co2), False)
            if trigger:
                fired.append((self.clock.now, cause))
        gaps = [fired[i + 1][0] - fired[i][0] for i in range(len(fired) - 1)]
        self.assertTrue(all(gap >= 60 for gap in gaps),
                        "delta triggers must respect min_send_interval_s, gaps=%r" % (gaps,))
        self.assertLessEqual(len(fired), 6, "300s of violent swings must not send more than 6 times")

    def test_alarm_is_not_rate_limited(self):
        for _ in range(40):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1500), False)
        self.assertTrue(self.engine.alarm_active["High CO2"])
        self.engine.last_transmission_time = self.clock.now
        for _ in range(70):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1000), False)
        self.assertFalse(self.engine.alarm_active["High CO2"])

    def test_sustained_alarm_is_not_flagged_as_a_heartbeat(self):
        """A sustained alarm marked hb=True is hidden from both dashboard
        graphs (they filter heartbeat=0) and is skipped by the offline buffer.
        In a care setting that is the one message that must never be dropped
        or hidden."""
        for _ in range(40):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=1500), False)
        self.assertTrue(self.engine.alarm_active["High CO2"])
        seen = None
        for _ in range(400):
            self.clock.advance(2)
            trigger, cause, hb = self.engine.evaluate(reading(co2=1500), False)
            if trigger and cause.startswith("Sustained Alarm"):
                seen = (cause, hb)
                break
        self.assertIsNotNone(seen, "a sustained alarm should re-announce")
        self.assertFalse(seen[1],
                         "sustained alarm must NOT be hb=True: %r" % (seen,))

    def test_routine_heartbeat_is_still_flagged(self):
        fired = self._run(400, 2, co2=600)
        self.assertTrue(any("Heartbeat" in c for c in fired))

    def test_heartbeat_still_fires_when_nothing_happens(self):
        fired = self._run(400, 2, co2=600)
        self.assertTrue(any("Heartbeat" in cause for cause in fired),
                        "a quiet site must still emit a routine heartbeat")

    def test_median_ignores_one_lying_sensor(self):
        """The live numbers from AMU_T1 on 2026-08-24. The DHT22 alone reported
        62.55% humidity and the SCD30 reported 83.48% - a 28-point spread, so
        whichever sensor the alarm happened to read decided whether a
        high-humidity alarm existed. The median cannot be moved by one of
        three."""
        import triggers
        data = {"scd30": {"temperature_c": 27.21, "humidity_pct": 83.48},
                "dht22": {"temperature_c": 32.9, "humidity_pct": 62.55},
                "enviro_plus": {"temperature_c": 31.23, "humidity_pct": 55.07}}
        self.assertAlmostEqual(triggers.TriggerEngine.fuse(data, "temp"), 31.23)
        self.assertAlmostEqual(triggers.TriggerEngine.fuse(data, "hum"), 62.55)

        data["scd30"]["humidity_pct"] = 5.0
        self.assertAlmostEqual(
            triggers.TriggerEngine.fuse(data, "hum"), 55.07,
            "a sensor swinging from 83.5 to 5.0 must move the median by less "
            "than the mean would - it may only shift to the next value in")

    def test_median_degrades_instead_of_going_blind(self):
        import triggers
        two = {"dht22": {"temperature_c": 20.0},
               "scd30": {"temperature_c": 24.0},
               "enviro_plus": {}}
        self.assertEqual(triggers.TriggerEngine.fuse(two, "temp"), 22.0)
        one = {"dht22": {"temperature_c": 20.0}, "scd30": {}, "enviro_plus": {}}
        self.assertEqual(triggers.TriggerEngine.fuse(one, "temp"), 20.0)
        none = {"dht22": {}, "scd30": {}, "enviro_plus": {}}
        self.assertIsNone(triggers.TriggerEngine.fuse(none, "temp"))
        junk = {"dht22": {"temperature_c": "warm"}, "scd30": {"temperature_c": True},
                "enviro_plus": {"temperature_c": 21.5}}
        self.assertEqual(triggers.TriggerEngine.fuse(junk, "temp"), 21.5,
                         "a string and a bool are not readings")

    def test_active_alarm_is_not_masked_by_unrelated_traffic(self):
        """The 2026-08-24 defect. A latched High Temp alarm went unmentioned for
        hours: CO2 drift fired every few minutes, each send reset the shared
        transmission timer, so the heartbeat branch that re-announces the alarm
        never came due. The newest record - the one the dashboard shows - always
        named the drift, and the temperature alarm was invisible."""
        for _ in range(40):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=600, temp=40), False)
        self.assertTrue(self.engine.alarm_active["High Temp"])

        seen_alarm = False
        co2 = 600
        for index in range(400):
            self.clock.advance(2)
            # Unrelated CO2 chatter, violent enough to trigger constantly.
            co2 = 600 if index % 2 else 1000
            trigger, cause, _hb = self.engine.evaluate(
                reading(co2=co2, temp=40), False)
            if trigger and cause.startswith("Sustained Alarm"):
                seen_alarm = True
                break
        self.assertTrue(seen_alarm,
                        "an active alarm must re-announce on its own clock even "
                        "while other triggers keep firing")

    def test_alarm_repeat_stops_once_the_alarm_clears(self):
        for _ in range(40):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=600, temp=40), False)
        self.assertTrue(self.engine.alarm_active["High Temp"])
        for _ in range(200):
            self.clock.advance(2)
            self.engine.evaluate(reading(co2=600, temp=20), False)
        self.assertFalse(self.engine.alarm_active["High Temp"])
        fired = []
        for _ in range(300):
            self.clock.advance(2)
            trigger, cause, _hb = self.engine.evaluate(reading(co2=600, temp=20), False)
            if trigger:
                fired.append(cause)
        self.assertFalse(any(c.startswith("Sustained Alarm") for c in fired),
                         "a cleared alarm must stop repeating: %r" % (fired,))

    def test_missing_sensor_values_never_raise(self):
        for _ in range(20):
            self.clock.advance(2)
            self.engine.evaluate(reading(), False)


class BufferConcurrencyTests(unittest.TestCase):
    def setUp(self):
        import amu_config
        import buffer
        self.tmpdir = tempfile.mkdtemp()
        amu_config.BUFFER_FILE = os.path.join(self.tmpdir, "offline_buffer.json")
        amu_config.MAX_BUFFER_SIZE = 100
        self.buffer = buffer
        buffer.save_buffer([])

    def test_concurrent_appends_lose_nothing(self):
        threads = []
        for worker in range(4):
            def append(worker=worker):
                for index in range(25):
                    self.buffer.append_to_buffer(
                        {"event": "w%d_%d" % (worker, index), "hb": False})
            threads.append(threading.Thread(target=append))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        stored = self.buffer.load_buffer()
        self.assertEqual(len(stored), 100)
        self.assertEqual(len({record["event"] for record in stored}), 100)

    def test_flush_does_not_destroy_records_appended_during_it(self):
        for index in range(10):
            self.buffer.append_to_buffer({"event": "old_%d" % index, "hb": False})
        offline = self.buffer.load_buffer()

        stop = threading.Event()

        def appender():
            index = 0
            while not stop.is_set():
                self.buffer.append_to_buffer({"event": "new_%d" % index, "hb": False})
                index += 1
                if index >= 20:
                    break

        writer = threading.Thread(target=appender)
        writer.start()
        writer.join()
        stop.set()

        self.buffer.remove_delivered([record["event"] for record in offline])
        remaining = {record["event"] for record in self.buffer.load_buffer()}
        self.assertFalse(any(name.startswith("old_") for name in remaining),
                         "delivered records must be gone")
        self.assertEqual(len([n for n in remaining if n.startswith("new_")]), 20,
                         "records appended during the flush must survive it")

    def test_heartbeats_are_never_buffered(self):
        self.buffer.append_to_buffer({"event": "hb_1", "hb": True})
        self.assertEqual(self.buffer.load_buffer(), [])

    def test_oldest_is_discarded_when_full(self):
        for index in range(150):
            self.buffer.append_to_buffer({"event": "e_%d" % index, "hb": False})
        stored = self.buffer.load_buffer()
        self.assertEqual(len(stored), 100)
        self.assertEqual(stored[0]["event"], "e_50")
        self.assertEqual(stored[-1]["event"], "e_149")


class WolfsslGuardTests(unittest.TestCase):
    def setUp(self):
        import wolfssl_guard
        self.guard = wolfssl_guard
        wolfssl_guard._stuck_since = None

    def test_a_success_between_two_timeouts_clears_the_stuck_timer(self):
        self.guard.note_lock_timeout("first")
        self.assertIsNotNone(self.guard._stuck_since)
        self.guard.note_lock_acquired()
        self.assertIsNone(self.guard._stuck_since,
                          "any successful acquisition must prove the lock still cycles")

    def test_timer_arms_on_first_timeout(self):
        self.guard.note_lock_timeout("only")
        self.assertIsNotNone(self.guard._stuck_since)


if __name__ == "__main__":
    unittest.main(verbosity=2)
