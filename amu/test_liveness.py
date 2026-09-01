"""The system must stay LIVE, not merely lossless.

A buffer that never loses a reading is still a failure if an alarm sits in it
for an hour. These tests pin the two behaviours that keep a unit live:

  1. the newest reading is sent BEFORE any backlog, and
  2. a buffered record is retried on its own clock, without waiting for the
     next reading to come along and carry it out.

    py -3.12 amu/test_liveness.py
"""

import io
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _install_stub_modules():
    """main.py pulls in wolfSSL and sensor drivers at import. Neither exists
    on the gate machine, and neither is what these tests are about."""
    for name in ("sensors", "live_cache"):
        if name not in sys.modules:
            module = types.ModuleType(name)
            module.init_hardware = lambda: None
            module.read_all_sensors = lambda: {}
            module.write_sample = lambda *_a, **_k: None
            sys.modules[name] = module

    if "network" not in sys.modules:
        sys.modules["network"] = _FakeNetwork()


class _FakeNetwork(types.ModuleType):
    """Stands in for the DTLS transport, recording the ORDER of what it sent."""

    def __init__(self):
        types.ModuleType.__init__(self, "network")
        self.sent = []
        self.link_up = True
        self.ack_failures = 0
        self.radio_resets = 0

    def deliver(self, payload):
        if not self.link_up:
            return False
        self.sent.append(payload.get("event"))
        return True

    def ensure_session(self):
        return self.link_up

    def mark_ack_failure(self):
        self.ack_failures += 1

    def hard_network_reset(self):
        self.radio_resets += 1

    def take_question(self):
        return None

    def close(self):
        pass


_install_stub_modules()

import amu_config
import buffer
import main
import retry_schedule


class Backoff(unittest.TestCase):
    def setUp(self):
        self.s = retry_schedule.RetrySchedule(5.0, 60.0)

    def test_a_brief_glitch_costs_seconds_not_minutes(self):
        self.s.failed(0.0)
        self.assertTrue(self.s.due(5.0),
                        "the first retry must come within the floor interval")

    def test_a_dead_link_backs_off_but_never_past_the_ceiling(self):
        now = 0.0
        for _ in range(20):
            self.s.failed(now)
            now += self.s.interval_s
        self.assertLessEqual(self.s.interval_s, 60.0,
                             "backoff must never exceed the ceiling")

    def test_success_snaps_back_to_the_floor(self):
        for step in range(10):
            self.s.failed(float(step))
        self.assertGreater(self.s.interval_s, 5.0)
        self.s.succeeded(100.0)
        self.assertEqual(self.s.interval_s, 5.0,
                         "a working link must restore prompt retries")


class LiveFirst(unittest.TestCase):
    def setUp(self):
        self.net = sys.modules["network"]
        self.net.sent = []
        self.net.link_up = True
        self.net.ack_failures = 0
        self.tmp = os.path.join(HERE, "_test_liveness_buffer.json")
        self._original_buffer_file = amu_config.BUFFER_FILE
        amu_config.BUFFER_FILE = self.tmp
        buffer.save_buffer([])

    def tearDown(self):
        amu_config.BUFFER_FILE = self._original_buffer_file
        if os.path.exists(self.tmp):
            os.remove(self.tmp)

    def test_a_fresh_alarm_overtakes_the_whole_backlog(self):
        """The failure this closes: the live reading was sent LAST, behind a
        full flush of up to 100 buffered records. A fresh alarm queued behind
        history is not an alarm."""
        for index in range(1, 6):
            buffer.append_to_buffer({"event": "old_%d" % index})

        alarm = {"event": "ALARM_NOW", "hb": False}
        self.assertTrue(self.net.deliver(alarm))
        main.flush_backlog()

        self.assertEqual(self.net.sent[0], "ALARM_NOW",
                         "the newest reading must go out before any history")
        self.assertEqual(self.net.sent[1:],
                         ["old_1", "old_2", "old_3", "old_4", "old_5"],
                         "history must still follow, oldest first")

    def test_a_failed_flush_leaves_the_undelivered_records_alone(self):
        for index in range(1, 4):
            buffer.append_to_buffer({"event": "old_%d" % index})
        self.net.link_up = False

        self.assertFalse(main.flush_backlog(),
                         "a flush with no link must report failure")
        self.assertEqual(len(buffer.load_buffer()), 3,
                         "nothing may be deleted when nothing was acknowledged")

    def test_the_backlog_drains_without_a_new_reading_arriving(self):
        """The heart of it: on an event-driven unit the next reading can be an
        hour away. The buffer must empty on its own retry clock."""
        for index in range(1, 4):
            buffer.append_to_buffer({"event": "old_%d" % index})
        self.net.link_up = False
        self.assertFalse(main.flush_backlog())

        # The link comes back. No new reading is produced at any point.
        self.net.link_up = True
        self.assertTrue(main.flush_backlog())
        self.assertEqual(self.net.sent, ["old_1", "old_2", "old_3"])
        self.assertEqual(buffer.load_buffer(), [],
                         "the backlog must be gone without a new reading")

    def test_nothing_is_lost_when_the_handover_queue_overflows(self):
        """While the sender is blocked waiting for an ACK, sampling carries on.
        The handover queue is small, so it fills - and when it does the OLDEST
        item must be written to the buffer, never dropped."""
        depth = main.network_queue.maxsize
        for index in range(depth + 5):
            main.enqueue_newest_wins({"event": "e_%d" % index})

        queued = []
        while not main.network_queue.empty():
            queued.append(main.network_queue.get_nowait()["event"])
        spilled = [item["event"] for item in buffer.load_buffer()]

        self.assertEqual(len(queued) + len(spilled), depth + 5,
                         "every reading must be either queued or buffered, "
                         "queued=%s spilled=%s" % (queued, spilled))
        self.assertIn("e_%d" % (depth + 4), queued,
                      "the NEWEST reading must always win a slot in the queue")
        self.assertEqual(spilled, ["e_0", "e_1", "e_2", "e_3", "e_4"],
                         "the oldest readings are the ones that spill to disk")

    def test_the_sender_thread_survives_an_exception(self):
        """A daemon thread with nothing supervising it. If it dies the unit
        samples forever, sends nothing, and never escalates - a silent zombie
        that only a person power-cycling it can fix."""
        def _explode(_payload):
            raise RuntimeError("simulated transport blow-up")

        original = self.net.deliver
        self.net.deliver = _explode
        try:
            schedule = retry_schedule.RetrySchedule(5.0, 60.0)
            main.network_queue.put_nowait({"event": "boom", "hb": False})
            with self.assertRaises(RuntimeError):
                main._network_cycle(schedule)
        finally:
            self.net.deliver = original

        # The wrapper is what must absorb it. Prove the loop keeps going by
        # driving one more cycle after the failure.
        self.assertFalse(main._network_cycle(
            retry_schedule.RetrySchedule(5.0, 60.0)),
            "the worker must be ready to run again after a failed cycle")

    def test_the_retry_clock_is_faster_than_the_heartbeat(self):
        """The whole point: a buffered alarm must not wait for a heartbeat."""
        self.assertLess(amu_config.BACKLOG_RETRY_MAX_S,
                        amu_config.heartbeat_interval,
                        "the slowest retry must still beat the heartbeat")
        self.assertLessEqual(amu_config.RECOVERY_TICK_S,
                             amu_config.BACKLOG_RETRY_MIN_S,
                             "the worker must wake often enough to honour the "
                             "retry floor")


class _Completed(object):
    """The two fields of subprocess.CompletedProcess these tests care about."""

    def __init__(self, returncode, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class ExternalCommandsCannotWedgeTheWorker(unittest.TestCase):
    """The 2026-08-25 fault, pinned.

    Two units went silent for 45 minutes while fully alive: sampling every
    two seconds, answering live queries instantly, delivering nothing and
    never escalating. The cause was an unbounded nmcli call inside
    hard_network_reset(), which is reached from the network worker - the same
    thread that advances the recovery ladder. A stall there is not one lost
    reading, it disables the unit's ability to recover from anything.
    """

    @staticmethod
    def _real_connect_wifi():
        """Load the SHIPPED connect_wifi and nothing else.

        This test file stubs the network module (it must, to run off a Pi),
        so importing it would test the stub. Reading the real function out of
        the real file and running it against a fake subprocess tests the code
        that actually ships.
        """
        import re
        import subprocess
        with io.open(os.path.join(HERE, "network.py"), encoding="utf-8") as handle:
            source = handle.read()
        body = re.search(r"^def connect_wifi\(.*?(?=^def |\Z)", source,
                         re.S | re.M).group(0)
        # The timeout constants live at module level, outside the extracted
        # body. Without them every call raised NameError and was swallowed by
        # the function's own except clause - the test passed for the wrong
        # reason. Caught 2026-08-26.
        constants = "\n".join(line for line in source.split("\n")
                              if re.match(r"^WIFI_\w+_TIMEOUT_S\s*=", line))
        namespace = {"os": os, "subprocess": subprocess, "print": lambda *a, **k: None}
        exec(compile(constants + "\n" + body, "network.py", "exec"), namespace)
        return namespace["connect_wifi"], body

    def test_every_nmcli_call_is_bounded(self):
        """Structural, on purpose: the bug WAS the missing keyword. No
        behavioural test can see an absent timeout - only a real hang can,
        and by then a unit has been silent for 45 minutes."""
        _, body = self._real_connect_wifi()
        calls = body.count("subprocess.run(")
        bounded = body.count("timeout=")
        self.assertEqual(calls, bounded,
                         "every external command in connect_wifi must carry a "
                         "timeout - found %d call(s), %d bounded" % (calls, bounded))

    def test_a_hung_nmcli_costs_a_cycle_not_the_thread(self):
        import subprocess
        connect_wifi, _ = self._real_connect_wifi()

        original = subprocess.run
        subprocess.run = lambda *a, **k: (_ for _ in ()).throw(
            subprocess.TimeoutExpired(cmd="nmcli", timeout=15.0))
        try:
            result = connect_wifi("any-ssid", "any-pass")
        finally:
            subprocess.run = original

        self.assertFalse(result,
                         "a hung nmcli must return a failure, so the caller "
                         "escalates instead of waiting forever")

    def test_recovery_uses_the_saved_profile_not_a_fresh_association(self):
        """The 2026-08-26 fault, pinned.

        `nmcli dev wifi connect <ssid> password <pass>` returns exit 1 on
        every unit in this fleet - the profile is netplan-owned and nmcli
        refuses to rebuild it. Recovery rung 1 must activate the saved
        profile by name instead.
        """
        import subprocess
        connect_wifi, _ = self._real_connect_wifi()
        seen = []

        def fake_run(args, **kwargs):
            seen.append(args)
            if args[:5] == ["nmcli", "-t", "-f", "active,ssid"]:
                return _Completed(0, "no:test-wifi-net\n")
            if args[:4] == ["nmcli", "-t", "-f", "NAME,TYPE"]:
                return _Completed(0, "netplan-wlan0-test-wifi-net:802-11-wireless\n"
                                     "netplan-eth0:802-3-ethernet\n")
            if args[:4] == ["nmcli", "-t", "-f", "802-11-wireless.ssid"]:
                return _Completed(0, "802-11-wireless.ssid:test-wifi-net\n")
            if args[:5] == ["sudo", "-n", "nmcli", "connection", "up"]:
                return _Completed(0, "")
            if "dev" in args and "wifi" in args and "connect" in args:
                return _Completed(1, "", "802-11-wireless-security.key-mgmt: "
                                         "property is missing")
            return _Completed(0, "")

        original = subprocess.run
        subprocess.run = fake_run
        try:
            result = connect_wifi("test-wifi-net", "secret")
        finally:
            subprocess.run = original

        self.assertTrue(result)
        self.assertIn(["sudo", "-n", "nmcli", "connection", "up",
                       "netplan-wlan0-test-wifi-net"], seen)
        self.assertFalse([a for a in seen if "connect" in a and "dev" in a],
                         "the command that is known to fail on this fleet must "
                         "not be used when a saved profile exists")

    def test_a_failed_reconnect_is_reported_as_a_failure(self):
        """The second half of the same fault: the old code returned True
        unconditionally, so a rung that did nothing looked like a success."""
        import subprocess
        connect_wifi, _ = self._real_connect_wifi()

        def fake_run(args, **kwargs):
            if args[:5] == ["nmcli", "-t", "-f", "active,ssid"]:
                return _Completed(0, "no:test-wifi-net\n")
            if args[:4] == ["nmcli", "-t", "-f", "NAME,TYPE"]:
                return _Completed(0, "netplan-wlan0-test-wifi-net:802-11-wireless\n")
            if args[:4] == ["nmcli", "-t", "-f", "802-11-wireless.ssid"]:
                return _Completed(0, "802-11-wireless.ssid:test-wifi-net\n")
            return _Completed(1, "", "Error: no suitable device found.")

        original = subprocess.run
        subprocess.run = fake_run
        try:
            result = connect_wifi("test-wifi-net", "secret")
        finally:
            subprocess.run = original

        self.assertFalse(result, "a non-zero nmcli exit must reach the caller, "
                                 "or the ladder never escalates")

    def test_a_card_that_never_saw_this_network_still_joins(self):
        """First-ever association on a fresh card: no saved profile exists,
        so the password path is the correct one and must remain reachable."""
        import subprocess
        connect_wifi, _ = self._real_connect_wifi()
        seen = []

        def fake_run(args, **kwargs):
            seen.append(args)
            if args[:5] == ["nmcli", "-t", "-f", "active,ssid"]:
                return _Completed(0, "no:other\n")
            if args[:4] == ["nmcli", "-t", "-f", "NAME,TYPE"]:
                return _Completed(0, "netplan-eth0:802-3-ethernet\n")
            return _Completed(0, "")

        original = subprocess.run
        subprocess.run = fake_run
        try:
            result = connect_wifi("test-wifi-net", "secret")
        finally:
            subprocess.run = original

        self.assertTrue(result)
        self.assertIn(["sudo", "-n", "nmcli", "dev", "wifi", "connect",
                       "test-wifi-net", "password", "secret"], seen)

    def test_a_wedged_worker_is_survivable_without_a_person(self):
        """The watchdog runs in the SAMPLING thread, which is precisely the
        one still alive when the worker is stuck - so it can act."""
        self.assertGreater(main.WORKER_STALL_LIMIT_S,
                           amu_config.BACKLOG_RETRY_MAX_S * 5,
                           "the stall limit must clear a slow but legitimate "
                           "backlog flush, or a poor link causes restart loops")
        self.assertLess(main.WORKER_STALL_LIMIT_S, 3600.0,
                        "a wedged unit must be rebuilt well inside an hour - "
                        "the observed fault ran 45 minutes unnoticed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
