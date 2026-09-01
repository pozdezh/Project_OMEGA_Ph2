"""Autonomy tests for the AMU: the recovery ladder and clock trust.

Both exist for a device that cannot be touched physically or over SSH once
deployed, so both are tested for the cases that actually strand a unit.

    py -3.12 amu/test_recovery_clock.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class Ladder(unittest.TestCase):
    def setUp(self):
        import recovery
        import shutil
        import tempfile
        self.r = recovery
        self.t = 1000.0
        self.rebooted = []
        self.exited = []
        self.radio = []
        # The contact clock is a real file now, so every test gets its own
        # directory - a leaked clock from one test would silently seed the
        # next one and the suite would stop meaning anything.
        self._state_dir = tempfile.mkdtemp(prefix="omega_recovery_")
        self._saved_state_path = recovery.CONTACT_STATE_PATH
        recovery.CONTACT_STATE_PATH = os.path.join(self._state_dir,
                                                   "contact_state.json")
        self.addCleanup(shutil.rmtree, self._state_dir, True)
        self.addCleanup(setattr, recovery, "CONTACT_STATE_PATH",
                        self._saved_state_path)
        # The reboot rung leaves a note for clock.py saying "this restart was
        # deliberate". Left at its default path that note lands in the SOURCE
        # TREE and is then read by whatever test runs next, which is exactly
        # what happened on 2026-08-27: the stamp tests began believing the
        # machine had just rebooted and a deliberately-untrusted clock came
        # back trusted. Redirect it here too.
        import clock
        self._saved_marker_path = clock.REBOOT_MARKER_PATH
        clock.REBOOT_MARKER_PATH = os.path.join(self._state_dir,
                                                "reboot_marker.json")
        self.addCleanup(setattr, clock, "REBOOT_MARKER_PATH",
                        self._saved_marker_path)
        recovery.reset(self.t)

    def _check(self, at, reboot_ok=True):
        return self.r.check(
            now=at,
            on_radio_reset=lambda: self.radio.append(at),
            reboot=lambda: (self.rebooted.append(at), reboot_ok)[1],
            exit_fn=lambda code: self.exited.append((at, code)))

    def test_healthy_unit_is_never_touched(self):
        for step in range(0, 600, 30):
            self.r.note_contact(self.t + step)
            self.assertEqual(self._check(self.t + step), self.r.STAGE_NONE)
        self.assertEqual((self.radio, self.exited, self.rebooted), ([], [], []))

    def test_ladder_climbs_in_order_and_each_rung_fires_once(self):
        self.assertEqual(self._check(self.t + 599), self.r.STAGE_NONE)
        self.assertEqual(self._check(self.t + 601), self.r.STAGE_RADIO)
        self.assertEqual(len(self.radio), 1)
        self.assertEqual(self._check(self.t + 700), self.r.STAGE_NONE,
                         "the radio rung must not fire repeatedly")
        self.assertEqual(len(self.radio), 1)

        self.assertEqual(self._check(self.t + 1201), self.r.STAGE_RESTART)
        self.assertEqual(len(self.exited), 1)
        self.assertEqual(self.exited[0][1], 1)

        self._check(self.t + 1801)
        self.assertEqual(len(self.rebooted), 1,
                         "past the last rung the unit must reboot itself")

    def test_contact_at_any_point_resets_the_whole_ladder(self):
        self._check(self.t + 601)
        self.assertEqual(len(self.radio), 1)
        self.r.note_contact(self.t + 700)
        self.assertEqual(self._check(self.t + 1250), self.r.STAGE_NONE,
                         "1250 is far past the restart rung in absolute time, "
                         "but only 550s since contact - nothing may fire")
        self.assertEqual(self.exited, [])

    def test_reboot_falls_back_to_process_restart_when_not_permitted(self):
        """Without the sudoers rule the reboot request fails. The unit must
        still restart its process rather than silently doing nothing."""
        self._check(self.t + 1801, reboot_ok=False)
        self.assertEqual(len(self.rebooted), 1)
        self.assertEqual(len(self.exited), 1,
                         "a refused reboot must degrade to a restart")

    def test_no_rung_can_fire_before_the_device_has_had_its_say(self):
        """The flaw this closes: with a 60 min heartbeat and a fixed 30 min
        reboot rung, a perfectly healthy unit that simply had nothing to
        report would reboot itself before its own heartbeat could ever prove
        the link was fine - forever, and on every quiet unit at once.

        Silence shorter than one heartbeat is not evidence of a fault, so
        every rung must sit clear of the heartbeat by construction.
        """
        beat = 3600.0
        radio_at, restart_at, reboot_at = self.r.thresholds(beat)
        self.assertGreater(radio_at, beat,
                           "a unit must be allowed to miss nothing at all "
                           "before its first rung fires")
        self.assertGreater(restart_at, radio_at)
        self.assertGreater(reboot_at, restart_at)

        for step in range(0, int(beat), 60):
            stage = self.r.check(
                now=self.t + step, heartbeat_s=beat,
                on_radio_reset=lambda: self.radio.append(step),
                reboot=lambda: (self.rebooted.append(step), True)[1],
                exit_fn=lambda code: self.exited.append((step, code)))
            self.assertEqual(stage, self.r.STAGE_NONE)
        self.assertEqual((self.radio, self.exited, self.rebooted), ([], [], []),
                         "nothing may escalate inside one heartbeat period")

    def _simulate_process_restart(self):
        """What rung 2 ACTUALLY does in production: os._exit, systemd rebuilds
        the process, every module-level variable is gone.

        This test file used to model the restart as a no-op lambda that
        returned and let the same process keep counting - which is why the
        bug below survived every test run. A restart that does not restart
        proves nothing.
        """
        self.r._last_contact = None
        self.r._stage = self.r.STAGE_NONE
        self.r._persisted_at = 0.0

    def test_a_dead_link_still_reaches_every_rung_without_a_human(self):
        """The autonomy requirement itself: whatever the heartbeat, a unit
        that genuinely cannot reach its server must climb the whole ladder on
        its own - radio reset, process restart, then reboot - and never wait
        for someone to walk over and replug it.

        Rung 2 is followed by a REAL restart here. Before 2026-08-26 that
        wiped the contact clock, the fresh process started counting from
        zero, and the reboot rung was unreachable: the unit oscillated
        between rungs 1 and 2 forever. AMU_15 sat offline for three hours
        that way, with exactly one boot in its journal.
        """
        beat = 3600.0
        radio_at, restart_at, reboot_at = self.r.thresholds(beat)
        self.assertEqual(self.r.check(now=self.t + radio_at, heartbeat_s=beat,
                                      on_radio_reset=lambda: self.radio.append(1)),
                         self.r.STAGE_RADIO)
        self.assertEqual(self.r.check(now=self.t + restart_at, heartbeat_s=beat,
                                      exit_fn=lambda code: self.exited.append(code)),
                         self.r.STAGE_RESTART)

        self._simulate_process_restart()
        self.assertEqual(self.r.check(now=self.t + restart_at + 1.0, heartbeat_s=beat),
                         self.r.STAGE_NONE)

        self.assertEqual(self.r.check(now=self.t + reboot_at, heartbeat_s=beat,
                                      reboot=lambda: (self.rebooted.append(1), True)[1]),
                         self.r.STAGE_RESTART)
        self.assertEqual((self.radio, self.exited, self.rebooted),
                         ([1], [1], [1]),
                         "every rung must fire unaided on a genuinely dead link")

    def test_the_contact_clock_survives_a_process_restart(self):
        """Rung 2 restarts the process; the outage did not restart with it."""
        beat = 3600.0
        _, restart_at, _ = self.r.thresholds(beat)
        self._simulate_process_restart()
        self.r.check(now=self.t + restart_at, heartbeat_s=beat)
        self.assertAlmostEqual(self.r.silent_for(self.t + restart_at), restart_at,
                               delta=self.r.PERSIST_INTERVAL_S,
                               msg="a restarted process must continue the same "
                                   "outage, not start a new one")

    def test_rung_two_does_not_repeat_after_the_restart_it_performs(self):
        """The 2026-08-27 fault, pinned.

        Persisting the clock across rung 2 made the top rung reachable, but
        only the CLOCK was persisted - not how far up the ladder the outage
        had got. The restarted process inherited the silence with a clear
        "this rung has fired" flag, so it restarted itself again, and again:
        AMU_15's own log shows rung 2 at 1201s, then 1300, 1404, 1510, 1612
        and 1721, before the reboot rung finally caught it at 1824s.

        Six restarts where one was intended. The unit still recovered, so
        nothing looked broken from outside - which is exactly why this needs
        a test rather than an eye on the dashboard.
        """
        beat = 3600.0
        _, restart_at, reboot_at = self.r.thresholds(beat)
        self.assertEqual(self.r.check(now=self.t + restart_at, heartbeat_s=beat,
                                      exit_fn=lambda code: self.exited.append(code)),
                         self.r.STAGE_RESTART)

        # Three successive restarts, each a genuinely fresh process. Two
        # checks per restart on purpose: the first only ADOPTS the inherited
        # clock, the second is the one that would act on it. Checking once
        # would pass for the wrong reason.
        for extra in (60.0, 160.0, 260.0):
            self._simulate_process_restart()
            at = self.t + restart_at + extra
            self.r.check(now=at, heartbeat_s=beat)
            self.assertEqual(
                self.r.check(now=at, heartbeat_s=beat,
                             exit_fn=lambda code: self.exited.append(code)),
                self.r.STAGE_NONE,
                "rung 2 fired again %.0fs after the restart it performed" % extra)

        self.assertEqual(self.exited, [1],
                         "rung 2 must restart the process ONCE per outage, "
                         "not on every cycle of the restarted process")

        # ...and the ladder must still climb to the top from there. Silencing
        # a repeated rung must not silence the ladder.
        self._simulate_process_restart()
        self.r.check(now=self.t + reboot_at, heartbeat_s=beat)
        self.assertEqual(self.r.check(now=self.t + reboot_at, heartbeat_s=beat,
                                      reboot=lambda: (self.rebooted.append(1), True)[1]),
                         self.r.STAGE_RESTART)
        self.assertEqual(self.rebooted, [1])

    def test_a_reboot_clears_the_clock_so_it_cannot_reboot_loop(self):
        """The other half of the rule. A reboot IS a fresh start - otherwise
        the unit comes back, reads its own pre-reboot silence, and reboots
        again at once."""
        beat = 3600.0
        _, _, reboot_at = self.r.thresholds(beat)
        self.assertEqual(self.r.check(now=self.t + reboot_at, heartbeat_s=beat,
                                      reboot=lambda: (self.rebooted.append(1), True)[1]),
                         self.r.STAGE_RESTART)
        self._simulate_process_restart()
        self.assertEqual(self.r.check(now=self.t + reboot_at + 1.0, heartbeat_s=beat,
                                      reboot=lambda: (self.rebooted.append(2), True)[1]),
                         self.r.STAGE_NONE,
                         "a unit that just rebooted must not reboot again")
        self.assertEqual(self.rebooted, [1])

    def test_a_clock_from_the_future_is_refused(self):
        """An unsynced RTC or a jumped NTP correction must not be able to
        strand a healthy unit."""
        self.r._write_contact(self.t + 99999.0)
        self._simulate_process_restart()
        self.r.check(now=self.t, heartbeat_s=3600.0)
        self.assertAlmostEqual(self.r.silent_for(self.t), 0.0, delta=1.0,
                               msg="a nonsense clock must start fresh, "
                                   "not be acted on")

    def test_the_shipped_heartbeat_defaults_can_never_arm_a_reboot_loop(self):
        """Regression guard for the whole class of bug: no heartbeat an
        operator can set from the dashboard may put the reboot rung inside
        two heartbeat periods."""
        for minutes in (5, 10, 20, 30, 60, 120):
            beat = minutes * 60.0
            _, _, reboot_at = self.r.thresholds(beat)
            self.assertGreater(reboot_at, beat * 2.0,
                               "hb=%d min arms a reboot loop" % minutes)

    def test_a_unit_booted_before_its_server_does_not_reboot_loop(self):
        import recovery
        recovery._last_contact = None
        recovery._stage = recovery.STAGE_NONE
        self.assertEqual(self._check(self.t + 99999), recovery.STAGE_NONE,
                         "never having reached the server yet must start the "
                         "clock, not read as an infinitely old outage")


class Clock(unittest.TestCase):
    def setUp(self):
        import clock
        self.c = clock
        clock._offset_s = 0.0
        clock._server_synced = False

    def test_untrusted_clock_stamps_zero_so_the_server_decides(self):
        self.c._ntp_at_start = False
        self.assertFalse(self.c.trusted())
        self.assertEqual(self.c.stamp(), 0,
                         "a wrong confident timestamp is worse than none: the "
                         "server substitutes arrival time only for ts == 0")

    def test_server_time_from_an_ack_makes_the_clock_trusted(self):
        self.c._ntp_at_start = False
        import time
        target = time.time() + 50000
        self.assertTrue(self.c.apply_server_time(target))
        self.assertTrue(self.c.trusted())
        self.assertAlmostEqual(self.c.now(), target, delta=2.0)
        self.assertNotEqual(self.c.stamp(), 0)

    def test_nonsense_server_time_is_refused(self):
        self.c._ntp_at_start = False
        for bad in (0, -1, None, "yesterday"):
            self.assertFalse(self.c.apply_server_time(bad), repr(bad))
        self.assertFalse(self.c.trusted())

    def test_ntp_synced_unit_is_trusted_without_the_server(self):
        self.c._ntp_at_start = True
        self.assertTrue(self.c.trusted())
        self.assertNotEqual(self.c.stamp(), 0)

    def test_session_ids_do_not_come_from_the_clock(self):
        ids = {self.c.new_session_id() for _ in range(500)}
        self.assertGreater(len(ids), 490,
                           "clock-derived ids collide across reboots when the "
                           "clock is wrong, and colliding ids make event ids "
                           "collide, which the server silently dedups away")
        for value in ids:
            self.assertTrue(0 <= value < 2**32)


if __name__ == "__main__":
    unittest.main(verbosity=2)
