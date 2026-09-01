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
        self.r = recovery
        self.t = 1000.0
        self.rebooted = []
        self.exited = []
        self.radio = []
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
