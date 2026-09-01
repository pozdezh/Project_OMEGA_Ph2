"""A buffered AMU reading is dated when it was TAKEN, not when it arrived.

The AMU had the defect the NMU was fixed for on 2026-08-27: a reading captured
before the clock could be trusted goes out with ts = 0, and the server then
dates it by arrival. That is right for a live reading and wrong for one that
waited in the buffer - a whole outage collapses onto the reconnect moment and
the history for that period is false.

The AMU carries one complication the NMU does not. Its buffer is a FILE, so a
reading outlives the process that took it - and rung 2 of the recovery ladder
restarts the process, so by the time a buffered reading is finally sent it is
usually a different process sending it. The elapsed time is therefore measured
against the BOOT (/proc/uptime, keyed on the kernel's boot id) rather than
against the process. Across a reboot the reference is genuinely gone and the
reading stays undated rather than invented.

    py -3.12 amu/test_stamp_resolution.py
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class StampResolution(unittest.TestCase):
    def setUp(self):
        import clock
        self.c = clock
        self._saved = (clock._server_synced, clock._offset_s,
                       clock._ntp_at_start, clock._restored_across_planned_reboot)

    def tearDown(self):
        (self.c._server_synced, self.c._offset_s, self.c._ntp_at_start,
         self.c._restored_across_planned_reboot) = self._saved

    def _trust_the_clock(self):
        self.c._server_synced = True
        self.c._offset_s = 0.0

    def _distrust_the_clock(self):
        # ALL THREE sources, not two. Missing the third let a leaked reboot
        # marker from another test quietly re-trust a clock this test had
        # deliberately taken away.
        self.c._server_synced = False
        self.c._ntp_at_start = False
        self.c._restored_across_planned_reboot = False

    def test_a_reading_that_knows_its_time_is_never_second_guessed(self):
        self._trust_the_clock()
        self.assertEqual(self.c.resolve_stamp(1787780000, self.c.capture_marker()),
                         1787780000,
                         "a reading dated at capture keeps its own timestamp")

    def test_an_unknown_time_stays_unknown(self):
        self._distrust_the_clock()
        marker = self.c.capture_marker()
        self.assertEqual(self.c.resolve_stamp(0, marker), 0,
                         "with no trustworthy clock it must admit it cannot "
                         "tell, never guess")

    def test_a_reading_buffered_through_an_outage_keeps_its_real_time(self):
        """The repair. Captured 20 minutes ago while the clock was unknown,
        sent now that it is known: it must be dated 20 minutes ago."""
        marker = {"up": self.c._since_boot_s() - 1200.0, "boot": self.c._boot_identity()}
        self._trust_the_clock()
        resolved = self.c.resolve_stamp(0, marker)
        self.assertAlmostEqual(resolved, int(time.time()) - 1200, delta=2,
                               msg="a 20 min wait must be subtracted, not ignored")

    def test_a_whole_outage_does_not_collapse_onto_the_reconnect(self):
        """The failure itself, in AMU form: readings taken across 20 minutes
        and drained in seconds must stay 20 minutes wide."""
        now_up = self.c._since_boot_s()
        markers = [{"up": now_up - (1200.0 * i / 39.0), "boot": self.c._boot_identity()}
                   for i in range(40)]
        self._trust_the_clock()
        dated = [self.c.resolve_stamp(0, m) for m in markers]
        spread = max(dated) - min(dated)
        self.assertGreater(spread, 1150,
                           "40 readings spanning 20 min collapsed into %ds - "
                           "this is the bug" % spread)
        self.assertLess(spread, 1250)

    def test_a_reading_that_outlived_its_own_boot_is_not_invented(self):
        """The AMU-only case. The buffer is on disk and survives a REBOOT;
        the elapsed-time reference does not. A marker from a boot that has
        ended cannot be measured against this one, and must be refused rather
        than produce a confident wrong time.

        Deliberately keyed on the BOOT and not the process: rung 2 of the
        ladder restarts the process, so a per-process key refused almost every
        real case - which is how 15 readings still collapsed on AMU_15 after
        the first version of this fix shipped."""
        stale = {"up": self.c._since_boot_s() - 1200.0, "boot": "a-boot-that-has-ended"}
        self._trust_the_clock()
        self.assertEqual(self.c.resolve_stamp(0, stale), 0,
                         "a marker from a dead process must not be used")

    def test_a_missing_or_broken_marker_never_raises(self):
        self._trust_the_clock()
        for junk in (None, {}, {"up": "not-a-number", "boot": self.c._boot_identity()},
                     {"boot": self.c._boot_identity()}, "string", 42):
            self.assertEqual(self.c.resolve_stamp(0, junk), 0,
                             "a malformed marker must yield 0, not an exception")

    def test_a_clock_restored_across_our_own_reboot_is_trusted(self):
        """The rule the user set on 2026-08-27: a reboot performed to heal is
        not the same event as being unplugged.

        The Pi has no hardware clock, but systemd saves the time on the way
        down and restores it on the way up, so across OUR OWN reboot the clock
        comes back correct. Refusing it meant a unit that healed itself could
        not date anything until the server answered - and a reboot is exactly
        when the server cannot be reached.
        """
        import os
        import tempfile
        marker = os.path.join(tempfile.mkdtemp(prefix="omega_marker_"),
                              "reboot_marker.json")
        saved = self.c.REBOOT_MARKER_PATH
        self.c.REBOOT_MARKER_PATH = marker
        try:
            self.assertTrue(self.c.note_planned_reboot())
            self.assertTrue(self.c._consume_planned_reboot_marker(),
                            "a note this unit just left must be believed")
        finally:
            self.c.REBOOT_MARKER_PATH = saved

    def test_the_note_can_never_be_believed_twice(self):
        """It is consumed on reading. A note left behind would make the NEXT
        power cut look like a planned reboot."""
        import os
        import tempfile
        marker = os.path.join(tempfile.mkdtemp(prefix="omega_marker_"),
                              "reboot_marker.json")
        saved = self.c.REBOOT_MARKER_PATH
        self.c.REBOOT_MARKER_PATH = marker
        try:
            self.c.note_planned_reboot()
            self.assertTrue(self.c._consume_planned_reboot_marker())
            self.assertFalse(self.c._consume_planned_reboot_marker(),
                             "the note must be destroyed once acted on")
            self.assertFalse(os.path.exists(marker))
        finally:
            self.c.REBOOT_MARKER_PATH = saved

    def test_an_unplugged_unit_does_not_trust_its_stale_clock(self):
        """No note at all - which is what being unplugged looks like."""
        import os
        import tempfile
        marker = os.path.join(tempfile.mkdtemp(prefix="omega_marker_"),
                              "reboot_marker.json")
        saved = self.c.REBOOT_MARKER_PATH
        self.c.REBOOT_MARKER_PATH = marker
        try:
            self.assertFalse(self.c._consume_planned_reboot_marker(),
                             "with no note the clock must not be believed")
        finally:
            self.c.REBOOT_MARKER_PATH = saved

    def test_an_old_note_is_refused(self):
        """A note from some earlier reboot, with a power cut since. The
        restored clock is not continuous with it and must not be trusted."""
        import json
        import os
        import tempfile
        marker = os.path.join(tempfile.mkdtemp(prefix="omega_marker_"),
                              "reboot_marker.json")
        saved = self.c.REBOOT_MARKER_PATH
        self.c.REBOOT_MARKER_PATH = marker
        try:
            with open(marker, "w", encoding="utf-8") as handle:
                json.dump({"at": time.time() - 99999.0}, handle)
            self.assertFalse(self.c._consume_planned_reboot_marker(),
                             "a note older than the reboot window is not "
                             "evidence of a clean reboot")
        finally:
            self.c.REBOOT_MARKER_PATH = saved

    def test_the_marker_never_reaches_the_wire(self):
        """_to_record is a whitelist; the marker is internal bookkeeping and
        the server has no field for it."""
        import re
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "network.py"), encoding="utf-8") as handle:
            source = handle.read()
        body = re.search(r"^def _to_record\(.*?(?=^def )", source, re.S | re.M).group(0)
        emitted = re.findall(r'record\["(\w+)"\]', body) + re.findall(r'"(\w+)":', body)
        self.assertNotIn("_cap", emitted,
                         "the capture marker must never be sent to the server")


if __name__ == "__main__":
    unittest.main(verbosity=2)
