"""The unit's own log must survive its own reboot, and must never cause one.

Written after 2026-08-27, when AMU_15 became the first unit ever to reboot
itself unaided and the log of WHY was gone by the time anyone read it: the
journal was in RAM, because /var/log/journal existed but was empty and
Storage=auto therefore meant volatile.

    py -3.12 amu/test_applog.py
"""

import io
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class AppLog(unittest.TestCase):
    def setUp(self):
        import applog
        self.applog = applog
        self.dir = tempfile.mkdtemp(prefix="omega_applog_")
        self.path = os.path.join(self.dir, "logs", "amu.log")
        self._saved = (sys.stdout, sys.stderr)
        self.addCleanup(shutil.rmtree, self.dir, True)
        self.addCleanup(self._restore)

    def _restore(self):
        sys.stdout, sys.stderr = self._saved

    def _read(self, path=None):
        with io.open(path or self.path, encoding="utf-8") as handle:
            return handle.read()

    def test_what_the_unit_says_reaches_the_file(self):
        self.assertEqual(self.applog.start(self.path), self.path)
        print("no server contact for 3150s - rebooting this unit")
        sys.stdout.flush()
        self.assertIn("rebooting this unit", self._read(),
                      "the line explaining a reboot is the one that must survive it")

    def test_the_original_stream_still_works(self):
        """journalctl must keep working exactly as before - this is a mirror,
        not a replacement."""
        captured = io.StringIO()
        sys.stdout = captured
        self.applog.start(self.path)
        print("still visible to journald")
        self.assertIn("still visible to journald", captured.getvalue())
        self.assertIn("still visible to journald", self._read())

    def test_the_log_cannot_fill_the_card(self):
        """An SD card that fills up turns a diagnostic aid into the outage it
        was supposed to explain."""
        self.applog.start(self.path, max_bytes=2048, keep=2)
        for i in range(400):
            print("line %d %s" % (i, "x" * 60))
        sys.stdout.flush()
        total = 0
        for name in os.listdir(os.path.dirname(self.path)):
            total += os.path.getsize(os.path.join(os.path.dirname(self.path), name))
        self.assertLess(total, 2048 * 5,
                        "rotation must bound the log, found %d bytes" % total)
        self.assertTrue(os.path.exists(self.path + ".1"),
                        "the previous file must be kept, not simply discarded")

    def test_the_newest_lines_are_the_ones_kept(self):
        self.applog.start(self.path, max_bytes=1024, keep=2)
        for i in range(300):
            print("event %d" % i)
        sys.stdout.flush()
        self.assertIn("event 299", self._read(),
                      "the most recent history is what a diagnosis needs")

    def test_an_unwritable_log_never_takes_the_unit_down(self):
        """Losing the diary is an inconvenience. Losing the sensor is the job."""
        blocked = os.path.join(self.dir, "nope")
        with io.open(blocked, "w", encoding="utf-8") as handle:
            handle.write("I am a file, not a directory")
        self.assertIsNone(self.applog.start(os.path.join(blocked, "amu.log")),
                          "an impossible path must report failure, not raise")

    def test_a_card_that_dies_mid_run_is_survivable(self):
        self.applog.start(self.path)
        shutil.rmtree(os.path.dirname(self.path))
        try:
            print("the card just went read-only")
            sys.stdout.flush()
        except Exception as error:
            self.fail("writing to a dead card raised %r" % error)


if __name__ == "__main__":
    unittest.main(verbosity=2)
