"""Two boards provisioned with the same name must not both be accepted.

Both would carry validly CA-signed certificates, so certificate verification
alone cannot tell them apart - their data would merge under one device id and
the (id, event) index would silently discard the loser's records. See
ARCHITECTURE.md 17.
"""

import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import identity_guard as guard

CERT_A = b"\x30\x82\x01\x0a" + b"DEVICE-A-KEY-MATERIAL" * 8
CERT_B = b"\x30\x82\x01\x0a" + b"DEVICE-B-KEY-MATERIAL" * 8


class IdentityGuardTests(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(tempfile.mkdtemp(), "identities.json")
        self.now = time.time()

    def test_first_certificate_is_accepted_and_recorded(self):
        ok, _ = guard.check(self.path, "NMU_16", CERT_A, self.now)
        self.assertTrue(ok)
        self.assertTrue(os.path.exists(self.path))

    def test_same_device_reconnecting_is_accepted(self):
        guard.check(self.path, "NMU_16", CERT_A, self.now)
        for _ in range(5):
            ok, reason = guard.check(self.path, "NMU_16", CERT_A, self.now)
            self.assertTrue(ok, reason)

    def test_a_clone_with_the_same_name_is_refused(self):
        guard.check(self.path, "NMU_16", CERT_A, self.now)
        ok, reason = guard.check(self.path, "NMU_16", CERT_B, self.now)
        self.assertFalse(ok, "a second, different certificate must be refused")
        self.assertIn("DUPLICATE IDENTITY", reason)

    def test_refusal_does_not_evict_the_legitimate_unit(self):
        guard.check(self.path, "NMU_16", CERT_A, self.now)
        guard.check(self.path, "NMU_16", CERT_B, self.now)
        ok, _ = guard.check(self.path, "NMU_16", CERT_A, self.now)
        self.assertTrue(ok, "the original unit must keep working after a clone tries")

    def test_different_names_are_independent(self):
        self.assertTrue(guard.check(self.path, "NMU_16", CERT_A, self.now)[0])
        self.assertTrue(guard.check(self.path, "NMU_17", CERT_B, self.now)[0])

    def test_forget_allows_deliberate_reprovision(self):
        guard.check(self.path, "NMU_16", CERT_A, self.now)
        self.assertFalse(guard.check(self.path, "NMU_16", CERT_B, self.now)[0])
        self.assertTrue(guard.forget(self.path, "NMU_16"))
        self.assertTrue(guard.check(self.path, "NMU_16", CERT_B, self.now)[0],
                        "after forget, a re-provisioned unit must be accepted")

    def test_missing_certificate_bytes_do_not_block_a_device(self):
        ok, reason = guard.check(self.path, "NMU_16", None, self.now)
        self.assertTrue(ok, "guard must fail open if the stack gives no DER")
        self.assertIn("not checked", reason)

    def test_record_survives_a_corrupt_file(self):
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("{ this is not json")
        ok, _ = guard.check(self.path, "NMU_16", CERT_A, self.now)
        self.assertTrue(ok)
        ok2, _ = guard.check(self.path, "NMU_16", CERT_B, self.now)
        self.assertFalse(ok2, "guard must still work after recovering the file")

    def test_no_temp_file_left_behind(self):
        guard.check(self.path, "NMU_16", CERT_A, self.now)
        self.assertFalse(os.path.exists(self.path + ".tmp"))


def _run():
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(IdentityGuardTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print("RESULT: %s" % ("PASS - duplicate identities are refused"
                          if result.wasSuccessful() else "FAIL"))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(_run())
