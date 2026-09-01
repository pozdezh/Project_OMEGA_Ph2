"""A rogue discovery responder must cost one failed handshake, not the fleet.

Threat: an attacker who knows only the WLAN password runs a process that
answers discovery probes. It has no CA-signed certificate, so it can never
complete a handshake - but before 2026-08-21 it did not need to. Discovery
cached whatever answered a probe, and every later recovery attempt consulted
that cache FIRST, so a device that once heard the rogue never looked anywhere
else again. Proven at the time: 5 consecutive recoveries all returned the
rogue while the real server sat reachable on the same LAN.

These tests pin the fix: discovery yields CANDIDATES, the handshake selects,
and only a completed handshake is allowed to write the cache.

    OMEGA_AMU_CONFIG=$PWD/test_fixture.ini py -3.12 test_rogue_server.py
"""

import json
import io
import os
import socket
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import server_discovery as sd

REAL = ("10.99.99.1", 11400)
ROGUE = ("127.0.0.1", 11400)


class _Responder(threading.Thread):
    """Answers discovery probes exactly like a real server would."""

    def __init__(self, claim_ip, claim_port):
        threading.Thread.__init__(self, daemon=True)
        self._claim = (claim_ip, claim_port)
        self._stop_event = threading.Event()
        self.answered = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # DISCOVERY_PORT, not an ephemeral one: probe_reachable() always sends
        # to the fixed discovery port and ignores its own port argument,
        # because the responder is always there regardless of which port
        # telemetry uses. A rogue on any other port would never be probed.
        self.sock.bind(("127.0.0.1", sd.DISCOVERY_PORT))
        self.port = sd.DISCOVERY_PORT
        self.sock.settimeout(0.3)

    def run(self):
        while not self._stop_event.is_set():
            try:
                data, peer = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            if sd.DISCOVERY_MAGIC in data:
                self.answered += 1
                reply = json.dumps({"magic": sd.DISCOVERY_REPLY_MAGIC,
                                    "ip": self._claim[0],
                                    "port": self._claim[1],
                                    "t": int(time.time())}).encode("utf-8")
                try:
                    self.sock.sendto(reply, peer)
                except OSError:
                    pass
        self.sock.close()

    def stop(self):
        self._stop_event.set()
        self.join(timeout=2.0)


def _quiet(*_a, **_k):
    pass


class RogueServerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.cache = os.path.join(self.tmp, "cache.json")

    def test_discovery_never_writes_the_cache(self):
        """The bug that made poisoning permanent: answering a probe was enough
        to be remembered. Only a handshake may write the cache now."""
        sd.find_servers(static_fallback=REAL, cache_path=self.cache, log=_quiet)
        self.assertFalse(os.path.exists(self.cache),
                         "discovery must not cache anything - only a completed "
                         "handshake may, via confirm_server()")

    def test_only_confirm_server_writes_the_cache(self):
        sd.confirm_server(REAL[0], REAL[1], self.cache)
        self.assertEqual(sd.load_cached(self.cache), REAL)

    def test_forget_cached_clears_a_poisoned_entry(self):
        sd.confirm_server(ROGUE[0], ROGUE[1], self.cache)
        self.assertEqual(sd.load_cached(self.cache), ROGUE)
        sd.forget_cached(self.cache)
        self.assertIsNone(sd.load_cached(self.cache),
                          "a cache entry that failed to authenticate must be "
                          "dropped, or the device retries it forever")

    def test_real_server_is_still_offered_when_a_rogue_answers_first(self):
        """The heart of it: a cached rogue must not hide the real server."""
        sd.confirm_server(ROGUE[0], ROGUE[1], self.cache)
        candidates = sd.find_servers(static_fallback=REAL,
                                     cache_path=self.cache, log=_quiet)
        self.assertIn(REAL, candidates,
                      "the real server must remain reachable as a candidate "
                      "even while a rogue address is cached; got %r" % (candidates,))

    def test_candidates_are_deduplicated(self):
        sd.confirm_server(REAL[0], REAL[1], self.cache)
        candidates = sd.find_servers(static_fallback=REAL,
                                     cache_path=self.cache, log=_quiet)
        self.assertEqual(len(candidates), len(set(candidates)),
                         "a repeated address wastes a whole handshake timeout")

    def test_a_rogue_that_answers_probes_is_only_one_candidate(self):
        try:
            rogue = _Responder(*ROGUE)
        except OSError:
            self.skipTest("discovery port busy on this machine")
        rogue.start()
        try:
            sd.confirm_server("127.0.0.1", rogue.port, self.cache)
            alive = sd.probe_reachable("127.0.0.1", rogue.port, timeout_s=1.0)
            candidates = sd.find_servers(static_fallback=REAL,
                                         cache_path=self.cache, log=_quiet)
        finally:
            rogue.stop()
        self.assertTrue(alive, "the rogue must genuinely be answering probes, "
                               "or this test proves nothing")
        self.assertIn(REAL, candidates,
                      "even a rogue that actively answers must not crowd the "
                      "real server out of the candidate list")
        self.assertGreater(rogue.answered, 0,
                           "the rogue must have been probed at least once")


class DiscoveryIsNotSkipped(unittest.TestCase):
    """The 2026-08-26 fault, pinned.

    An AMU carries a configured server address as its FINAL fallback. The
    connect path tried that address before consulting discovery, so on a
    network where it happened to be correct the unit worked perfectly and the
    discovery path never executed - dead code in production, on the feature
    the fleet's DHCP-independence rests on.

    Nothing observable failed. It took a packet capture of a restart, showing
    a handshake with no mDNS query in front of it, to see it at all.
    """

    @staticmethod
    def _shipped(path, func):
        """Read a function out of the shipped file rather than importing it.

        network.py pulls in wolfSSL, which has no build on this machine, so
        importing it here would make the test unrunnable exactly where the
        gate runs. The file on disk is the thing that ships either way.
        """
        import os
        import re
        here = os.path.dirname(os.path.abspath(__file__))
        with io.open(os.path.join(here, path), encoding="utf-8") as handle:
            src = handle.read()
        return re.search(r"^def " + func + r"\(.*?(?=^def |\Z)", src, re.S | re.M).group(0)

    def test_the_first_connection_consults_discovery(self):
        """Structural, because the bug WAS an ordering: no behavioural test
        can see a candidate list that is never asked for."""
        body = self._shipped("network.py", "ensure_session")
        # The bare pre-emptive attempt must be gated on a previous success.
        self.assertNotRegex(
            body, r"\n\s*if client\.connect\(\):",
            "ensure_session() must not try its current address before discovery - "
            "gate it on a handshake having already succeeded")
        self.assertIn("_server_established", body,
                      "the retry shortcut must be gated on a proven address")

    def test_the_configured_address_is_ranked_last(self):
        """Discovery must outrank configuration, or a stale configured address
        silently wins over the server that is actually answering."""
        body = self._shipped("server_discovery.py", "find_servers")
        order = [body.index(k) for k in ("mDNS", "UDP broadcast", "configured fallback")]
        self.assertEqual(order, sorted(order),
                         "candidate order must be mDNS, broadcast, then the "
                         "configured address LAST")



class UnreachableIsNotRejection(unittest.TestCase):
    """A dead link must never be reported as an authentication failure.

    Observed for real on AMU_15, 2026-08-27, while its MAC was blocked:

        DTLS connect failed: OSError: [Errno 101] Network is unreachable
        discovery: 192.168.0.112:11400 answered but failed mutual auth -
                   rejected (total rejected: 6)

    The server never answered. Reported that way it reads as an impostor on
    the network, and the rejection tally - which exists as a SECURITY signal -
    counts events that carry no security meaning at all.
    """

    def _load_rule(self):
        import errno
        import re
        here = os.path.dirname(os.path.abspath(__file__))
        with io.open(os.path.join(here, "network.py"), encoding="utf-8") as handle:
            source = handle.read()
        block = re.search(r"^_UNREACHABLE_ERRNOS.*?(?=^def _try_candidates)",
                          source, re.S | re.M).group(0)

        class _Client(object):
            last_error = None

        namespace = {"errno": errno, "client": _Client}
        exec(compile(block, "network.py", "exec"), namespace)
        return namespace["_failure_was_unreachable"], namespace["client"]

    def test_a_dead_link_is_not_called_an_auth_failure(self):
        check, fake_client = self._load_rule()
        for code in (101, 113, 111, 110):
            fake_client.last_error = OSError(code, "unreachable")
            self.assertTrue(check(),
                            "errno %d means the packet never arrived" % code)

    def test_a_real_auth_failure_is_still_a_rejection(self):
        """The dangerous direction. wolfSSL's SSLError inherits from OSError,
        so a type-based test would silently excuse a genuine impostor."""
        check, fake_client = self._load_rule()
        for failure in (Exception("certificate verify failed"),
                        OSError("handshake failure"),
                        None):
            fake_client.last_error = failure
            self.assertFalse(check(),
                             "%r must still count as a rejection" % failure)

if __name__ == "__main__":
    unittest.main(verbosity=2)
