"""Shared lock serialising ALL wolfSSL socket I/O across threads.

wolfSSL is documented thread-safe for genuinely independent SSLContext
objects, but real segfaults were observed on real hardware (2026-08-17,
Raspberry Pi AMU) when the AMU's own DTLS session to the server
(network.py, in the network_worker thread) and the live-query DTLS
listener (live_server.py, its own thread) touched wolfSSL at the same
moment - two threads, same process, two separate SSLContext/connection
objects.

First attempt locked only the handshake call (wrap_socket()) and reduced
but did NOT eliminate the crash - the second crash happened one step later,
inside a post-handshake recv(). That disproves the narrower theory that
only context/handshake creation is unsafe; treat the ENTIRE wolfSSL
touchpoint - handshake through the last read/write - as needing mutual
exclusion between threads, not just its first step.

Held for the duration of one bounded exchange (a handshake plus a few
kilobytes each way, capped by the callers' own timeouts), not the life of
a session, so contention costs at most a few seconds of delay, never a
deadlock.
"""

import os
import threading
import time

WOLFSSL_LOCK = threading.Lock()

# `with WOLFSSL_LOCK:` waits forever by default - if the main AMU-to-server
# session happens to be mid-retry (send_and_ack can legitimately hold this
# lock for several seconds), an unbounded wait made a live-query attempt
# hang indefinitely with no error, no crash, just silence. Every acquisition
# must use this bound instead of a bare `with`, so contention fails fast and
# visibly (the caller can just retry) rather than hanging.
LOCK_ACQUIRE_TIMEOUT_S = 3.0

# Last-resort watchdog (added 2026-08-20 after a real field wedge - see
# FINDINGS #32). Every legitimate hold of this lock is bounded by the
# holder's own timeouts: a live query is capped by live_server's handshake
# (8s) plus one bounded read (10s), and a telemetry exchange by
# dtls_client's own per-read timeouts. So the lock being CONTINUOUSLY
# unavailable for minutes cannot be contention - it means a thread died or
# parked in the kernel while holding it, and a threading.Lock cannot be
# safely force-released from outside its owner.
#
# For an unattended field device the only recovery left is to restart the
# process, which systemd does immediately (Restart=always, and
# StartLimitIntervalSec=0 so it can never permanently give up). The offline
# buffer is on disk, so a restart costs no data. Left un-fixed, the failure
# is far worse and completely silent: the AMU keeps sampling, keeps logging
# "wolfSSL busy", and never delivers another reading until a human notices.
LOCK_STUCK_RESTART_S = 120.0

_stuck_since = None
_stuck_guard = threading.Lock()


def note_lock_acquired():
    """Call after any SUCCESSFUL acquisition - proves the lock still cycles."""
    global _stuck_since
    with _stuck_guard:
        _stuck_since = None


def note_lock_timeout(who):
    """Call after any FAILED (timed-out) acquisition. Restarts the process if
    the lock has been continuously unavailable long enough that a wedged
    holder is the only remaining explanation."""
    global _stuck_since
    with _stuck_guard:
        now = time.time()
        if _stuck_since is None:
            _stuck_since = now
            return
        stuck_for = now - _stuck_since
        if stuck_for < LOCK_STUCK_RESTART_S:
            return
    print("FATAL: wolfSSL lock held by a wedged thread for %.0fs (%s cannot "
          "proceed) - restarting so systemd recovers the unit; buffered data "
          "is on disk and survives" % (stuck_for, who), flush=True)
    os._exit(1)
