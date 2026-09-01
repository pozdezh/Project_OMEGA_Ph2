"""Brick 4 AMU transport - a DTLS 1.3 client session, hardware-independent.

Same contract as brick3's client (connect / send_and_ack / close), so
sensor_manager keeps calling it unchanged. Four things are different, each one
paid for by a measured failure:

  1. DTLS 1.3 over wolfSSL instead of DTLS 1.2 over python3-dtls. 1.3 derives
     keys before certificates are exchanged, so the device identity is not
     visible in cleartext at every handshake.

  2. SESSION REUSE. The session stays open across many records instead of one
     handshake per record. Measured 96-116 records per handshake over an hour.

  3. RESEND BEFORE RECONNECT. A lost datagram does NOT mean the session is
     broken - the key is still valid and the peer is still authenticated - so
     the record is re-sent on the SAME session up to MAX_RECORD_ATTEMPTS times.
     Before this, one lost packet forced a full re-handshake; in the 1-hour
     soak that accounted for 46% of the NMU's handshakes. After: 37 resend
     attempts, 31 recovered on attempt 2, only 2 records lost out of 7200.

  4. select()-GATED READS. wolfSSL's own DTLS receive callback rewrites the
     socket's SO_RCVTIMEO to {0,0} - "block forever" - as soon as the
     handshake completes, so a socket timeout silently stops working and one
     lost datagram wedges the client permanently. select() keeps its own clock
     and is immune. See LOGS/KEY_ROTATION_AND_AMU_HANG_INVESTIGATION.md.

The server address is DISCOVERED, never hardcoded - see server_discovery.
"""

import json
import os
import select
import socket
import struct
import time

import wolfssl

import wolfssl_guard
from wolfssl_guard import WOLFSSL_LOCK, LOCK_ACQUIRE_TIMEOUT_S

HANDSHAKE_TIMEOUT_S = 10.0
ACK_TIMEOUT_S = 5.0
RECV_BYTES = 8192
MAX_RECORD_ATTEMPTS = 3

# A session quiet longer than this is presumed dead and re-handshaked before
# the next send. UDP never reports that the far end hung up, so a session the
# server already closed still looks perfectly healthy from this side - and is
# only discovered dead by sending into the void and burning every retry
# attempt first (3 x ACK_TIMEOUT_S = 15 s per record). The server closes an
# idle session at 2.5x the heartbeat it handed us (server/session.py:
# idle_timeout_for), so expiring at 2.0x always acts first. Updated from the
# ACK's heartbeat by set_stale_after(); this is only the pre-first-ACK default.
DEFAULT_STALE_AFTER_S = 120.0


class RecordTimeout(Exception):
    """No datagram arrived within the deadline."""


def _set_recv_timeout(sock, seconds):
    """Kernel-level timeout, honoured during the HANDSHAKE only.

    NOT socket.settimeout(), which switches the socket to non-blocking and
    breaks wolfSSL's DTLS handshake outright. And not relied on afterwards -
    wolfSSL overwrites it the moment the handshake finishes.
    """
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO,
                    struct.pack("ll", int(seconds), 0))


class DtlsClient:
    """One reusable DTLS 1.3 session to the Omega server. Not thread-safe; the
    AMU drives it from its single network_worker thread."""

    def __init__(self, pki_dir, device_id, server_host, port,
                 handshake_timeout_s=HANDSHAKE_TIMEOUT_S,
                 ack_timeout_s=ACK_TIMEOUT_S, log=print):
        self._pki_dir = pki_dir
        self._device_id = device_id
        self._server = (server_host, int(port))
        self._handshake_timeout_s = handshake_timeout_s
        self._ack_timeout_s = ack_timeout_s
        self._log = log
        self._raw = None
        self._conn = None
        self._ctx = None
        self._last_exchange = 0.0
        self._stale_after_s = DEFAULT_STALE_AFTER_S
        # An operator question the server piggybacked on an ACK
        # (server/acks.py's "q"), waiting to be handed to the caller.
        self._pending_question = None
        # Counters the caller (and the memo) can read.
        self.handshakes = 0
        self.resend_attempts = 0
        self.recovered_by_resend = 0
        self.records_lost = 0
        self.reauths = 0
        self.last_error = None

    # ------------------------------------------------------------- lifecycle

    def server_address(self):
        return self._server

    def set_server(self, host, port):
        """Point at a (re)discovered server. Drops any open session."""
        new_server = (str(host), int(port))
        if new_server != self._server:
            self._log("transport: server address now %s:%d" % new_server)
            self.close()
            self._server = new_server

    def set_stale_after(self, seconds):
        """Track the server's idle policy, which is derived from the heartbeat
        it hands us in every ACK. Called by the caller on each applied config
        so the two ends can never silently drift apart."""
        if seconds > 0:
            self._stale_after_s = float(seconds)

    def connected(self):
        """True only for a session that is BOTH open and not presumed stale.

        Reporting a stale session as disconnected is what makes the caller
        re-handshake before sending, instead of after wasting every retry -
        see DEFAULT_STALE_AFTER_S.
        """
        if self._conn is None:
            return False
        if self._last_exchange and (
                time.time() - self._last_exchange) > self._stale_after_s:
            self._log("session idle %.0fs - presumed closed by server, reconnecting"
                      % (time.time() - self._last_exchange))
            self.close()
            return False
        return True

    def _build_context(self):
        """Build (once) this client's wolfSSL context, under the shared lock.

        Context creation is a wolfSSL touchpoint like any other and must not
        overlap the live-query thread doing the same - see live_server.py's
        _build_context for the incident that proved it.
        """
        if self._ctx is not None:
            return self._ctx
        if not WOLFSSL_LOCK.acquire(timeout=LOCK_ACQUIRE_TIMEOUT_S):
            wolfssl_guard.note_lock_timeout("telemetry context build")
            raise RuntimeError("wolfSSL busy - context build deferred")
        wolfssl_guard.note_lock_acquired()
        try:
            ctx = wolfssl.SSLContext(wolfssl.PROTOCOL_DTLSv1_3, server_side=False)
            ctx.load_cert_chain(
                os.path.join(self._pki_dir, self._device_id + "-cert.pem"),
                os.path.join(self._pki_dir, self._device_id + "-key.pem"),
            )
            ctx.load_verify_locations(os.path.join(self._pki_dir, "ca-cert.pem"))
            ctx.verify_mode = wolfssl.CERT_REQUIRED
        finally:
            WOLFSSL_LOCK.release()
        self._ctx = ctx
        return ctx

    def prepare(self):
        """Build the wolfSSL context up front, on the MAIN thread.

        THIS MUST BE CALLED FROM THE MAIN THREAD, BEFORE ANY WORKER STARTS.
        wolfSSL_Init() succeeds only on the main thread of this process:
        proved on the live AMU on 2026-08-25 with a two-line experiment -
        the identical SSLContext() call returned "wolfSSL library
        initialization failed" from a worker thread and succeeded from the
        main thread, in the same process, seconds apart.

        The context is cached once built, so this is the only moment that has
        to be on the main thread; every later handshake reuses it and may run
        anywhere. Touches no socket, so a server that is down at boot cannot
        stop it.
        """
        try:
            self._build_context()
            return True
        except Exception as error:
            self._log("DTLS context build failed: %s: %s"
                      % (type(error).__name__, error))
            return False

    def connect(self):
        """Open the socket and run the mutual-auth handshake. Returns True on
        success; on failure leaves the client disconnected so the caller can
        buffer and rediscover, exactly as before."""
        self.close()
        try:
            ctx = self._build_context()
            self._raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _set_recv_timeout(self._raw, self._handshake_timeout_s)
            self._raw.connect(self._server)
            # Bounded, not `with WOLFSSL_LOCK:` - a live-query session on the
            # AMU can legitimately hold this lock for several seconds (see
            # live_server.py), and an unbounded wait here once hung the whole
            # network_worker thread with no error. Busy just means retry.
            got_lock = WOLFSSL_LOCK.acquire(timeout=LOCK_ACQUIRE_TIMEOUT_S)
            if not got_lock:
                wolfssl_guard.note_lock_timeout("telemetry handshake")
                self._log("DTLS connect: wolfSSL busy with a live query - retry next cycle")
                self.close()
                return False
            wolfssl_guard.note_lock_acquired()
            try:
                self._conn = ctx.wrap_socket(self._raw, server_side=False)
            finally:
                WOLFSSL_LOCK.release()
            self.handshakes += 1
            self._last_exchange = time.time()
            return True
        except Exception as error:
            # Kept so the CALLER can say why. Without it every failure read as
            # "answered but failed mutual auth", including a dead link - which
            # points a security investigation at an impostor that was never
            # there. See network.last_failure_was_unreachable().
            self.last_error = error
            self._log("DTLS connect failed: %s: %s" % (type(error).__name__, error))
            self.close()
            return False

    def close(self):
        for closeable in ("_conn", "_raw"):
            handle = getattr(self, closeable, None)
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
                setattr(self, closeable, None)

    # ----------------------------------------------------------------- io

    def _recv_with_timeout(self, timeout_s):
        """Read one record with a timeout that actually works - see module
        docstring point 4.

        select() alone is NOT sufficient. It reports that a datagram arrived,
        but wolfSSL may find that datagram incomplete for the record it is
        assembling and go straight back to the socket for more - and by then
        it has already reset SO_RCVTIMEO to "block forever", so that second
        read never returns and the whole worker thread parks silently. So the
        kernel timeout is re-armed immediately before every read: whatever
        wolfSSL did to it, the read is bounded and surfaces as RecordTimeout,
        which the retry path already knows how to handle.
        """
        _set_recv_timeout(self._raw, timeout_s)
        try:
            if self._conn.pending() > 0:
                return self._conn.recv(RECV_BYTES)
        except Exception:
            pass
        ready, _, _ = select.select([self._raw], [], [], timeout_s)
        if not ready:
            raise RecordTimeout("no ACK within %.1f s" % timeout_s)
        _set_recv_timeout(self._raw, timeout_s)
        return self._conn.recv(RECV_BYTES)

    def take_question(self):
        """Hand over an operator question that arrived on an ACK, or None.

        Pure state read - deliberately touches NO socket and NO wolfSSL
        object, so it is safe to call from the sampling loop while the
        network worker owns the session.
        """
        queued = self._pending_question
        self._pending_question = None
        return queued

    def send_and_ack(self, record):
        """Send one telemetry record and return the server's parsed ACK dict,
        or None if no valid ACK arrived after all attempts. Holds WOLFSSL_LOCK
        for its full duration - see wolfssl_guard.py: the crash this closes
        happened inside this method's own recv(), not just at handshake time.

        The acquisition is BOUNDED, not a bare `with`: if a live-query session
        (live_server.py) is mid-exchange and holding the lock, this returns
        None (record stays buffered, tried again next cycle) instead of
        blocking the whole network_worker thread indefinitely. The session
        itself is left open - unlike a real transport error, there is nothing
        wrong with it, so it is NOT closed here.
        """
        got_lock = WOLFSSL_LOCK.acquire(timeout=LOCK_ACQUIRE_TIMEOUT_S)
        if not got_lock:
            wolfssl_guard.note_lock_timeout("telemetry send")
            self._log("DTLS send: wolfSSL busy with a live query - buffering record")
            return None
        wolfssl_guard.note_lock_acquired()
        try:
            return self._send_and_ack_locked(record)
        finally:
            WOLFSSL_LOCK.release()

    def _send_and_ack_locked(self, record):
        """Send one telemetry record and return the server's parsed ACK dict
        (carrying heartbeat config, server time, and possibly reauth), or None
        if no valid ACK arrived after all attempts.

        A None return means the caller should buffer and reconnect. Note this
        is now RARE: a lost datagram is retried on the live session first.
        """
        if not self.connected():
            return None

        payload = json.dumps(record).encode("utf-8")
        want = str(record.get("event"))

        for attempt in range(1, MAX_RECORD_ATTEMPTS + 1):
            try:
                self._conn.send(payload)
                data = self._recv_with_timeout(self._ack_timeout_s)
            except RecordTimeout:
                # Silence only. The session key is still valid and the peer is
                # still authenticated, so re-send the SAME record rather than
                # paying for a fresh handshake. The event id is unchanged, so
                # if it was the ACK that was lost the server sees a duplicate
                # and its (id, event) index absorbs it.
                self.resend_attempts += 1
                continue
            except Exception as error:
                # A real transport/TLS error: the session is genuinely unusable.
                self._log("DTLS session error: %s" % type(error).__name__)
                self.close()
                return None

            try:
                ack = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                self.resend_attempts += 1
                continue

            # An operator question rides on an ACK we were already waiting
            # for (server/acks.py's "q"). Capturing it HERE - rather than
            # reading the socket at some other moment - is what keeps every
            # wolfSSL read on this session a reply the device asked for.
            # Reading speculatively blocks inside wolfSSL and starves the
            # sender; that was measured on hardware, see FINDINGS #32.
            if ack.get("q") is not None:
                self._pending_question = ack["q"]

            if str(ack.get("ack")) != want:
                # An ACK for a different record (a late duplicate). Not fatal;
                # keep waiting for ours within the remaining attempts.
                self.resend_attempts += 1
                continue

            self._last_exchange = time.time()
            if attempt > 1:
                self.recovered_by_resend += 1
                self._log("record %s recovered on attempt %d, session kept"
                          % (want, attempt))
            if ack.get("reauth"):
                # Server's session-age policy expired: it wants our certificate
                # checked again. Close now; the caller reconnects on the next
                # cycle. This is the revocation checkpoint.
                self.reauths += 1
                self._log("server requested re-auth - closing session")
                self.close()
            return ack

        self.records_lost += 1
        self._log("record %s LOST after %d attempts - dropping session"
                  % (want, MAX_RECORD_ATTEMPTS))
        self.close()
        return None

    def stats(self):
        return {
            "handshakes": self.handshakes,
            "resend_attempts": self.resend_attempts,
            "recovered_by_resend": self.recovered_by_resend,
            "records_lost": self.records_lost,
            "reauths": self.reauths,
        }
