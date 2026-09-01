"""Brick 4 AMU live command server - direct MCP-to-device queries.

Migrated to wolfSSL DTLS 1.3, mirroring server/listener.py's proven
server-mode setup - see FINDINGS #11. The prior implementation targeted
python3-dtls (DTLS 1.2), a library retired by this project's transport
migration and never installed anywhere in the brick4 dependency set, so
this feature crashed on boot until now.

The AMU is mains-powered and always on, so unlike the battery NMU it can
accept INBOUND connections. This module runs a small DTLS 1.3 listener on
the AMU that an operator (the MCP tool on a laptop, holding a CA-signed
operator cert) connects to DIRECTLY - no server REST API, no store-and-
forward. The operator sends a deterministic command and gets a fresh reply
over the encrypted, mutual-authenticated channel.

Design goal (for the paper): the LLM only maps an informal prompt to ONE
typed tool call; everything below that is deterministic and auditable -
the command protocol here is a fixed dictionary, not free text.

Only certificates whose Common Name is in `allowed_operators` may issue
commands, so one device can never command another. Readings are supplied by
an injected read_fn (in production, the latest cached sample from main.py -
never a second GPIO reader, which previously caused a hardware conflict),
so this module is fully unit-testable with no sensors.

KNOWN LIMITATION (field-verified 2026-08-17): two live-query attempts that
truly overlap (arrive within the same instant, not one-after-another) can
collide during the handshake and one will fail - a lower-level timing issue
in how two connection attempts briefly share one UDP port, separate from
the WOLFSSL_LOCK contention issue fixed the same day. Not chased further
because it does not affect the sensor pipeline (see wolfssl_guard.py) and
does not occur in the actual usage pattern: an MCP client (Claude Desktop)
sends one tool call, waits for its result, then sends the next - it cannot
issue two live-query calls at the same instant. A failed query is safe to
just retry.
"""

import json
import os
import select
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import wolfssl_guard
from wolfssl_guard import WOLFSSL_LOCK, LOCK_ACQUIRE_TIMEOUT_S

LIVE_PORT = 5001
RECV_BYTES = 8192
ACCEPT_TIMEOUT_S = 1.0
HANDSHAKE_TIMEOUT_S = 8.0
SESSION_TIMEOUT_S = 10.0
# How long a caller waits for the handshake lock before being declined.
# Deliberately short: the operator's own call has a 3s budget on the server
# side, so a longer wait here would only produce an answer nobody is still
# listening for. Declining fast also stops threads accumulating.
HANDSHAKE_LOCK_WAIT_S = 2.0
# A real handshake on this LAN measures 1.1-1.4s (40-query run, 2026-08-21),
# so 8s is still a ~6x margin over the worst legitimate case. It was 20s
# first, which was too generous in the wrong direction: a wedged handshake
# cannot be interrupted, so the threshold is not "how long might a handshake
# take" but "how long may the listener stay unreachable". Restart costs ~2s
# and loses nothing, so a tighter bound is strictly better here.
HANDSHAKE_STUCK_S = 8.0
HANDSHAKE_WATCHDOG_TICK_S = 2.0
# How long a peer stays remembered after its session ends, so late DTLS
# retransmissions cannot be mistaken for a new caller. Comfortably longer
# than any client's own retransmission window, far shorter than the gap
# between real operator questions.
PEER_COOLDOWN_S = 30.0
DEFAULT_OPERATORS = ("operator",)

CMD_READ_NOW = "read_now"
CMD_STATUS = "status"


def handle_command(command, device_id, read_fn):
    """Pure, deterministic command dispatch. Returns the reply dict.

    Commands (fixed vocabulary, not free text):
      {"cmd": "read_now"} -> latest sensor reading
      {"cmd": "status"}   -> device liveness
    Anything else is an explicit error.
    """
    cmd = str(command.get("cmd", ""))
    if cmd == CMD_READ_NOW:
        return {"ok": True, "device_id": device_id, "ts": int(time.time()),
                "reading": read_fn()}
    if cmd == CMD_STATUS:
        return {"ok": True, "device_id": device_id, "ts": int(time.time()),
                "status": "online"}
    return {"ok": False, "device_id": device_id, "error": "unknown command: " + cmd}


def _set_recv_timeout(sock, seconds):
    """Kernel-level timeout, honoured during the HANDSHAKE only - same
    device_live.py/dtls_client.py pattern. Without this, a stalled or
    incomplete handshake attempt (a lost datagram, a bad actor sending one
    stray packet) blocks ctx.wrap_socket() forever - and since _serve_one()
    runs inline (not threaded), that wedges the WHOLE listener: every later
    caller's packets pile up unread on the socket while this one handshake
    never finishes. Confirmed live (2026-08-17): three back-to-back real
    queries all failed once an earlier handshake stalled, because nothing
    was draining serve_forever()'s accept loop anymore."""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO,
                    struct.pack("ll", int(seconds), 0))


def _peer_common_name(conn):
    peer_cert = conn.getpeercert()
    if not peer_cert:
        return None
    for rdn in peer_cert.get("subject", ()):
        for key, value in rdn:
            if key == "commonName":
                return value
    return None


def _build_context(pki_dir, device_id):
    # Lazy import: handle_command() (unit-tested directly, no sockets) must
    # stay importable on any machine, regardless of whether wolfssl happens
    # to be installed there - only actually SERVING a connection needs it.
    import wolfssl
    # Under the lock like every other wolfSSL touchpoint. Building a context
    # is NOT exempt: this runs once at thread start, and main.py starts this
    # thread ~20 s after the network worker - which lands almost exactly on
    # the worker's first handshake. Unprotected, that collision wedged the
    # worker in a blocking socket read forever (2026-08-17: AMU queued data
    # for 6 minutes and delivered none, no error logged, thread parked in
    # __skb_wait_for_more_packets).
    with WOLFSSL_LOCK:
        ctx = wolfssl.SSLContext(wolfssl.PROTOCOL_DTLSv1_3, server_side=True)
        ctx.load_cert_chain(
            os.path.join(pki_dir, device_id + "-cert.pem"),
            os.path.join(pki_dir, device_id + "-key.pem"),
        )
        ctx.load_verify_locations(os.path.join(pki_dir, "ca-cert.pem"))
        ctx.verify_mode = wolfssl.CERT_REQUIRED
    return ctx


class _NullLock:
    """Stand-in used when this listener owns its process outright.

    WOLFSSL_LOCK and its watchdog exist to stop this listener and the AMU's
    telemetry client from touching wolfSSL at the same moment INSIDE ONE
    PROCESS. In live_agent.py there is no telemetry client to collide with -
    this process has exactly one wolfSSL user - so the lock protects nothing.

    It is not merely redundant there, it is harmful. A handshake that blocks
    holds the real lock, every later query then fails to acquire it, and
    wolfssl_guard's watchdog concludes a thread has wedged and calls
    os._exit(1) to let systemd recover. Measured 2026-08-21: one slow
    handshake took the whole agent down, the port closed, and every
    subsequent query failed instantly against nothing. A slow query must
    degrade into a slow query, never into a dead service."""

    def acquire(self, timeout=None):
        return True

    def release(self):
        return None


class LiveCommandServer:
    def __init__(self, pki_dir, device_id, read_fn, port=LIVE_PORT,
                 allowed_operators=DEFAULT_OPERATORS, serialize=True):
        self._pki_dir = pki_dir
        self._device_id = device_id
        self._read_fn = read_fn
        self._port = port
        self._allowed = set(allowed_operators)
        self._stop = False
        # serialize=True keeps the historical in-process behaviour for any
        # caller that still shares a process with the telemetry client.
        self._lock = WOLFSSL_LOCK if serialize else _NullLock()
        self._serialize = serialize
        # Handshakes are serialised even when this listener owns its process.
        #
        # Not defensive programming - this is a measured crash. Every session
        # is wrapped from ONE shared SSLContext, and wolfSSL's Python binding
        # does not guard that object against concurrent use. With a thread per
        # caller, faulthandler caught four threads inside do_handshake() at the
        # instant of a SIGSEGV (2026-08-21):
        #
        #   Fatal Python error: Segmentation fault
        #   Current thread ... wolfssl/__init__.py line 813 in do_handshake
        #   Thread ...       ... wolfssl/__init__.py line 813 in do_handshake
        #   Thread ...       ... wolfssl/__init__.py line 813 in do_handshake
        #   Thread ...       ... wolfssl/__init__.py line 813 in do_handshake
        #
        # This lock is separate from WOLFSSL_LOCK on purpose: it provides the
        # mutual exclusion without wolfssl_guard's watchdog, which used to kill
        # the whole agent whenever a handshake was merely slow.
        self._handshake_lock = threading.Lock()
        # When the current handshake started, or None. Read by the stuck
        # handshake watchdog below.
        self._handshake_started_at = None

    def _serve_one(self, ctx, raw, peer):
        """Complete one handshake and answer exactly one command on a freshly
        connected per-peer socket - the same recvfrom-then-connect() UDP demux
        pattern proven in server/listener.py, which avoids the MSG_PEEK bug
        documented there. Single operator at a time is the expected MCP usage
        pattern, so serving inline (no per-peer thread) keeps this simple.

        Holds WOLFSSL_LOCK ONLY around the two actual wolfSSL touchpoints -
        the handshake, and the later recv/send - NOT across the idle wait in
        between for the operator to actually send a command. That wait is a
        plain select() on the raw OS socket; it never calls into wolfSSL, so
        it does not need the lock. This split is itself a fix for a real
        incident (2026-08-17): holding the lock across the FULL up-to-10s
        wait starved the AMU's own main telemetry session, which lost 5
        straight ACKs, forced a soft-network reset, and crashed the process.
        Each of the two locked sections is still BOUNDED (LOCK_ACQUIRE_TIMEOUT_S)
        - an earlier unbounded `with WOLFSSL_LOCK:` hung this method outright
        whenever the main session was mid-retry and already holding it."""
        conn = None
        try:
            got_lock = self._lock.acquire(timeout=LOCK_ACQUIRE_TIMEOUT_S)
            if not got_lock:
                if self._serialize:
                    wolfssl_guard.note_lock_timeout("live query handshake")
                print("live: AMU's own session was busy - try again shortly")
                return
            if self._serialize:
                wolfssl_guard.note_lock_acquired()
            try:
                # ONE handshake at a time, always. Concurrent wrap_socket()
                # calls against the shared SSLContext segfault the process -
                # caught in the act by faulthandler, see __init__.
                if not self._handshake_lock.acquire(timeout=HANDSHAKE_LOCK_WAIT_S):
                    print("live: another handshake is in progress - "
                          "declining %s, retry is safe" % (peer,))
                    return
                self._handshake_started_at = time.time()
                try:
                    # SO_RCVTIMEO is set as a best effort only. It genuinely
                    # cannot bound this call: wolfSSL rewrites that option
                    # from inside its own receive callback (FINDINGS #3), so
                    # a client that walks away mid-handshake leaves this
                    # thread blocked in the kernel with no way to interrupt
                    # it from Python. The watchdog below is what actually
                    # bounds it.
                    _set_recv_timeout(raw, HANDSHAKE_TIMEOUT_S)
                    conn = ctx.wrap_socket(raw, server_side=True)
                finally:
                    self._handshake_started_at = None
                    self._handshake_lock.release()
            finally:
                self._lock.release()

            operator = _peer_common_name(conn)
            if operator not in self._allowed:
                print("live: refused operator identity " + str(operator))
                return

            # NOT raw.settimeout(): that switches the socket to non-blocking
            # and breaks wolfSSL's DTLS handshake outright. select() keeps its
            # own clock and is immune to wolfSSL rewriting SO_RCVTIMEO, so it
            # gates the wait here. Lock-free window - see docstring above.
            ready, _, _ = select.select([raw], [], [], SESSION_TIMEOUT_S)
            if not ready:
                print("live: no command received within %.0fs" % SESSION_TIMEOUT_S)
                return

            got_lock = self._lock.acquire(timeout=LOCK_ACQUIRE_TIMEOUT_S)
            if not got_lock:
                if self._serialize:
                    wolfssl_guard.note_lock_timeout("live query reply")
                print("live: AMU's own session was busy - dropping this reply")
                return
            if self._serialize:
                wolfssl_guard.note_lock_acquired()
            try:
                # select() above is NOT sufficient on its own, and assuming it
                # was wedged this whole listener permanently (2026-08-20): it
                # only proves ONE datagram arrived, but wolfSSL may find that
                # datagram incomplete for the record it is assembling and go
                # straight back to the socket for more - and by then it has
                # reset SO_RCVTIMEO to {0,0}, "block forever". That second
                # read never returns, this thread parks in the kernel holding
                # WOLFSSL_LOCK, and the AMU's own telemetry session can then
                # never acquire it again ("wolfSSL busy with a live query"
                # forever). Re-arming the kernel timeout immediately before
                # the read bounds it whatever wolfSSL did - the identical fix
                # dtls_client._recv_with_timeout() already carries.
                _set_recv_timeout(raw, SESSION_TIMEOUT_S)
                data = conn.recv(RECV_BYTES)
                command = json.loads(data.decode("utf-8"))
                reply = handle_command(command, self._device_id, self._read_fn)
                conn.send(json.dumps(reply).encode("utf-8"))
                print("live: %s -> %s" % (command.get("cmd"), reply.get("ok")))
            finally:
                self._lock.release()
        except (ValueError, UnicodeDecodeError) as error:
            print("live: malformed command: " + str(error))
        except Exception as error:
            print("live: session error (%s): %s" % (peer, error))
        finally:
            for closeable in (conn, raw):
                try:
                    if closeable is not None:
                        closeable.close()
                except Exception:
                    pass

    def _watch_stuck_handshake(self):
        """Restart the process if one handshake has held the lock too long.

        This is a watchdog, and an earlier version of this file had one
        removed for good reason - wolfssl_guard's version killed the agent
        whenever a handshake was merely SLOW. This one is different in the
        two ways that matter: its threshold is far above any legitimate
        handshake (measured at 1.1-1.4s on this LAN), and it exists to escape
        a state that is genuinely unrecoverable rather than merely slow.

        The unrecoverable state: wrap_socket() cannot be bounded, because
        wolfSSL rewrites the socket receive timeout from inside its own
        receive callback. A client that gives up mid-handshake - which the
        server's own 3s budget guarantees will happen sometimes - leaves this
        thread blocked in the kernel forever, holding the handshake lock.
        Python cannot kill a thread stuck in a syscall, so every later caller
        is declined for as long as the process lives. Measured 2026-08-21:
        one abandoned handshake declined every subsequent query indefinitely.

        Restarting is cheap and safe here in a way it is not for the
        telemetry service: this agent holds no state, answers from a cache
        another process owns, and systemd brings it back in about two
        seconds. A permanent stall is strictly worse than a two-second gap."""
        while not self._stop:
            time.sleep(HANDSHAKE_WATCHDOG_TICK_S)
            started = self._handshake_started_at
            if started is None:
                continue
            held = time.time() - started
            if held < HANDSHAKE_STUCK_S:
                continue
            print("FATAL: a handshake has been stuck for %.0fs holding the "
                  "handshake lock - it cannot be interrupted from Python, so "
                  "every later query would be declined forever. Restarting; "
                  "systemd brings this agent back in about 2s and no data is "
                  "lost (the sample cache belongs to another process)."
                  % held, flush=True)
            os._exit(1)

    def _serve_and_release(self, ctx, peer_sock, peer, touch):
        try:
            self._serve_one(ctx, peer_sock, peer)
        finally:
            # Refresh rather than forget - see serve_forever's cooldown note.
            touch(peer)

    def serve_forever(self):
        """Accept loop mirroring server/listener.py, which is the version of
        this pattern proven over real UDP for weeks.

        An earlier version here served each peer INLINE. That looks simpler -
        only one operator queries at a time - but it stops draining the
        listening socket for the duration of a session, so the client's DTLS
        handshake retransmissions pile up in the kernel queue. The next accept
        then pops a stale datagram, opens a connected socket for a peer that
        has already gone, and blocks on a handshake that can never complete.
        Measured on real hardware 2026-08-21: query 1 answered in 1.39s, query
        2 burned 128s and failed, and every query after that failed instantly.

        Handing each peer to its own thread fixes it at the source: the accept
        loop returns immediately and never stops draining. known_peers then
        absorbs the strays it catches before the kernel finishes routing a
        peer to its own socket - without it, each stray would spawn another
        socket for a peer already being served (the failure mode that once
        exhausted the server's file descriptors, FINDINGS #5).

        This is deliberately a copy of a working design rather than a new one.
        FINDINGS #5's closing line was that this loop had been reinvented
        instead of read; doing that twice would be careless."""
        ctx = _build_context(self._pki_dir, self._device_id)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self._port))
        sock.settimeout(ACCEPT_TIMEOUT_S)
        print("AMU live command server up on UDP " + str(self._port))

        threading.Thread(target=self._watch_stuck_handshake,
                         daemon=True).start()

        # peer -> time it was last seen. NOT a set that forgets a peer the
        # moment its session ends, which is what this was first written as.
        #
        # A DTLS client keeps retransmitting until it is satisfied, so one or
        # two datagrams from a finished session routinely arrive AFTER it
        # closed. Forgetting immediately made each of those look like a brand
        # new caller: a thread was spawned to handshake with a client that had
        # already gone, and since wrap_socket() cannot be interrupted, that
        # thread stuck forever holding the handshake lock. Measured 2026-08-21
        # at a realistic 20s query spacing - all 10 queries answered, and all
        # 10 followed by a stuck handshake and a watchdog restart.
        #
        # Keeping the peer for a cooldown makes those strays cost nothing.
        # Real callers are unaffected: each new query comes from a fresh
        # ephemeral source port, so it is a different peer entry.
        recent_peers = {}
        peers_lock = threading.Lock()

        def touch(peer):
            with peers_lock:
                recent_peers[peer] = time.time()

        while not self._stop:
            try:
                _, peer = sock.recvfrom(RECV_BYTES)
            except socket.timeout:
                continue
            except OSError:
                continue

            now = time.time()
            with peers_lock:
                for stale in [p for p, seen in recent_peers.items()
                              if now - seen > PEER_COOLDOWN_S]:
                    del recent_peers[stale]
                if peer in recent_peers:
                    # In-flight, or a straggler from a session that just
                    # ended. Either way it must not start a new handshake.
                    recent_peers[peer] = now
                    continue
                recent_peers[peer] = now

            peer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            peer_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                peer_sock.bind(("0.0.0.0", self._port))
                peer_sock.connect(peer)
            except OSError as error:
                print("live: could not open per-peer socket for %s: %s" % (peer, error))
                peer_sock.close()
                touch(peer)
                continue

            threading.Thread(target=self._serve_and_release,
                             args=(ctx, peer_sock, peer, touch),
                             daemon=True).start()
        sock.close()

    def stop(self):
        self._stop = True
