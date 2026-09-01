"""Accept loop and per-peer socket plumbing for the brick4 DTLS 1.3 listener.

Same job as brick3's listener, three things changed, all proven by the
1-hour soak of 2026-08-15 (see dtls13_spike/LOGS/SOAK_1HOUR_RESULTS.md):

  1. wolfSSL DTLS 1.3 instead of python3-dtls DTLS 1.2. 1.3 derives keys
     BEFORE certificates are sent, so the device identity is no longer
     visible in cleartext on the wire at every handshake - the identity
     exposure gap 1.2 has. It is also the only stack that has ever completed
     a handshake on the real NMU hardware; the mbedTLS 1.2 path crashes the
     board (LOGS/OPEN_ISSUE_dtls12_nmu_panic.md).

  2. SESSION REUSE. One handshake, then many records over the same session
     (measured: 96-116 records per handshake). The NMU's handshake costs
     ~5 s, so a handshake per event cannot keep up with a 2 Hz event rate;
     amortising it is what makes the device viable at all.

  3. Session-age policy with jitter. The server closes any session past
     ~60 s (+/-20% random) and makes the device re-handshake. That
     re-handshake is the ONLY point a certificate is re-verified, so it is
     the only place revocation can actually be enforced. The jitter stops a
     whole fleet re-handshaking in lockstep, which would turn routine policy
     into a self-inflicted load spike.

This module owns socket-level plumbing only: accepting new peers, building
each one its own connected socket, and building the TLS context. Everything
about a session's contents lives in session.py; it must never parse telemetry.
"""

import os
import queue
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config_store
import discovery
import session
import storage

import wolfssl

UDP_IP = "0.0.0.0"
UDP_PORT = int(os.environ.get("OMEGA_PORT", "5000"))
RECV_BYTES = 8192
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(BASE_DIR, "sensor_data.db")
DEVICE_CONFIG_PATH = os.environ.get(
    "OMEGA_DEVICE_CONFIG", os.path.join(BASE_DIR, "device_config.json"))
PKI_DIR = os.environ.get("OMEGA_PKI_DIR", os.path.join(BASE_DIR, "pki"))
# The NMU never accepts inbound connections (see nmu_mailbox.py) - an
# operator's question rides the next ACK out and the answer rides the
# device's own next record back in. Separate small SQLite file, never inside
# sensor_data.db - this is control-plane state, not telemetry.
MAILBOX_DB_PATH = os.environ.get(
    "OMEGA_MAILBOX_DB", os.path.join(BASE_DIR, "nmu_mailbox.db"))
# First certificate seen per device name - see ARCHITECTURE.md 17.
IDENTITY_DB_PATH = os.environ.get(
    "OMEGA_IDENTITY_DB", os.path.join(BASE_DIR, "device_identities.json"))
# Where each device was last seen, learned from its own handshakes. Not a
# credential and not a trust decision - only somewhere to place an outbound
# live call, which is itself mutually authenticated.
ADDRESS_DB_PATH = os.environ.get(
    "OMEGA_ADDRESS_DB", os.path.join(BASE_DIR, "device_addresses.json"))

# The handshake runs on its own per-peer thread (never the accept thread), so
# one silent peer cannot stall any other device. Passed through to
# handle_peer() but NOT enforced with a socket timeout - see FINDINGS #12:
# raw.settimeout() puts the socket in non-blocking mode and broke every real
# handshake. The janitor below bounds it from the outside instead.
HANDSHAKE_TIMEOUT_S = 10.0

# A handshake that has not completed after this long is not slow, it is never
# going to finish: the measured figure against real hardware is ~4 s. The
# janitor closes such a peer's socket from the outside, which is what makes
# the blocked wrap_socket() return and let its thread unwind. Closing is not
# the same as settimeout() - it does not change the socket's blocking mode,
# so it cannot break a handshake in progress. See FINDINGS #43.
HANDSHAKE_STUCK_S = 30.0
JANITOR_TICK_S = 5.0

# Ceiling on handshakes in flight at once - a safety valve against an
# unauthenticated flood, NOT a load limiter for real devices. It must sit well
# above any legitimate peak, or a fleet coming back after a building power cut
# throttles itself. Size it from:
#
#     peak_concurrent ~= fleet_size * handshake_seconds / arrival_spread_seconds
#
# The NMU's handshake measures ~4 s and both device types spread their FIRST
# handshake over BOOT_JITTER (30 s), so 40 units give ~5 concurrent and 100
# give ~13. 64 leaves roughly a 5x margin over a 100-unit fleet while still
# bounding an attacker. Over the cap the datagram is dropped and DTLS clients
# retransmit by design. See FINDINGS #45.
MAX_INFLIGHT_HANDSHAKES = int(os.environ.get("OMEGA_MAX_INFLIGHT_HANDSHAKES", "64"))

# One hour, jittered +/-20% (48-72 min) inside session.py. See ARCHITECTURE.md
# section 12. Override with OMEGA_MAX_SESSION_AGE for testing.
DEFAULT_MAX_SESSION_AGE_S = float(os.environ.get("OMEGA_MAX_SESSION_AGE", "3600"))


def build_context():
    ctx = wolfssl.SSLContext(wolfssl.PROTOCOL_DTLSv1_3, server_side=True)
    ctx.load_cert_chain(
        os.path.join(PKI_DIR, "omega-server-cert.pem"),
        os.path.join(PKI_DIR, "omega-server-key.pem"),
    )
    ctx.load_verify_locations(os.path.join(PKI_DIR, "ca-cert.pem"))
    ctx.verify_mode = wolfssl.CERT_REQUIRED
    return ctx


def register_mdns_safe(port, log):
    try:
        return discovery.register_mdns(port, log=log)
    except Exception as error:
        log("DISCOVERY mDNS registration failed (%s) - plain-UDP discovery "
            "still active" % error)
        return None, None


def main():
    log = lambda message: print(
        time.strftime("%Y-%m-%d %H:%M:%S") + "  " + message, flush=True)

    # Fail closed. Without the device config the revocation list is empty and a
    # revoked device would still be admitted, so refuse to start at all.
    try:
        store = config_store.ConfigStore(DEVICE_CONFIG_PATH)
    except config_store.MissingDeviceConfig as error:
        print("=" * 70, file=sys.stderr)
        print("FATAL: " + str(error), file=sys.stderr)
        print("", file=sys.stderr)
        print("Create it (or point OMEGA_DEVICE_CONFIG at it) before starting.",
              file=sys.stderr)
        print("Minimal valid file:", file=sys.stderr)
        print('  {"devices": [], "revoked": [], "nmu": {"hb": 45}, '
              '"amu": {"hb": 30}}', file=sys.stderr)
        print("=" * 70, file=sys.stderr)
        return 2

    packet_queue = queue.Queue()
    threading.Thread(target=storage.db_worker,
                     args=(DB_FILE, packet_queue), daemon=True).start()

    ctx = build_context()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))

    # Discovery: advertise over mDNS and answer plain-UDP probes, so devices
    # never carry a hardcoded address and DHCP may reassign us freely.
    zeroconf, info = register_mdns_safe(UDP_PORT, log)
    responder = discovery.DiscoveryResponder(UDP_PORT, log=log)
    responder.start()
    # The advertised address is fixed at registration, so DHCP moving THIS
    # machine would leave devices chasing a dead address until a restart.
    mdns_keeper = discovery.MdnsKeeper(UDP_PORT, zeroconf, info, log=log)
    mdns_keeper.start()

    log("=" * 66)
    log("Omega brick4 listener - DTLS 1.3, session reuse")
    log("  address       : %s:%d (advertised, not hardcoded on devices)"
        % (discovery.primary_ipv4(), UDP_PORT))
    log("  max session   : %.0fs +/-20%% jitter, then forced re-handshake"
        % DEFAULT_MAX_SESSION_AGE_S)
    log("  revocation    : enforced at handshake AND per record")
    log("  device config : %s" % DEVICE_CONFIG_PATH)
    log("=" * 66)

    # A UDP socket has no accept(). The standard pattern:
    #   1. one unconnected listener socket learns a new peer's address;
    #   2. a SECOND socket on the same port is connect()ed to that peer, so the
    #      kernel routes that peer's datagrams to it alone;
    #   3. the client retransmits its ClientHello (DTLS does this by design)
    #      and the handshake completes on the connected socket.
    #
    # The listener MUST consume the datagram with a plain recvfrom. An earlier
    # version used MSG_PEEK, which leaves the datagram in the queue - so the
    # same packet was seen forever, a new socket was created every pass, and
    # the server died with "Too many open files" while never completing a
    # single handshake. known_peers is the second half of the fix: without it
    # the stray datagrams the listener catches before the kernel finishes
    # routing would each spawn another socket for a peer already being served.
    known_peers = set()
    peers_lock = threading.Lock()
    # peer -> [started_at, raw_socket, handshake_completed]
    inflight = {}
    inflight_lock = threading.Lock()

    def _forget(peer):
        with peers_lock:
            known_peers.discard(peer)
        with inflight_lock:
            inflight.pop(peer, None)

    def _handshake_done(peer):
        with inflight_lock:
            entry = inflight.get(peer)
            if entry is not None:
                entry[2] = True

    def _reap_stuck_handshakes():
        """Reclaim peers whose handshake never completed.

        Without this a peer that sends one datagram and then goes silent parks
        its thread inside wrap_socket() forever - so the thread's finally
        clause never runs, its socket is never closed, and its address stays
        in known_peers permanently. Any host on the LAN could repeat that from
        a fresh source port until the server ran out of descriptors.
        """
        while True:
            time.sleep(JANITOR_TICK_S)
            now = time.time()
            stuck = []
            with inflight_lock:
                for peer, entry in list(inflight.items()):
                    if not entry[2] and now - entry[0] > HANDSHAKE_STUCK_S:
                        stuck.append((peer, entry[1]))
                        del inflight[peer]
            for peer, peer_sock in stuck:
                log("handshake from %s never completed in %.0fs - closing its "
                    "socket and releasing the slot" % (peer, HANDSHAKE_STUCK_S))
                try:
                    peer_sock.close()
                except OSError:
                    pass
                with peers_lock:
                    known_peers.discard(peer)

    threading.Thread(target=_reap_stuck_handshakes, daemon=True).start()

    sock.settimeout(1.0)
    try:
        while True:
            try:
                _, peer = sock.recvfrom(RECV_BYTES)
            except socket.timeout:
                continue
            except OSError:
                continue

            with inflight_lock:
                if len(inflight) >= MAX_INFLIGHT_HANDSHAKES:
                    log("%d handshakes already in flight - dropping %s, it will "
                        "retransmit" % (len(inflight), peer))
                    continue

            with peers_lock:
                if peer in known_peers:
                    # Already has its own connected socket; this is a stray the
                    # listener caught before the kernel finished routing.
                    continue
                known_peers.add(peer)

            peer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            peer_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                peer_sock.bind((UDP_IP, UDP_PORT))
                peer_sock.connect(peer)
            except OSError as error:
                log("could not open per-peer socket for %s: %s" % (peer, error))
                peer_sock.close()
                _forget(peer)
                continue

            with inflight_lock:
                inflight[peer] = [time.time(), peer_sock, False]

            threading.Thread(
                target=session.handle_peer,
                args=(ctx, peer_sock, store, packet_queue,
                      DEFAULT_MAX_SESSION_AGE_S, HANDSHAKE_TIMEOUT_S, log,
                      lambda p=peer: _forget(p)),
                kwargs={"mailbox_db_path": MAILBOX_DB_PATH,
                        "identity_db_path": IDENTITY_DB_PATH,
                        "address_db_path": ADDRESS_DB_PATH,
                        "on_handshake_done": lambda p=peer: _handshake_done(p)},
                daemon=True).start()
    except KeyboardInterrupt:
        log("shutting down")
    finally:
        responder.stop()
        mdns_keeper.stop()
        # Ask the keeper, not the start-up locals: a re-advertisement replaces
        # both objects, and unregistering the originals would leave the live
        # record published.
        discovery.unregister_mdns(*mdns_keeper.current())
    return 0


if __name__ == "__main__":
    sys.exit(main())
