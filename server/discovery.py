"""Server-side discovery, so no device ever needs a hardcoded server address.

The problem this solves: the router hands out IP addresses (DHCP) and can
change them. With the address compiled into the firmware, a DHCP change bricks
the whole fleet and every unit needs a reflash.

Two independent mechanisms, both running at once, because the two device types
have different capabilities:

  1. mDNS/DNS-SD advertisement (_omega._udp.local) via Avahi. The AMU uses
     python-zeroconf and the NMU uses ESPmDNS to look this up. Note ESPmDNS
     cannot read SRV priority, so the priority is ALSO published as a TXT
     record ("prio") for the NMU to read.

  2. A tiny plain-UDP discovery responder. A device that cannot do mDNS - or
     whose mDNS query failed because Avahi is momentarily down - broadcasts a
     one-line probe to the discovery port and gets the server's address, port
     and clock back. This is deliberately unauthenticated and carries nothing
     secret: it only yields a candidate ADDRESS. The DTLS handshake remains
     the trust boundary, so a forged reply just points a device at a host that
     cannot produce a CA-signed certificate, and the device moves on.

Startup order therefore does not matter. A device that boots before the server
keeps probing and connects when the server appears; a server that restarts is
re-found by devices already running.
"""

import json
import socket
import struct
import threading
import time

SERVICE_TYPE = "_omega._udp.local."
SERVICE_NAME = "smartageing." + SERVICE_TYPE
DISCOVERY_PORT = 5001
DISCOVERY_MAGIC = "OMEGA_DISCOVER_V1"
DISCOVERY_REPLY_MAGIC = "OMEGA_SERVER_V1"
PROBE_BYTES = 512


def primary_ipv4():
    """The address other hosts on the LAN can actually reach us on.

    Connecting a UDP socket sends no packets; it just makes the kernel pick
    the interface it would use to reach the outside, which is the one the
    fleet is on. More reliable than gethostbyname(), which often returns
    127.0.1.1 on Debian/Ubuntu.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


class DiscoveryResponder(threading.Thread):
    """Answers plain-UDP probes with the server's address, port and clock."""

    def __init__(self, service_port, discovery_port=DISCOVERY_PORT, log=print):
        threading.Thread.__init__(self, daemon=True)
        self._service_port = service_port
        self._discovery_port = discovery_port
        self._log = log
        # NOT self._stop: threading.Thread uses that name internally, and
        # shadowing it breaks Thread.join().
        self._stop_event = threading.Event()
        self.replies_sent = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except OSError:
            pass
        try:
            sock.bind(("0.0.0.0", self._discovery_port))
        except OSError as error:
            self._log("DISCOVERY responder could not bind udp/" +
                      str(self._discovery_port) + ": " + str(error))
            return
        sock.settimeout(1.0)
        self._log("DISCOVERY responder up on udp/" + str(self._discovery_port))

        while not self._stop_event.is_set():
            try:
                data, peer = sock.recvfrom(PROBE_BYTES)
            except socket.timeout:
                continue
            except OSError:
                continue
            if DISCOVERY_MAGIC.encode("utf-8") not in data:
                continue
            reply = json.dumps({
                "magic": DISCOVERY_REPLY_MAGIC,
                "ip": primary_ipv4(),
                "port": self._service_port,
                "t": int(time.time()),
            }, separators=(",", ":")).encode("utf-8")
            try:
                sock.sendto(reply, peer)
                self.replies_sent += 1
                self._log("DISCOVERY probe from " + str(peer[0]) +
                          " answered (total " + str(self.replies_sent) + ")")
            except OSError:
                continue
        sock.close()


def register_mdns(service_port, log=print):
    """Advertise _omega._udp.local via zeroconf. Returns (zeroconf, info) or
    (None, None) if python-zeroconf is not installed - discovery then falls
    back to the plain-UDP responder, which is always available."""
    try:
        from zeroconf import Zeroconf, ServiceInfo
    except ImportError:
        log("DISCOVERY mDNS unavailable (python-zeroconf not installed) - "
            "plain-UDP discovery still active")
        return None, None

    address = primary_ipv4()
    info = ServiceInfo(
        SERVICE_TYPE,
        SERVICE_NAME,
        addresses=[socket.inet_aton(address)],
        port=service_port,
        # ESPmDNS on the NMU cannot read SRV priority, so publish it as TXT too.
        properties={"prio": "10", "role": "primary", "proto": "dtls1.3"},
        server="smartageing.local.",
    )
    zeroconf = Zeroconf()
    zeroconf.register_service(info)
    log("DISCOVERY mDNS advertising " + SERVICE_NAME + " -> " +
        address + ":" + str(service_port))
    return zeroconf, info


def unregister_mdns(zeroconf, info):
    if zeroconf is None:
        return
    try:
        zeroconf.unregister_service(info)
        zeroconf.close()
    except Exception:
        pass


class MdnsKeeper(threading.Thread):
    """Keeps the advertised mDNS address equal to the server's CURRENT one.

    register_mdns() bakes whatever primary_ipv4() returned at start-up into the
    published record. If DHCP moves the SERVER while this process keeps running,
    that record keeps naming an address nothing answers on, and every device
    that trusts mDNS is sent to a dead host until someone restarts the listener.

    The plain-UDP responder never had this problem - it recomputes the address
    on every reply - so this closes the one discovery path that was not already
    self-correcting.

    Teardown is total rather than an in-place record edit: a Zeroconf instance
    binds its sockets when it is constructed, so after the interface address
    changes the old instance can still be advertising from a socket bound to
    the address that no longer exists. Dropping it and building a new one reuses
    the same code path that works correctly at start-up.
    """

    def __init__(self, service_port, zeroconf, info, check_s=30.0, log=print):
        threading.Thread.__init__(self, daemon=True)
        self._service_port = service_port
        self._zeroconf = zeroconf
        self._info = info
        self._check_s = check_s
        self._log = log
        self._address = primary_ipv4()
        self._stop_event = threading.Event()
        self.readvertisements = 0

    def stop(self):
        self._stop_event.set()

    def current(self):
        """The (zeroconf, info) pair to unregister at shutdown."""
        return self._zeroconf, self._info

    def _readvertise(self, address):
        unregister_mdns(self._zeroconf, self._info)
        self._zeroconf, self._info = register_mdns(self._service_port,
                                                   log=self._log)
        self._address = address
        self.readvertisements += 1

    def run(self):
        while not self._stop_event.wait(self._check_s):
            try:
                address = primary_ipv4()
            except OSError:
                continue
            if address == self._address or address == "127.0.0.1":
                continue
            self._log("DISCOVERY server address changed " + self._address +
                      " -> " + address + ", re-advertising mDNS")
            try:
                self._readvertise(address)
            except Exception as error:
                # A failed re-advertisement must not take the listener down:
                # the plain-UDP responder still answers with the right address.
                self._log("DISCOVERY mDNS re-advertisement failed (" +
                          str(error) + ") - plain-UDP discovery still correct")
