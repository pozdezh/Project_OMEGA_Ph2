"""Find the Omega server without ever hardcoding its address.

The router (DHCP) can change the server's IP at any time. Anything compiled or
typed into a device dies the moment that happens. This module answers the
question "where is the server right now?" using three sources, cheapest first:

  1. THE CACHE - the address that worked last time, kept in a small file. Tried
     first because it is almost always still correct and costs one datagram.
  2. mDNS - ask the local network by name (_omega._udp.local), the same trick
     printers and Chromecasts use. No central server involved.
  3. A PLAIN-UDP BROADCAST PROBE - shout on the LAN and let the server answer.
     Works when mDNS/Avahi is unavailable on either end.

A static address from the config file is kept as a final fallback so a site
with no multicast still works.

SAFETY: discovery only ever yields a candidate ADDRESS. Nothing secret is
exchanged and nothing is trusted yet. A forged reply simply points the device
at a host that cannot produce a CA-signed certificate, so the DTLS handshake
fails and the device tries the next candidate. The handshake stays the trust
boundary.
"""

import json
import os
import socket
import time

SERVICE_TYPE = "_omega._udp.local."
DISCOVERY_PORT = 5001
DISCOVERY_MAGIC = b"OMEGA_DISCOVER_V1"
DISCOVERY_REPLY_MAGIC = "OMEGA_SERVER_V1"
PROBE_TIMEOUT_S = 2.0
MDNS_TIMEOUT_S = 3.0
# Where a server that publishes no usable "prio" TXT record sorts. Must match
# MDNS_DEFAULT_PRIORITY in nmu/omega_discovery.cpp so both device types rank
# the same fleet identically. Higher than the primary's advertised 10, so a
# silent or misconfigured responder can never outrank the real server.
DEFAULT_PRIORITY = 50
DEFAULT_CACHE = os.path.expanduser("~/.omega_server_cache.json")


def _write_atomic(path, doc):
    """Write to a temp file then rename. A rename cannot be half-done, so a
    power cut leaves either the old file or the new one, never a corrupt one."""
    tmp = path + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(doc, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except OSError:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def load_cached(cache_path=DEFAULT_CACHE):
    try:
        with open(cache_path, "r", encoding="utf-8") as handle:
            doc = json.load(handle)
        if doc.get("ip") and doc.get("port"):
            return (str(doc["ip"]), int(doc["port"]))
    except (OSError, ValueError, TypeError):
        pass
    return None


def save_cached(host, port, cache_path=DEFAULT_CACHE):
    _write_atomic(cache_path, {"ip": host, "port": int(port),
                               "saved": int(time.time())})


def probe_reachable(host, port, timeout_s=PROBE_TIMEOUT_S):
    """Is a server answering discovery probes at this address? Cheap liveness
    check before paying for a full DTLS handshake."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout_s)
    try:
        sock.sendto(DISCOVERY_MAGIC, (host, DISCOVERY_PORT))
        data, _ = sock.recvfrom(512)
        doc = json.loads(data.decode("utf-8"))
        return doc.get("magic") == DISCOVERY_REPLY_MAGIC
    except (OSError, ValueError, UnicodeDecodeError):
        return False
    finally:
        sock.close()


def priority_of(properties):
    """The "prio" TXT record as an int, lower meaning more preferred.

    A server that publishes no usable prio sorts LAST rather than first, so a
    misconfigured or minimal responder can never outrank the primary just by
    saying nothing.
    """
    try:
        raw = (properties or {}).get(b"prio")
        return int(raw.decode("utf-8")) if raw else DEFAULT_PRIORITY
    except (ValueError, AttributeError, UnicodeDecodeError):
        return DEFAULT_PRIORITY


def discover_all_by_mdns(timeout_s=MDNS_TIMEOUT_S, log=print):
    """Every Omega server answering on this LAN, best first.

    Sorted by the "prio" TXT record (lower wins, like a DNS SRV priority), so
    a site with a spare server can say which one is preferred. Servers that
    publish no prio sort last. Returns a list of (host, port); it may be empty.
    """
    try:
        from zeroconf import Zeroconf, ServiceBrowser
    except ImportError:
        return []

    found = []

    class _Listener:
        def add_service(self, zc, type_, name):
            info = zc.get_service_info(type_, name, timeout=int(timeout_s * 1000))
            if info and info.addresses:
                found.append((priority_of(info.properties),
                              socket.inet_ntoa(info.addresses[0]), info.port))

        def update_service(self, zc, type_, name):
            pass

        def remove_service(self, zc, type_, name):
            pass

    zeroconf = Zeroconf()
    try:
        ServiceBrowser(zeroconf, SERVICE_TYPE, _Listener())
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            time.sleep(0.1)
    except Exception as error:
        log("discovery: mDNS failed (%s)" % error)
    finally:
        try:
            zeroconf.close()
        except Exception:
            pass
    found.sort(key=lambda entry: entry[0])
    return [(host, port) for _prio, host, port in found]


def discover_by_mdns(timeout_s=MDNS_TIMEOUT_S, log=print):
    """The single best mDNS answer, or None."""
    servers = discover_all_by_mdns(timeout_s=timeout_s, log=log)
    return servers[0] if servers else None


def discover_by_broadcast(timeout_s=PROBE_TIMEOUT_S, log=print):
    """Shout on the LAN broadcast address and take the first server that
    answers. Returns (host, port) or None."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except OSError:
        pass
    sock.settimeout(timeout_s)
    try:
        sock.sendto(DISCOVERY_MAGIC, ("255.255.255.255", DISCOVERY_PORT))
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                data, peer = sock.recvfrom(512)
            except socket.timeout:
                break
            try:
                doc = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if doc.get("magic") != DISCOVERY_REPLY_MAGIC:
                continue
            return (str(doc.get("ip") or peer[0]), int(doc.get("port")))
    except OSError as error:
        log("discovery: broadcast failed (%s)" % error)
    finally:
        sock.close()
    return None


def forget_cached(cache_path=DEFAULT_CACHE):
    """Drop the remembered address. Called when it fails to authenticate."""
    try:
        os.unlink(cache_path)
        return True
    except OSError:
        return False


def confirm_server(host, port, cache_path=DEFAULT_CACHE):
    """Remember an address that COMPLETED a mutual-auth handshake.

    This is the only place the cache is written. Discovery itself must never
    write it: answering a discovery probe proves nothing, so caching on that
    basis let any responder on the LAN pin a device to itself permanently.
    """
    save_cached(host, port, cache_path)


def find_servers(static_fallback=None, cache_path=DEFAULT_CACHE, log=print):
    """Every candidate address worth trying, best first, deduplicated.

    These are CANDIDATES, not servers. Nothing here is trusted and nothing is
    cached: the caller tries each in turn and the DTLS handshake decides which
    one is real. That is what makes a rogue responder a nuisance rather than a
    denial of service - it costs one failed handshake, then the device moves
    on to the next candidate.
    """
    candidates = []

    def add(entry, why):
        if not entry or not entry[0]:
            return
        pair = (str(entry[0]), int(entry[1]))
        if pair in candidates:
            return
        candidates.append(pair)
        log("discovery: candidate %s:%d (%s)" % (pair[0], pair[1], why))

    cached = load_cached(cache_path)
    if cached and probe_reachable(cached[0], cached[1]):
        add(cached, "last confirmed, still answering")

    for server in discover_all_by_mdns(log=log):
        add(server, "mDNS")

    add(discover_by_broadcast(log=log), "UDP broadcast")

    if static_fallback and static_fallback[0]:
        add((static_fallback[0], int(static_fallback[1])), "configured fallback")

    if cached:
        add(cached, "last confirmed, unverified")

    if not candidates:
        log("discovery: no server found by any method")
    return candidates


def find_server(static_fallback=None, cache_path=DEFAULT_CACHE, log=print):
    """The single best candidate, or None. Prefer find_servers()."""
    candidates = find_servers(static_fallback=static_fallback,
                              cache_path=cache_path, log=log)
    return candidates[0] if candidates else None
