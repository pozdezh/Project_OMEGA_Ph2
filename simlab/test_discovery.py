"""Gate check: server discovery, so no device carries a hardcoded address.

Why this matters: the router (DHCP) can reassign the server's IP at any time.
With the address compiled into firmware, that change bricks the whole fleet and
every unit needs a reflash. These tests prove a device can find the server
without being told where it is, and that startup order does not matter.

Runs the REAL server responder against the REAL client discovery code over
loopback UDP - not mocks - so a break in either side fails here.
"""

import os
import socket
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "server"))
sys.path.insert(0, os.path.join(ROOT, "amu"))

import discovery as server_discovery_mod
import server_discovery as client_discovery

TEST_DISCOVERY_PORT = 15001
SERVICE_PORT = 15000


def _silent(*_args, **_kwargs):
    pass


def _start_responder():
    responder = server_discovery_mod.DiscoveryResponder(
        SERVICE_PORT, discovery_port=TEST_DISCOVERY_PORT, log=_silent)
    responder.start()
    time.sleep(0.4)
    return responder


def _stop_responder(responder):
    """Stop AND wait for the thread to actually exit.

    The responder loop uses a 1 s socket timeout, so stop() only sets a flag -
    the socket can stay bound for up to a second afterwards. Without joining,
    the next test can be answered by the previous test's responder, which is
    exactly the false pass this helper prevents.
    """
    responder.stop()
    responder.join(timeout=5.0)
    assert not responder.is_alive(), "responder thread did not stop"


def test_responder_answers_probe():
    """A device that knows nothing must be able to ask 'where is the server?'"""
    responder = _start_responder()
    try:
        # Speak the client's protocol directly at the responder.
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3.0)
        sock.sendto(client_discovery.DISCOVERY_MAGIC,
                    ("127.0.0.1", TEST_DISCOVERY_PORT))
        data, _ = sock.recvfrom(512)
        sock.close()

        import json
        doc = json.loads(data.decode("utf-8"))
        assert doc.get("magic") == client_discovery.DISCOVERY_REPLY_MAGIC, \
            "reply magic mismatch: " + str(doc)
        assert int(doc.get("port")) == SERVICE_PORT, \
            "reply must carry the DTLS service port, got " + str(doc.get("port"))
        assert doc.get("ip"), "reply must carry an address"
        assert int(doc.get("t", 0)) > 1700000000, \
            "reply must carry server time for clockless devices"
        print("PASS responder answers a discovery probe with address, port, time")
    finally:
        _stop_responder(responder)


def test_probe_reachable_true_and_false():
    """The cheap liveness check must say yes to a live server and no to a dead
    address - that is what stops a device paying ~5 s for a doomed handshake."""
    responder = _start_responder()
    try:
        original = client_discovery.DISCOVERY_PORT
        client_discovery.DISCOVERY_PORT = TEST_DISCOVERY_PORT
        try:
            alive = client_discovery.probe_reachable("127.0.0.1", SERVICE_PORT,
                                                     timeout_s=2.0)
            assert alive, "live server must answer a probe"
            # 203.0.113.0/24 is TEST-NET-3, reserved for documentation: nothing
            # can legitimately answer there.
            dead = client_discovery.probe_reachable("203.0.113.7", SERVICE_PORT,
                                                    timeout_s=1.0)
            assert not dead, "dead address must not report reachable"
        finally:
            client_discovery.DISCOVERY_PORT = original
        print("PASS liveness probe distinguishes live from dead server")
    finally:
        _stop_responder(responder)


def test_cache_is_atomic_and_survives_corruption():
    """The cached address is written temp-file-then-rename, so a power cut
    leaves either the old file or the new one - never a corrupt one."""
    tmp = tempfile.mkdtemp(prefix="omega_disc_")
    cache = os.path.join(tmp, "server_cache.json")

    client_discovery.save_cached("192.168.0.112", 5000, cache)
    assert client_discovery.load_cached(cache) == ("192.168.0.112", 5000), \
        "cache round-trip failed"
    assert not os.path.exists(cache + ".tmp"), "temp file must not be left behind"

    with open(cache, "w", encoding="utf-8") as handle:
        handle.write("{ truncated by a power cut")
    assert client_discovery.load_cached(cache) is None, \
        "corrupt cache must be ignored, not crash"
    assert client_discovery.load_cached(os.path.join(tmp, "absent.json")) is None, \
        "missing cache must be ignored, not crash"
    print("PASS cache is atomic and survives corruption")


def test_startup_order_does_not_matter():
    """A device booted BEFORE the server must find it once it appears.

    This is the requirement that the fleet comes up in unison regardless of
    who powers on first.
    """
    tmp = tempfile.mkdtemp(prefix="omega_order_")
    cache = os.path.join(tmp, "server_cache.json")
    original = client_discovery.DISCOVERY_PORT
    client_discovery.DISCOVERY_PORT = TEST_DISCOVERY_PORT
    try:
        # Device boots first: nothing answers, and it must not crash or hang.
        found = client_discovery.find_server(static_fallback=None,
                                             cache_path=cache, log=_silent)
        assert found is None, "must report no server while none is running"

        # Server appears later.
        responder = _start_responder()
        try:
            found = client_discovery.find_server(static_fallback=None,
                                                 cache_path=cache, log=_silent)
            assert found is not None, "device must find the server once it starts"
            assert int(found[1]) == SERVICE_PORT, \
                "discovered port must be the DTLS service port, got " + str(found)
            # Discovery must NOT cache. Answering a probe proves nothing, and
            # caching on that basis let a rogue responder pin a device to
            # itself permanently - see ARCHITECTURE.md 13. Only a completed
            # handshake may write the cache, via confirm_server().
            assert client_discovery.load_cached(cache) is None, \
                "discovery must not cache an unauthenticated address"
            client_discovery.confirm_server(found[0], found[1], cache)
            assert client_discovery.load_cached(cache) == found, \
                "a handshake-confirmed address must be remembered"
            client_discovery.forget_cached(cache)
            assert client_discovery.load_cached(cache) is None, \
                "an address that fails to authenticate must be forgettable"
        finally:
            _stop_responder(responder)
        print("PASS device booted before the server still finds it")
        print("PASS only a confirmed handshake writes the discovery cache")
    finally:
        client_discovery.DISCOVERY_PORT = original


def test_static_fallback_used_when_nothing_answers():
    """A site with no multicast must still work via the configured address."""
    tmp = tempfile.mkdtemp(prefix="omega_fallback_")
    cache = os.path.join(tmp, "server_cache.json")
    original = client_discovery.DISCOVERY_PORT
    client_discovery.DISCOVERY_PORT = TEST_DISCOVERY_PORT
    try:
        found = client_discovery.find_server(
            static_fallback=("192.168.0.112", 5000), cache_path=cache,
            log=_silent)
        assert found == ("192.168.0.112", 5000), \
            "configured fallback must be used when discovery finds nothing, got " + str(found)
        print("PASS static fallback still works where multicast is unavailable")
    finally:
        client_discovery.DISCOVERY_PORT = original


def test_mdns_readvertises_when_server_address_changes():
    """The advertised mDNS record must follow a DHCP move of the SERVER.

    register_mdns() bakes the address in at registration time. Without a
    watcher, a server whose own IP changes keeps publishing the old one and
    every device trusting mDNS is sent to a dead host until a restart.

    Real zeroconf is not exercised here (multicast is unavailable in the gate);
    what is proven is the decision logic and the full teardown-and-rebuild,
    which is the part that was missing.
    """
    calls = {"register": 0, "unregister": []}
    address = {"value": "192.168.0.112"}

    real_primary = server_discovery_mod.primary_ipv4
    real_register = server_discovery_mod.register_mdns
    real_unregister = server_discovery_mod.unregister_mdns

    def fake_register(port, log=None):
        calls["register"] += 1
        return ("zc%d" % calls["register"], "info%d" % calls["register"])

    server_discovery_mod.primary_ipv4 = lambda: address["value"]
    server_discovery_mod.register_mdns = fake_register
    server_discovery_mod.unregister_mdns =         lambda zc, info: calls["unregister"].append((zc, info))
    try:
        keeper = server_discovery_mod.MdnsKeeper(
            SERVICE_PORT, "zc0", "info0", check_s=0.05, log=_silent)
        keeper.start()

        time.sleep(0.25)
        assert keeper.readvertisements == 0,             "a stable address must not cause re-advertisement"
        assert keeper.current() == ("zc0", "info0")

        address["value"] = "192.168.0.207"
        deadline = time.time() + 3.0
        while keeper.readvertisements == 0 and time.time() < deadline:
            time.sleep(0.05)
        keeper.stop()

        assert keeper.readvertisements == 1,             "address change must trigger exactly one re-advertisement, got "             + str(keeper.readvertisements)
        assert calls["unregister"] == [("zc0", "info0")],             "the stale record must be withdrawn, not left published: "             + str(calls["unregister"])
        assert keeper.current() == ("zc1", "info1"),             "shutdown must unregister the CURRENT pair, not the start-up one"
        print("PASS mDNS re-advertises when the server's own address changes")
    finally:
        server_discovery_mod.primary_ipv4 = real_primary
        server_discovery_mod.register_mdns = real_register
        server_discovery_mod.unregister_mdns = real_unregister


def test_mdns_keeper_survives_a_failed_readvertisement():
    """A broken re-advertisement must not kill the listener.

    The plain-UDP responder computes the address per reply, so it stays correct
    even when mDNS cannot be republished. Losing the process would lose that.
    """
    address = {"value": "192.168.0.112"}
    real_primary = server_discovery_mod.primary_ipv4
    real_register = server_discovery_mod.register_mdns
    real_unregister = server_discovery_mod.unregister_mdns

    def exploding_register(port, log=None):
        raise OSError("multicast socket unavailable")

    server_discovery_mod.primary_ipv4 = lambda: address["value"]
    server_discovery_mod.register_mdns = exploding_register
    server_discovery_mod.unregister_mdns = lambda zc, info: None
    try:
        keeper = server_discovery_mod.MdnsKeeper(
            SERVICE_PORT, "zc0", "info0", check_s=0.05, log=_silent)
        keeper.start()
        address["value"] = "192.168.0.207"
        time.sleep(0.5)
        assert keeper.is_alive(),             "a failed re-advertisement must not stop the keeper thread"
        keeper.stop()
        print("PASS a failed mDNS re-advertisement does not take the listener down")
    finally:
        server_discovery_mod.primary_ipv4 = real_primary
        server_discovery_mod.register_mdns = real_register
        server_discovery_mod.unregister_mdns = real_unregister


def main():
    test_responder_answers_probe()
    test_probe_reachable_true_and_false()
    test_cache_is_atomic_and_survives_corruption()
    test_startup_order_does_not_matter()
    test_static_fallback_used_when_nothing_answers()
    test_mdns_readvertises_when_server_address_changes()
    test_mdns_keeper_survives_a_failed_readvertisement()
    print("RESULT: PASS - discovery proven, no hardcoded server address needed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
