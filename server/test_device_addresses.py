"""The server learns where each device lives, so nobody maintains a map.

A live query needs an address to call. Keeping that in a hand-written
"endpoints" block meant every new unit needed a config edit before it could
be reached, and a DHCP change broke the entry silently because nothing
re-checked it.

Every device dials the server for telemetry, so at the moment a handshake
completes the listener holds both facts at once: the identity the certificate
proved, and the address the datagram came from. These tests pin that pairing
and the rules around it.

    py -3.12 server/test_device_addresses.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import device_addresses


def _fresh():
    return os.path.join(tempfile.mkdtemp(prefix="omega_addr_"), "device_addresses.json")


def test_a_device_is_reachable_after_it_reports_once():
    """The whole point: no configuration step between installing a unit and
    being able to call it."""
    path = _fresh()
    assert device_addresses.lookup(path, "AMU_12") is None, \
        "an unknown device must not resolve to a guess"

    device_addresses.remember(path, "AMU_12", "192.168.0.101")
    assert device_addresses.lookup(path, "AMU_12") == "192.168.0.101"
    print("PASS a device is reachable after it reports once, with no config edit")


def test_a_device_that_moves_is_followed():
    """DHCP reassigns addresses. A learned address that never updated would be
    worse than none: it would send live calls to whatever now holds it."""
    path = _fresh()
    device_addresses.remember(path, "AMU_12", "192.168.0.101")
    changed = device_addresses.remember(path, "AMU_12", "192.168.0.155")
    assert changed is True, "a move must be reported so it can be logged"
    assert device_addresses.lookup(path, "AMU_12") == "192.168.0.155"
    print("PASS a device that moves is followed to its new address")


def test_a_routine_reconnection_does_not_rewrite_the_file():
    """The fleet re-handshakes hourly. Rewriting on every reconnection would
    be pointless disk traffic and would bury a real move in the noise."""
    path = _fresh()
    device_addresses.remember(path, "AMU_12", "192.168.0.101")
    again = device_addresses.remember(path, "AMU_12", "192.168.0.101")
    assert again is False, "an unchanged address must not report a move"
    print("PASS a routine reconnection does not rewrite the file")


def test_devices_do_not_share_or_overwrite_each_other():
    path = _fresh()
    for n in range(11, 18):
        device_addresses.remember(path, "AMU_%d" % n, "192.168.0.%d" % (100 + n))
    known = device_addresses.all_known(path)
    assert len(known) == 7, known
    assert device_addresses.lookup(path, "AMU_11") == "192.168.0.111"
    assert device_addresses.lookup(path, "AMU_17") == "192.168.0.117"
    print("PASS each device keeps its own address")


def test_a_corrupt_or_missing_file_is_survivable():
    """This file is written while the fleet is live. A half-written or deleted
    one must degrade to 'address unknown', never take the server down."""
    path = _fresh()
    assert device_addresses.lookup(path, "AMU_12") is None, "missing file must not raise"

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("{ this is not json")
    assert device_addresses.lookup(path, "AMU_12") is None, "corrupt file must not raise"

    device_addresses.remember(path, "AMU_12", "192.168.0.101")
    assert device_addresses.lookup(path, "AMU_12") == "192.168.0.101", \
        "a corrupt file must be replaced by a good one, not poison every later write"
    print("PASS a corrupt or missing file degrades to 'unknown', never a crash")


def test_forgetting_makes_the_next_call_relearn():
    path = _fresh()
    device_addresses.remember(path, "AMU_12", "192.168.0.101")
    assert device_addresses.forget(path, "AMU_12") is True
    assert device_addresses.lookup(path, "AMU_12") is None
    assert device_addresses.forget(path, "AMU_12") is False, \
        "forgetting an unknown device is not an error"
    print("PASS a forgotten address is re-learned on the next handshake")


def main():
    test_a_device_is_reachable_after_it_reports_once()
    test_a_device_that_moves_is_followed()
    test_a_routine_reconnection_does_not_rewrite_the_file()
    test_devices_do_not_share_or_overwrite_each_other()
    test_a_corrupt_or_missing_file_is_survivable()
    test_forgetting_makes_the_next_call_relearn()
    print("RESULT: PASS - device addresses are learned, not configured")
    return 0


if __name__ == "__main__":
    sys.exit(main())
