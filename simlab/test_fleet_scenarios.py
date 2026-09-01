"""Gate check: the fleet-behaviour promises a field test is supposed to prove.

These cover what the other gate files do not: how a device RANKS the servers
discovery offers it, how long it will keep trying before it gives up and
buffers, and that a backlog drains oldest-first and is deleted only once each
record has its own receipt.

Constants are read from the SHIPPED sources - including the C++ firmware and
dtls_client.py, which cannot be imported here because wolfSSL has no Windows
wheel - so firmware, client and documentation cannot drift apart silently.
"""

import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "amu"))
sys.path.insert(0, os.path.join(ROOT, "server"))

import server_discovery
import config_store

NMU_DTLS = os.path.join(ROOT, "nmu", "omega_dtls.cpp")
NMU_DISCOVERY = os.path.join(ROOT, "nmu", "omega_discovery.cpp")
NMU_CONFIG = os.path.join(ROOT, "deploy", "esp32", "stage3_beta_2_dtls",
                          "config.h.example")
AMU_DTLS = os.path.join(ROOT, "amu", "dtls_client.py")
AMU_MAIN = os.path.join(ROOT, "amu", "main.py")
AMU_NETWORK = os.path.join(ROOT, "amu", "network.py")
AMU_INI = os.path.join(ROOT, "deploy", "amu", "payload", "config",
                       "global.ini.example")
MAKE_UNITS = os.path.join(ROOT, "provisioning", "pendrive", "make-units")
MAKE_AMU = os.path.join(ROOT, "provisioning", "make-amu-bundle")
NMU_TASKS = os.path.join(ROOT, "nmu", "omega_tasks.cpp")


def _constant(path, pattern):
    """One numeric constant out of a shipped source file, C++ or Python."""
    with open(path, "r", encoding="utf-8") as handle:
        found = re.search(pattern, handle.read())
    assert found is not None, "constant not found in " + path + ": " + pattern
    return float(found.group(1))


def test_a_server_that_publishes_no_priority_sorts_last():
    """The bug this closes: DEFAULT_PRIORITY was referenced but never defined,
    so ranking a server that published no "prio" raised NameError on the
    zeroconf callback thread. The exception was swallowed there and that
    server was silently dropped from the candidate list - a spare or
    minimally-configured server would simply never be found, with no error
    anywhere. Our own server publishes prio, which is why it never showed."""
    primary = server_discovery.priority_of({b"prio": b"10"})
    silent = server_discovery.priority_of({})
    malformed = server_discovery.priority_of({b"prio": b"not-a-number"})
    absent = server_discovery.priority_of(None)

    assert silent == malformed == absent == server_discovery.DEFAULT_PRIORITY
    assert primary < silent, \
        "a server that says nothing must never outrank the declared primary"
    print("PASS a server publishing no priority sorts last instead of raising")


def test_both_device_types_rank_the_same_fleet_identically():
    """The AMU (python-zeroconf) and the NMU (ESPmDNS) are separate
    implementations. If their default priorities disagreed, the two device
    types could prefer DIFFERENT servers on the same LAN."""
    nmu_default = _constant(NMU_DISCOVERY, r"MDNS_DEFAULT_PRIORITY\s*=\s*(\d+)")
    assert nmu_default == server_discovery.DEFAULT_PRIORITY, \
        ("NMU default priority %d != AMU default %d - the two device types "
         "would rank the same fleet differently"
         % (nmu_default, server_discovery.DEFAULT_PRIORITY))
    print("PASS AMU and NMU rank servers by the same default priority")


def test_lower_priority_wins_and_an_unlabelled_server_goes_last():
    ranked = sorted([(server_discovery.priority_of(props), name)
                     for props, name in (
                         ({b"prio": b"10"}, "primary"),
                         ({}, "unlabelled"),
                         ({b"prio": b"20"}, "secondary"))],
                    key=lambda entry: entry[0])
    assert [name for _prio, name in ranked] == \
        ["primary", "secondary", "unlabelled"], ranked
    print("PASS lowest advertised priority is tried first")


def test_neither_device_type_drops_a_reading_it_could_not_hand_over():
    """Both device types keep sampling while the sender is blocked waiting for
    an acknowledgement, so both overflow the same small hand-over queue. The
    rule is identical on both: the OLDEST goes to the persistent buffer, the
    newest keeps its slot, and nothing is dropped on the floor.

    The NMU used to ignore the result of the final hand-over after making
    room. In that case the event vanished with no trace, while the AMU
    persisted it. Two device types must not disagree about what happens to a
    reading, so this asserts the fallback exists in both sources.
    """
    with open(NMU_TASKS, "r", encoding="utf-8") as handle:
        nmu = handle.read()
    body = nmu.split("static bool enqueueEventNewestWins", 1)[1]
    body = body.split(chr(10) + "static ", 1)[0]
    assert "bufferSave(event);" in body, (
        "the NMU must persist an event it could not hand over, not drop it")
    assert body.count("bufferSave(") >= 2, (
        "both the displaced event and the undeliverable new one must persist")

    with open(AMU_MAIN, "r", encoding="utf-8") as handle:
        amu = handle.read()
    body = amu.split("def enqueue_newest_wins", 1)[1]
    body = body.split(chr(10) + "def ", 1)[0]
    assert body.count("buffer.append_to_buffer") >= 2, (
        "the AMU must persist both the displaced record and one it cannot queue")
    print("PASS neither device type drops a reading it could not hand over")


def test_exactly_one_thread_may_ever_drive_the_dtls_session():
    """The crash this guards against was seen live on 2026-08-25 during a
    deliberate server outage: two SIGSEGVs in a row, each reported as
    "Fatal Python error: _PyThreadState_Attach: non-NULL old thread state",
    both straight after a failed handshake to the unreachable server.

    Cause: main() started the network worker and THEN ran a handshake from
    its own thread, so two threads could sit inside DtlsClient at once. The
    shared wolfSSL lock covers the handshake call, but not close(), not
    socket creation and not the assignment of the connection handles - so one
    thread could free a wolfSSL object while the other wrapped a socket
    around it.

    One owner needs no lock and cannot race, so this asserts the ownership
    rather than the locking.
    """
    with open(AMU_MAIN, "r", encoding="utf-8") as handle:
        main_src = handle.read()

    assert main_src.count("threading.Thread(") == 1, (
        "exactly one background thread may exist in the AMU sender process")
    assert "connect_initial" not in main_src, (
        "no handshake may be driven from the sampling thread - the network "
        "worker owns the session")

    body = main_src.split("def main(", 1)[1]
    started = body.index("threading.Thread(")
    for owned in ("network.deliver", "network.ensure_session", "client.connect"):
        assert owned not in body[started:], (
            "main() must not touch the DTLS session after starting the worker")

    with open(AMU_NETWORK, "r", encoding="utf-8") as handle:
        network_src = handle.read()
    assert "def connect_initial" not in network_src, (
        "the second entry point into the session must stay deleted, not just "
        "unused - an unused one invites the race straight back in")
    print("PASS exactly one thread may ever drive the DTLS session")


def test_the_dtls_context_is_built_on_the_main_thread_before_any_worker():
    """wolfSSL initialises only on the MAIN thread of a process.

    Proved on the live AMU on 2026-08-25 with a two-line experiment: the same
    SSLContext() call returned "wolfSSL library initialization failed" from a
    worker thread and succeeded from the main thread, in one process, seconds
    apart. Earlier that day the first handshake was moved onto the worker
    thread to give the session a single owner - correct for the race it
    closed, but it silently took the crypto with it, and the unit could not
    open a session at all until this was found.

    Both properties are needed together and neither may be traded for the
    other, so this asserts the ORDER: build on the main thread, then hand
    ownership to the one worker.
    """
    with open(AMU_MAIN, "r", encoding="utf-8") as handle:
        main_src = handle.read()

    body = main_src.split("def main(", 1)[1]
    assert "network.prepare_crypto()" in body, (
        "main() must build the DTLS context itself, on the main thread")
    assert body.index("network.prepare_crypto()") < body.index("threading.Thread("), (
        "the context must be built BEFORE the worker thread starts, or "
        "wolfSSL initialises on the wrong thread and no session can open")

    with open(AMU_NETWORK, "r", encoding="utf-8") as handle:
        network_src = handle.read()
    assert "def prepare_crypto" in network_src, (
        "the main-thread entry point for context creation must exist")

    with open(AMU_DTLS, "r", encoding="utf-8") as handle:
        dtls_src = handle.read()
    prepare = dtls_src.split("def prepare(self)", 1)[1].split(chr(10) + "    def ", 1)[0]
    for forbidden in ("socket.socket(", "self._raw", "wrap_socket"):
        assert forbidden not in prepare, (
            "context creation must open no socket (found %r) - a server that "
            "is down at boot must not stop a unit initialising its crypto"
            % forbidden)
    print("PASS the DTLS context is built on the main thread before any worker")


def test_every_shipped_config_points_at_the_live_listener_port():
    """The port a device falls back to when discovery fails.

    The AMU example config shipped 5000 - brick1's port, and brick1's
    listener is disabled - while the live listener runs on 11400. A unit only
    ever uses this value when discovery has already failed, so the wrong one
    is invisible until the single moment it is needed, and then the unit is
    simply silent. Every place that names the port is compared here rather
    than trusted.
    """
    import re

    sources = {
        "AMU example config": (AMU_INI, r"target_port\s*=\s*(\d+)"),
        "NMU firmware config": (NMU_CONFIG, r"OMEGA_SERVER_PORT\s+(\d+)"),
        "NMU factory tool": (MAKE_UNITS, r"SERVER_PORT=(\d+)"),
        "AMU bundle tool": (MAKE_AMU, r"SERVER_PORT=(\d+)"),
    }

    seen = {}
    for label, (path, pattern) in sources.items():
        with open(path, "r", encoding="utf-8") as handle:
            found = re.search(pattern, handle.read())
        assert found is not None, "no port found in %s (%s)" % (label, path)
        seen[label] = int(found.group(1))

    distinct = set(seen.values())
    assert len(distinct) == 1, (
        "the shipped port disagrees between files: %s"
        % ", ".join("%s=%d" % (k, v) for k, v in sorted(seen.items())))
    assert distinct == {11400}, (
        "expected the brick4 listener port 11400, got %s" % distinct)
    print("PASS every shipped config points at listener port %d"
          % seen["AMU example config"])


def test_the_retry_budget_is_bounded_and_fits_inside_a_session():
    """How long a unit keeps trying ONE record before it gives up and buffers.

    Both device types must give up well inside the shortest session the
    server will hold open, or a retry burst would outlive the session it is
    being retried on and the unit would be talking into a closed socket.
    """
    nmu_attempts = _constant(NMU_DTLS, r"MAX_RECORD_ATTEMPTS\s*=\s*(\d+)")
    nmu_ack_ms = _constant(NMU_CONFIG, r"OMEGA_ACK_TIMEOUT_MS\s+(\d+)")
    nmu_budget_s = nmu_attempts * nmu_ack_ms / 1000.0

    amu_attempts = _constant(AMU_DTLS, r"MAX_RECORD_ATTEMPTS\s*=\s*([\d.]+)")
    amu_ack_s = _constant(AMU_DTLS, r"ACK_TIMEOUT_S\s*=\s*([\d.]+)")
    amu_budget_s = amu_attempts * amu_ack_s

    assert nmu_attempts == amu_attempts == 3, \
        "both device types are documented as trying a record three times"
    assert nmu_budget_s == 9.0, nmu_budget_s
    assert amu_budget_s == 15.0, amu_budget_s

    floor = config_store.IDLE_TIMEOUT_FLOOR_S
    assert nmu_budget_s < floor and amu_budget_s < floor, \
        ("a retry burst (NMU %.0fs, AMU %.0fs) must finish inside the shortest "
         "session the server will hold (%.0fs)"
         % (nmu_budget_s, amu_budget_s, floor))
    print("PASS retry budget bounded: NMU %.0fs, AMU %.0fs, both under the "
          "%.0fs session floor" % (nmu_budget_s, amu_budget_s, floor))


def test_an_outage_drains_oldest_first_and_deletes_only_what_was_acked():
    """A backlog must come back in the order it happened, and a record may
    only leave the buffer once ITS OWN receipt arrived - never because a
    later record succeeded."""
    import amu_config
    import buffer

    tmp = tempfile.mkdtemp(prefix="omega_drain_")
    original = amu_config.BUFFER_FILE
    amu_config.BUFFER_FILE = os.path.join(tmp, "offline_buffer.json")
    try:
        for index in range(1, 4):
            buffer.append_to_buffer({"event": "7_%d" % index, "seq": index})

        stored = buffer.load_buffer()
        assert [item["seq"] for item in stored] == [1, 2, 3], \
            "the backlog must replay in the order it happened, got " + str(stored)

        # The link comes back, drains two, then drops again mid-flush.
        buffer.remove_delivered(["7_1", "7_2"])
        left = buffer.load_buffer()
        assert [item["event"] for item in left] == ["7_3"], \
            "only acknowledged records may be deleted, left " + str(left)

        # A receipt that matches nothing must not delete anything.
        buffer.remove_delivered(["7_99"])
        assert len(buffer.load_buffer()) == 1, \
            "an unmatched receipt must never delete a pending record"
        print("PASS backlog drains oldest-first and only on its own receipt")
    finally:
        amu_config.BUFFER_FILE = original


def main():
    test_a_server_that_publishes_no_priority_sorts_last()
    test_both_device_types_rank_the_same_fleet_identically()
    test_lower_priority_wins_and_an_unlabelled_server_goes_last()
    test_neither_device_type_drops_a_reading_it_could_not_hand_over()
    test_exactly_one_thread_may_ever_drive_the_dtls_session()
    test_the_dtls_context_is_built_on_the_main_thread_before_any_worker()
    test_every_shipped_config_points_at_the_live_listener_port()
    test_the_retry_budget_is_bounded_and_fits_inside_a_session()
    test_an_outage_drains_oldest_first_and_deletes_only_what_was_acked()
    print("RESULT: PASS - discovery ranking, retry budget, and backlog "
          "ordering proven")
    return 0


if __name__ == "__main__":
    sys.exit(main())
