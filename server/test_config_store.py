"""Unit tests for config_store.py and acks.py (no sockets, no crypto).

Proves the transport-independent behaviour the DTLS layer depends on: fail-
closed startup, hot-reloaded revocation, and per-type heartbeat config
piggybacked correctly onto the ACK.
"""

import json
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import acks
import config_store
import nmu_mailbox
import session

CONFIG = {
    "devices": ["AMU_01", "NMU_01"],
    "nmu": {"cfg_ver": 2, "hb": 45},
    "amu": {"cfg_ver": 2, "hb": 30},
    "revoked": ["AMU_09"],
}


def _write_config(path):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(CONFIG, handle)


def test_config_and_ack():
    tmp = tempfile.mkdtemp(prefix="omega_configstore_")
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    store = config_store.ConfigStore(cfg_path)

    assert store.config_for("NMU_01") == {"cfg_ver": 2, "hb": 45}, "NMU config"
    assert store.config_for("AMU_01") == {"cfg_ver": 2, "hb": 30}, "AMU config"
    assert store.is_allowed("AMU_01") and store.is_allowed("NMU_01"), "listed allowed"
    assert not store.is_allowed("AMU_09"), "revoked refused"
    assert not store.is_allowed("AMU_77"), "unlisted refused when roster non-empty"

    ack = acks.build_ack(store, "NMU_01", "123_7")
    # The ACK now also carries "t", the server's wall-clock time. The ESP32 has
    # no battery-backed clock and boots believing it is 1970, so without this
    # every timestamp is wrong and certificate validity checks fail. Piggybacking
    # it costs nothing - it rides on a datagram that was already being sent.
    server_time = ack.pop("t", None)
    assert server_time is not None, "ACK must carry server time for clockless devices"
    assert server_time > 1700000000, "server time must be a real epoch, got " + str(server_time)
    # ...and "idle", the silence the server will tolerate before hanging up,
    # asserted in detail by test_ack_publishes_idle_limit.
    ack.pop("idle", None)
    assert ack == {"ack": "123_7", "cfg_ver": 2, "hb": 45}, "ack carries heartbeat: " + str(ack)

    # reauth is absent on a normal ACK and set when the server's session-age
    # policy expires. That forced re-handshake is the ONLY point a certificate
    # is re-verified, so it is the only place revocation can be enforced.
    assert "reauth" not in acks.build_ack(store, "NMU_01", "123_8"), \
        "normal ACK must not ask for re-auth"
    assert acks.build_ack(store, "NMU_01", "123_9", reauth=True).get("reauth") == 1, \
        "expired session must ask for re-auth"
    print("PASS config + ACK piggyback + time sync + reauth flag")


def test_revocation_fails_closed():
    """A missing or corrupt device config must STOP the server, not be ignored.

    With no config the revocation list is empty, so a revoked - stolen, retired,
    compromised - device would still be admitted and the ban list would be
    decoration. brick3 failed open here: the installer copied a config that did
    not exist, so revocation silently did nothing.
    """
    tmp = tempfile.mkdtemp(prefix="omega_failclosed_")

    missing = os.path.join(tmp, "does_not_exist.json")
    try:
        config_store.ConfigStore(missing)
        raise AssertionError("missing config was accepted - revocation would fail open")
    except config_store.MissingDeviceConfig:
        pass

    corrupt = os.path.join(tmp, "corrupt.json")
    with open(corrupt, "w", encoding="utf-8") as handle:
        handle.write("{ this is not valid json")
    try:
        config_store.ConfigStore(corrupt)
        raise AssertionError("corrupt config was accepted - revocation would fail open")
    except config_store.MissingDeviceConfig:
        pass

    print("PASS revocation fails closed on missing and corrupt config")


def test_revocation_is_hot_reloaded():
    """Revoking a device must take effect without restarting the server."""
    tmp = tempfile.mkdtemp(prefix="omega_revoke_")
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    store = config_store.ConfigStore(cfg_path)
    assert store.is_allowed("NMU_01"), "NMU_01 starts allowed"

    with open(cfg_path, "r", encoding="utf-8") as handle:
        doc = json.load(handle)
    doc["revoked"] = list(doc.get("revoked", [])) + ["NMU_01"]
    # mtime has 1 ns resolution here but bump it explicitly to be safe
    time.sleep(0.01)
    with open(cfg_path, "w", encoding="utf-8") as handle:
        json.dump(doc, handle)

    assert not store.is_allowed("NMU_01"), \
        "revocation must take effect with no restart"
    print("PASS revocation hot-reloads with no restart")


def test_idle_timeout_follows_heartbeat():
    """A session must tolerate a quiet spell longer than the device's own
    heartbeat, or the heartbeat can never keep the session warm.

    Regression test for a real field bug (2026-08-17): a flat 30 s idle
    timeout closed the NMU's session between sporadic noise events, so the
    device sent into a session the server had already dropped, burned every
    retry attempt, and buffered - one long LED blink then a burst of triple
    blinks as the buffer drained.
    """
    tmp = tempfile.mkdtemp(prefix="omega_idle_")
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    store = config_store.ConfigStore(cfg_path)

    for device, block in (("NMU_01", "nmu"), ("AMU_01", "amu")):
        hb_s = CONFIG[block]["hb"] * 60
        idle = store.idle_timeout_for(device)
        assert idle > hb_s, \
            "%s idle window (%.0fs) must exceed its heartbeat (%ds)" % (device, idle, hb_s)

    # A config block with no heartbeat at all must still yield a workable
    # window rather than a zero-length one that closes the session instantly.
    doc = dict(CONFIG)
    doc["nmu"] = {"cfg_ver": 2}
    time.sleep(0.01)
    with open(cfg_path, "w", encoding="utf-8") as handle:
        json.dump(doc, handle)
    assert store.idle_timeout_for("NMU_01") == config_store.IDLE_TIMEOUT_FLOOR_S, \
        "heartbeat-less config falls back to the floor"

    # A careless dashboard value must not pin a server thread open forever.
    doc["nmu"] = {"cfg_ver": 2, "hb": 10000}
    time.sleep(0.01)
    with open(cfg_path, "w", encoding="utf-8") as handle:
        json.dump(doc, handle)
    assert store.idle_timeout_for("NMU_01") == config_store.IDLE_TIMEOUT_CEILING_S, \
        "absurd heartbeat is capped at the ceiling"
    print("PASS idle timeout tracks heartbeat, with floor and ceiling")


def test_ack_publishes_idle_limit():
    """The device cannot see that the server hung up (UDP says nothing), so the
    server must TELL it the idle limit - otherwise the two ends drift apart the
    moment the heartbeat is changed from the dashboard, and the device is back
    to discovering closed sessions by burning its retries."""
    tmp = tempfile.mkdtemp(prefix="omega_ackidle_")
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    store = config_store.ConfigStore(cfg_path)

    ack = acks.build_ack(store, "NMU_01", "800_1")
    assert ack["idle"] == int(store.idle_timeout_for("NMU_01")), \
        "ACK carries the same idle limit the session loop enforces"
    assert ack["idle"] > CONFIG["nmu"]["hb"] * 60, "published limit outlives the heartbeat"
    print("PASS ACK publishes the server's idle limit")


def test_ack_piggybacks_a_pending_nmu_question():
    """The whole point of the mailbox: a question queued for a device that
    never accepts inbound connections must ride out on its next ack, and
    must not still be sitting there for the NEXT ack after that (delivered
    exactly once)."""
    tmp = tempfile.mkdtemp(prefix="omega_ackq_")
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    store = config_store.ConfigStore(cfg_path)
    mailbox_db = os.path.join(tmp, "mailbox.db")

    ack_before = acks.build_ack(store, "NMU_01", "1_1", mailbox_db_path=mailbox_db)
    assert "q" not in ack_before, "no question queued yet - ack must not carry one"

    nmu_mailbox.queue_query(mailbox_db, "NMU_01", {"cmd": "read_now"})
    ack_with_q = acks.build_ack(store, "NMU_01", "1_2", mailbox_db_path=mailbox_db)
    assert ack_with_q.get("q") == {"cmd": "read_now"}, \
        "a queued question must appear as 'q' on the very next ack"

    ack_after = acks.build_ack(store, "NMU_01", "1_3", mailbox_db_path=mailbox_db)
    assert "q" not in ack_after, "the question must not repeat on a later ack"
    print("PASS ack piggybacks a pending NMU question exactly once")


def test_event_id_contract():
    valid = ("1_1", "1787282343_4294967295", "0_0")
    invalid = (None, 1, "", "1", "1_", "_1", "1.2", "a_1",
               "1_2_3", "1_2 " + ("x" * 60))
    assert all(session.valid_event_id(value) for value in valid), \
        "generated session_counter IDs must be accepted"
    assert not any(session.valid_event_id(value) for value in invalid), \
        "malformed or non-string event IDs must be rejected before ACK/storage"
    print("PASS event ID contract rejects malformed packet identifiers")


def test_a_wrong_server_clock_is_not_distributed_to_the_fleet():
    """The server is the only source of time for a device with no battery.

    A wrong clock here does not just mislabel the server's own logs - every
    device adopts it, and every reading taken during that window is
    permanently mis-dated in the database. The realistic causes (dead RTC
    cell, fresh install that never reached NTP, boot with no network) all
    produce a date in the distant past, so a floor catches them all.
    """
    assert acks.clock_is_trustworthy(acks.CLOCK_FLOOR_EPOCH), \
        "the floor itself must be acceptable, not rejected off-by-one"
    assert acks.clock_is_trustworthy(acks.CLOCK_FLOOR_EPOCH + 1)
    assert not acks.clock_is_trustworthy(0), "epoch 0 must be refused"
    assert not acks.clock_is_trustworthy(946684800), \
        "a year-2000 RTC default must be refused"
    assert not acks.clock_is_trustworthy(acks.CLOCK_FLOOR_EPOCH - 1)
    print("PASS a provably wrong server clock is refused by the floor")


def test_the_ack_omits_time_rather_than_sending_a_wrong_one():
    """Omitted, not zeroed. A device that receives no "t" keeps the time it
    already had, which beats adopting a date known to be wrong."""
    tmp = tempfile.mkdtemp(prefix="omega_clockfloor_")
    cfg_path = os.path.join(tmp, "device_config.json")
    _write_config(cfg_path)
    store = config_store.ConfigStore(cfg_path)

    real = acks.time.time
    try:
        acks.time.time = lambda: 100.0          # long before the floor
        acks._clock_warned = False
        bad = acks.build_ack(store, "NMU_01", "s_1")
        assert "t" not in bad, \
            "a server that knows its clock is wrong must not hand it out"
        assert bad["ack"] == "s_1", "the ACK itself must still be delivered"

        acks.time.time = lambda: float(acks.CLOCK_FLOOR_EPOCH + 5000)
        good = acks.build_ack(store, "NMU_01", "s_2")
        assert good["t"] == acks.CLOCK_FLOOR_EPOCH + 5000, \
            "a trustworthy clock must still be published as before"
    finally:
        acks.time.time = real
    print("PASS the ACK omits 't' when the clock is wrong, and resumes when fixed")


def main():
    test_config_and_ack()
    test_revocation_fails_closed()
    test_revocation_is_hot_reloaded()
    test_idle_timeout_follows_heartbeat()
    test_ack_publishes_idle_limit()
    test_ack_piggybacks_a_pending_nmu_question()
    test_event_id_contract()
    test_a_wrong_server_clock_is_not_distributed_to_the_fleet()
    test_the_ack_omits_time_rather_than_sending_a_wrong_one()
    print("RESULT: PASS - config store + ack behaviour verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
