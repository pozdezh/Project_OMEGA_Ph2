"""Unit tests for nmu_mailbox.py - the queue-a-question-for-later mechanism
for devices (the NMU) that never accept inbound connections.

No sockets, no DTLS - pure SQLite state transitions, so this runs on any
platform with no hardware and no wolfssl.
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nmu_mailbox


def _tmp_db():
    fd, path = tempfile.mkstemp(prefix="omega_mailbox_", suffix=".db")
    os.close(fd)
    os.remove(path)  # nmu_mailbox creates its own schema fresh
    return path


def test_no_question_pops_nothing():
    db = _tmp_db()
    assert nmu_mailbox.pop_pending_query(db, "NMU_01") is None
    print("PASS no question queued -> pop returns None")


def test_queue_then_pop_is_exactly_once():
    db = _tmp_db()
    nmu_mailbox.queue_query(db, "NMU_01", {"cmd": "read_now"})
    first = nmu_mailbox.pop_pending_query(db, "NMU_01")
    assert first == {"cmd": "read_now"}, "first pop returns the queued question"
    second = nmu_mailbox.pop_pending_query(db, "NMU_01")
    assert second is None, "a question is delivered exactly once, not on every ack"
    print("PASS question is popped exactly once")


def test_queue_is_per_device():
    db = _tmp_db()
    nmu_mailbox.queue_query(db, "NMU_01", {"cmd": "read_now"})
    assert nmu_mailbox.pop_pending_query(db, "NMU_02") is None, \
        "a question for one device must not leak to another"
    assert nmu_mailbox.pop_pending_query(db, "NMU_01") == {"cmd": "read_now"}
    print("PASS questions are isolated per device_id")


def test_requeue_replaces_stale_question():
    db = _tmp_db()
    nmu_mailbox.queue_query(db, "NMU_01", {"cmd": "read_now"})
    nmu_mailbox.queue_query(db, "NMU_01", {"cmd": "status"})
    only = nmu_mailbox.pop_pending_query(db, "NMU_01")
    assert only == {"cmd": "status"}, \
        "re-queuing replaces an unanswered question rather than stacking it"
    print("PASS re-queuing replaces, does not stack")


def test_reply_round_trip():
    db = _tmp_db()
    nmu_mailbox.queue_query(db, "NMU_01", {"cmd": "read_now"})
    nmu_mailbox.pop_pending_query(db, "NMU_01")  # simulates the ack going out
    nmu_mailbox.deliver_reply(db, "NMU_01", {"db": 42.5, "buffered": 0})
    got = nmu_mailbox.poll_reply(db, "NMU_01", timeout_s=0.01)
    assert got["db"] == 42.5 and got["buffered"] == 0, got
    assert isinstance(got["answered_at"], int), \
        "answered_at is stamped so a live tool call can state exactly when the device replied: " + str(got)
    print("PASS reply round-trips from deliver to poll, with an answer timestamp")


def test_poll_claims_the_reply_exactly_once():
    db = _tmp_db()
    nmu_mailbox.deliver_reply(db, "NMU_01", {"db": 1.0})
    first = nmu_mailbox.poll_reply(db, "NMU_01", timeout_s=0.01)
    second = nmu_mailbox.poll_reply(db, "NMU_01", timeout_s=0.01)
    assert first["db"] == 1.0 and second is None, \
        "a second poll must not re-read a stale answer from a past question"
    print("PASS reply is claimed exactly once")


def test_poll_times_out_without_deleting_the_question():
    """The device may simply not have spoken yet. A timed-out poll must not
    cancel the still-pending question - it should still be answerable later."""
    db = _tmp_db()
    nmu_mailbox.queue_query(db, "NMU_01", {"cmd": "status"})
    got = nmu_mailbox.poll_reply(db, "NMU_01", timeout_s=0.05, interval_s=0.02)
    assert got is None, "no reply has arrived yet"
    still_queued = nmu_mailbox.pop_pending_query(db, "NMU_01")
    assert still_queued == {"cmd": "status"}, \
        "a timed-out poll must not have consumed or cancelled the question"
    print("PASS a poll timeout does not cancel the pending question")


def test_poll_wakes_up_as_soon_as_the_reply_lands():
    """Simulates the reply arriving mid-wait (a real device answering
    somewhere in the middle of the operator's poll window) by having the
    fake `sleep` callback deliver it after one tick."""
    db = _tmp_db()
    calls = []

    def fake_sleep(interval):
        calls.append(interval)
        if len(calls) == 1:
            nmu_mailbox.deliver_reply(db, "NMU_01", {"db": 7.0})

    got = nmu_mailbox.poll_reply(db, "NMU_01", timeout_s=5.0, interval_s=0.01,
                                 sleep=fake_sleep)
    assert got["db"] == 7.0
    assert len(calls) == 1, "must return on the very next check, not wait out the full timeout"
    print("PASS poll returns as soon as a reply lands, not after the full timeout")


def main():
    test_no_question_pops_nothing()
    test_queue_then_pop_is_exactly_once()
    test_queue_is_per_device()
    test_requeue_replaces_stale_question()
    test_reply_round_trip()
    test_poll_claims_the_reply_exactly_once()
    test_poll_times_out_without_deleting_the_question()
    test_poll_wakes_up_as_soon_as_the_reply_lands()
    print("RESULT: PASS - nmu_mailbox verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
