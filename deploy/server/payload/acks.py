"""Builds the ACK reply payload for one accepted record.

Pure data shaping - must never perform I/O. session.py decides when to send
the result; this module only decides what it contains.
"""

import time

import nmu_mailbox

# The earliest wall-clock time this server may believe it is.
#
# Every ACK carries "t", and a device with no battery-backed clock takes that
# as authoritative - it is the ONLY way an NMU learns the date. So a server
# with a wrong clock does not merely mislabel its own logs: it distributes
# the wrong time to the entire fleet, and every reading stamped during that
# window is permanently mis-dated in the database.
#
# The realistic causes are mundane and all produce a time in the distant
# past: a dead RTC coin cell (many boards then default to 1970, 2000 or
# 2010), a fresh install that has never reached an NTP server, or a boot with
# no network at all. None of them produce a plausible-looking wrong date.
# A floor therefore catches every case that matters.
#
# This mirrors what the NMU already does for itself with
# applyBuildTimeClockFloor() and __DATE__ (stage3_beta_2_dtls.ino). The
# server had no equivalent, which is the asymmetry this closes: the one
# machine that TELLS everyone else the time was the one machine not checking
# its own.
#
# 2026-01-01. Deliberately conservative - comfortably before this project
# ever ran, so it cannot reject a legitimate clock, while still catching
# every default-date failure above.
CLOCK_FLOOR_EPOCH = 1767225600

_clock_warned = False


def clock_is_trustworthy(now=None):
    """May this server's clock be handed to devices as the truth?

    Returns False when the clock is provably wrong. It cannot detect a clock
    that is merely INACCURATE - a few seconds of drift, or an RTC that is a
    minute out - and is not meant to. It exists to stop the fleet being told
    it is 1970.
    """
    return (now if now is not None else time.time()) >= CLOCK_FLOOR_EPOCH


def build_ack(config_store, device_id, event, reauth=False, mailbox_db_path=None):
    """The authenticated ACK for a received event.

    Carries three things back on the same datagram, so a device never needs a
    second round trip:
      - the per-type heartbeat config (dashboard-editable, hot-reloaded);
      - "t", the server's wall-clock time, which is how a device with no
        battery-backed clock (the ESP32 boots believing it is 1970) learns the
        real time without a separate NTP round trip;
      - "reauth", set when the server's session-age policy has expired and the
        device must re-handshake so its certificate is re-validated. See
        ARCHITECTURE.md section 12 for what that does and does not enforce.

    It also carries "idle": how long this server will tolerate silence before
    closing the session. UDP gives the device no hang-up notification, so
    without being told, a device cannot distinguish a live session from one the
    server dropped minutes ago - it finds out by sending into the void and
    burning every retry attempt. Publishing the number lets the device
    re-handshake FIRST, and means the two ends stay in step automatically when
    the heartbeat is changed from the dashboard.

    When mailbox_db_path is given and a question is waiting for this device
    (nmu_mailbox.queue_query), it piggybacks as "q" - the mechanism that lets
    an operator ask a device that never accepts inbound connections (the NMU)
    something, without keeping a socket open for it around the clock. See
    nmu_mailbox.py.
    """
    # "t" is OMITTED, not zeroed, when the clock cannot be trusted. A device
    # that receives no "t" keeps whatever time it already had - the AMU keeps
    # its last offset, the NMU keeps its build floor - which is strictly
    # better than adopting a date we know to be wrong. It resumes the moment
    # NTP corrects the server, with no action needed at either end.
    ack = {"ack": event}
    now = time.time()
    if clock_is_trustworthy(now):
        ack["t"] = int(now)
    else:
        global _clock_warned
        if not _clock_warned:
            _clock_warned = True
            print("CLOCK NOT TRUSTWORTHY: server time %d is before the floor "
                  "%d - withholding 't' from ACKs so the fleet is not given a "
                  "wrong date. Check NTP and the RTC battery."
                  % (int(now), CLOCK_FLOOR_EPOCH))
    ack.update(config_store.config_for(device_id))
    ack["idle"] = int(config_store.idle_timeout_for(device_id))
    if reauth:
        ack["reauth"] = 1
    if mailbox_db_path is not None:
        question = nmu_mailbox.pop_pending_query(mailbox_db_path, device_id)
        if question is not None:
            ack["q"] = question
    return ack
