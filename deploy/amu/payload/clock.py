"""Whether this unit's clock can be trusted, and what to stamp when it cannot.

See ARCHITECTURE.md 26. The Raspberry Pi has no battery-backed clock
(`RTC time: n/a`), so with no internet it boots believing an arbitrary time.
The server already substitutes its own arrival time for a record whose ts is
0, so an untrusted clock must send 0 rather than a confident wrong number.
"""

import json
import os
import struct
import subprocess
import time

_offset_s = 0.0
_server_synced = False


def _ntp_synced():
    try:
        result = subprocess.run(["timedatectl", "show", "-p",
                                 "NTPSynchronized", "--value"],
                                capture_output=True, timeout=5, text=True)
        return result.stdout.strip() == "yes"
    except Exception:
        return False


_ntp_at_start = _ntp_synced()


def apply_server_time(server_epoch):
    """Adopt the server's wall clock from an ACK. No privilege needed: the
    offset is applied to our own timestamps rather than to the system clock."""
    global _offset_s, _server_synced
    try:
        server_epoch = float(server_epoch)
    except (TypeError, ValueError):
        return False
    if server_epoch <= 0:
        return False
    _offset_s = server_epoch - time.time()
    _server_synced = True
    return True


# Identifies THIS process run - the fallback identity, used only where the
# kernel cannot tell us about the boot (i.e. off a Linux box, in tests).
RUN_ID = os.urandom(8).hex()


def _boot_identity():
    """Something that changes on REBOOT but not on a process restart.

    The first version of this used time.monotonic() and a per-process id, and
    the field showed why that is not enough: rung 2 of the recovery ladder
    restarts the process, so by the time a buffered reading is finally sent it
    is almost always a DIFFERENT process that sends it. The marker was then
    correctly refused - safe, but useless in the one case that actually
    happens. AMU_15, 2026-08-27: readings taken across 23 minutes after a
    self-reboot all landed in the same minute anyway.

    The kernel's boot id survives a process restart and changes only when the
    machine reboots, which is exactly the boundary that matters: across a
    reboot the elapsed-time reference is genuinely gone and the reading must
    stay undated rather than be invented.
    """
    try:
        with open("/proc/sys/kernel/random/boot_id", "r", encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return RUN_ID


def _since_boot_s():
    """Seconds since the machine booted. Needs no wall clock and never jumps,
    and unlike time.monotonic() it is the same reference for every process on
    the box."""
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as handle:
            return float(handle.read().split()[0])
    except (OSError, ValueError, IndexError):
        return time.monotonic()


# Written immediately before the recovery ladder reboots this unit, and
# consumed once on the way back up. Its whole job is to tell the two kinds of
# restart apart.
#
# The Pi has NO hardware clock. systemd saves the time to disk and restores it
# at boot, so after a REBOOT WE PERFORMED the clock comes back correct - the
# save happens on the way down. After being UNPLUGGED it comes back stale by
# however long the unit sat without power, and must not be believed.
#
# Nothing else can tell those apart from inside: both look like a fresh boot
# with no network. So the unit leaves itself a note before it goes.
REBOOT_MARKER_PATH = os.environ.get(
    "OMEGA_REBOOT_MARKER",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "reboot_marker.json"))

# How long after a deliberate reboot the note may still be believed. A reboot
# takes well under a minute; anything older is a note left by some earlier
# reboot that a power cut has since invalidated.
PLANNED_REBOOT_WINDOW_S = 600.0

_restored_across_planned_reboot = False


def note_planned_reboot():
    """Leave the note. Called only by the ladder's reboot rung."""
    try:
        tmp = REBOOT_MARKER_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump({"at": time.time()}, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, REBOOT_MARKER_PATH)
        return True
    except OSError:
        return False


def _consume_planned_reboot_marker():
    """Read the note and destroy it, so it can never be believed twice."""
    try:
        with open(REBOOT_MARKER_PATH, "r", encoding="utf-8") as handle:
            written_at = float(json.load(handle)["at"])
    except (OSError, ValueError, KeyError, TypeError):
        return False
    finally:
        try:
            os.remove(REBOOT_MARKER_PATH)
        except OSError:
            pass
    now = time.time()
    # The restored clock sits at roughly the moment of the reboot, so the gap
    # is about this boot's own age. A large gap means the restored clock is
    # NOT continuous with the note, i.e. power was lost.
    return 0.0 <= (now - written_at) <= PLANNED_REBOOT_WINDOW_S


_restored_across_planned_reboot = _consume_planned_reboot_marker()


def trusted():
    # The third source is the one added on 2026-08-27: a clock restored across
    # a reboot THIS UNIT CHOSE to perform. Refusing it meant a unit that healed
    # itself came back unable to date anything until the server answered - and
    # a reboot is precisely when the server cannot be reached, so that was the
    # worst possible moment to forget the time.
    return _server_synced or _ntp_at_start or _restored_across_planned_reboot


def capture_marker():
    """What to attach to a record whose time is not yet known, so it can be
    dated correctly later. See resolve_stamp()."""
    return {"up": _since_boot_s(), "boot": _boot_identity()}


def resolve_stamp(stamped_ts, marker):
    """The real capture time of a record that had to be stamped 0.

    The AMU has the same defect the NMU had: a record captured before the
    clock could be trusted is sent with ts = 0, and the server then dates it
    by ARRIVAL. Correct for a live record; wrong for one that waited in the
    buffer, which collapses a whole outage onto the reconnect moment. Proven
    on the NMU on 2026-08-27, when 84 records spanning twenty minutes all
    landed inside six seconds.

    time.monotonic() needs no clock and never jumps, so however long a record
    has waited is knowable - within one process run. Returns 0, never a guess,
    when the time is still unknown or the record outlived its own run.
    """
    if stamped_ts:
        return stamped_ts
    if not trusted() or not isinstance(marker, dict):
        return 0
    if marker.get("boot") != _boot_identity():
        # Captured before this machine rebooted. The reference it was measured
        # against no longer exists, so the wait cannot be known. Say so rather
        # than invent it.
        return 0
    try:
        waited_s = _since_boot_s() - float(marker["up"])
    except (TypeError, ValueError, KeyError):
        return 0
    if waited_s < 0:
        return 0
    resolved = int(now() - waited_s)
    return resolved if resolved > 0 else 0


def now():
    return time.time() + _offset_s


def stamp():
    """Epoch seconds to put on a record, or 0 to let the server stamp it."""
    if not trusted():
        return 0
    return int(now())


def new_session_id():
    """A session id that does not depend on the clock.

    int(time.time()) collides across reboots when the clock is wrong, and two
    sessions sharing an id make their event ids collide - which the server's
    (id, event) unique index then silently discards as duplicates.
    """
    return struct.unpack("<I", os.urandom(4))[0]


def status():
    return {"trusted": trusted(), "server_synced": _server_synced,
            "ntp_at_start": _ntp_at_start, "offset_s": round(_offset_s, 3)}
