"""Autonomous recovery ladder for the AMU. See ARCHITECTURE.md 25.

Escalation is measured from the last time the server was actually reached,
never from a counter that only moves while a session already exists.

Every rung is also held clear of the device's OWN heartbeat. A unit that
legitimately has nothing to say until its next heartbeat must never read its
own silence as a fault: with a 60 min heartbeat and a fixed 30 min reboot
rung, a perfectly healthy unit would reboot itself twice per heartbeat,
forever. The heartbeat is the only evidence this device ever gets that the
link works, so no rung may sit closer than one heartbeat away from it.
"""

import json
import os
import subprocess
import time

# Where the contact clock is kept so it survives rung 2.
#
# The clock used to live only in memory, and rung 2's action is "restart the
# process" - which wiped it. The new process started counting from zero, so a
# unit walked reconnect -> restart -> reconnect -> restart forever and could
# never count high enough to reach the reboot rung. AMU_15 proved it on
# 2026-08-26: three hours offline, rungs 1 and 2 fired repeatedly, and
# `journalctl --list-boots` showed ONE boot - the reboot rung never ran.
#
# The rule this file now implements: a PROCESS RESTART keeps the clock, a
# REBOOT resets it. That makes the top rung reachable, and stops a rebooted
# unit from reading its own pre-reboot silence and rebooting again at once.
CONTACT_STATE_PATH = os.environ.get(
    "OMEGA_CONTACT_STATE",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "contact_state.json"))

# Contact is noted on every delivered record. Writing the card that often
# would wear it for no gain, and being up to a minute stale is nothing
# against rungs measured in tens of minutes.
PERSIST_INTERVAL_S = 60.0

# A persisted clock older than this is not a long outage, it is a broken
# clock (an unsynced RTC, a jumped NTP correction). Distrust it and start
# fresh rather than reboot on the strength of it.
MAX_PERSISTED_AGE_S = 86400.0

RADIO_RESET_S = 600.0
PROCESS_RESTART_S = 1200.0
REBOOT_S = 1800.0

RADIO_RESET_BEATS = 1.5
PROCESS_RESTART_BEATS = 2.5
REBOOT_BEATS = 3.5

STAGE_NONE = 0
STAGE_RADIO = 1
STAGE_RESTART = 2

_last_contact = None
_stage = STAGE_NONE
_persisted_at = 0.0


def _read_persisted_contact(moment):
    """The last contact this unit recorded before its process died, or None.

    None means "no usable record" - never a guess. A value from the future or
    from before MAX_PERSISTED_AGE_S is refused, because acting on a bad clock
    here means rebooting a healthy unit.
    """
    try:
        with open(CONTACT_STATE_PATH, "r", encoding="utf-8") as handle:
            stored = json.load(handle)
        value = float(stored["last_contact"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if value > moment or moment - value > MAX_PERSISTED_AGE_S:
        return None
    try:
        stage = int(stored.get("stage", STAGE_NONE))
    except (TypeError, ValueError):
        stage = STAGE_NONE
    return (value, stage if stage in (STAGE_NONE, STAGE_RADIO, STAGE_RESTART)
            else STAGE_NONE)


def _write_contact(value, stage=STAGE_NONE):
    """Record the contact clock AND how far up the ladder we got, atomically.

    The stage has to travel with the clock. Persisting only the clock - which
    is what shipped first - made rung 2 fire over and over: the restarted
    process inherited the outage but not the memory of having already acted on
    it, so the "this rung has fired" guard was clear again and it restarted
    itself roughly every 100 s until the reboot rung finally caught it. Seen on
    AMU_15 on 2026-08-27: rung 2 at 1201s, then 1300, 1404, 1510, 1612, 1721.
    """
    tmp = CONTACT_STATE_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump({"last_contact": float(value), "stage": int(stage)}, handle)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, CONTACT_STATE_PATH)


def forget_persisted_contact():
    """Discard the clock. Called only when the unit is about to reboot - the
    one event that is allowed to be a fresh start."""
    global _persisted_at
    _persisted_at = 0.0
    try:
        os.remove(CONTACT_STATE_PATH)
    except OSError:
        pass


def _remember_stage(stage):
    """Persist how far up the ladder this outage has got, without moving the
    clock. Best-effort: failing to record it costs a repeated rung, not the
    recovery itself."""
    if _last_contact is None:
        return
    try:
        _write_contact(_last_contact, stage)
    except OSError:
        pass


def thresholds(heartbeat_s=0.0):
    """The three rungs for a device beating at heartbeat_s.

    Derived rather than hardcoded so that an operator raising the heartbeat
    from the dashboard cannot silently arm a reboot loop on every quiet unit
    in the fleet - the failure mode this function exists to make impossible.
    """
    try:
        beat = float(heartbeat_s)
    except (TypeError, ValueError):
        beat = 0.0
    if beat < 0.0:
        beat = 0.0
    return (max(RADIO_RESET_S, beat * RADIO_RESET_BEATS),
            max(PROCESS_RESTART_S, beat * PROCESS_RESTART_BEATS),
            max(REBOOT_S, beat * REBOOT_BEATS))


def reset(now=None):
    """Start the clock fresh at `now`, discarding anything persisted."""
    global _last_contact, _stage, _persisted_at
    _last_contact = time.time() if now is None else now
    _stage = STAGE_NONE
    _persisted_at = _last_contact
    try:
        _write_contact(_last_contact, STAGE_NONE)
    except OSError:
        _persisted_at = 0.0


def note_contact(now=None):
    global _last_contact, _stage, _persisted_at
    _last_contact = time.time() if now is None else now
    _stage = STAGE_NONE
    if _last_contact - _persisted_at < PERSIST_INTERVAL_S:
        return
    try:
        _write_contact(_last_contact, STAGE_NONE)
        _persisted_at = _last_contact
    except OSError as error:
        # Not fatal: the in-memory clock still works, only its ability to
        # survive rung 2 is lost. Say so rather than failing silently.
        print("contact clock not persisted (%s) - a process restart will "
              "reset the recovery ladder" % error, flush=True)


def silent_for(now=None):
    if _last_contact is None:
        return 0.0
    return (time.time() if now is None else now) - _last_contact


def _do_reboot():
    """Ask the OS to reboot. Returns True if the request was accepted.

    A process restart cannot clear a wedged kernel WiFi driver, so this rung
    has to exist; it needs a sudoers rule the installer adds, and degrades to
    a process restart when that rule is absent rather than doing nothing.
    """
    try:
        result = subprocess.run(["sudo", "-n", "/sbin/reboot"],
                                capture_output=True, timeout=10)
        return result.returncode == 0
    except Exception:
        return False


def check(now=None, on_radio_reset=None, reboot=_do_reboot, exit_fn=None,
          heartbeat_s=0.0):
    """Advance the ladder. Call often; costs one subtraction when healthy.

    Returns the stage acted on this call, or STAGE_NONE.
    """
    global _stage, _last_contact, _persisted_at
    if _last_contact is None:
        # A fresh process. Adopt the clock its predecessor left behind, so a
        # rung-2 restart continues the same outage instead of restarting the
        # ladder. With nothing to adopt - a real boot, or a first ever start -
        # the clock starts now, which is what stops a unit powered on before
        # its server from reading an infinitely old outage.
        moment = time.time() if now is None else now
        adopted = _read_persisted_contact(moment)
        if adopted is None:
            reset(moment)
        else:
            _last_contact, _stage = adopted
            _persisted_at = _last_contact
        return STAGE_NONE

    radio_at, restart_at, reboot_at = thresholds(heartbeat_s)
    silent = silent_for(now)
    exit_fn = exit_fn or (lambda code: os._exit(code))

    if silent >= reboot_at:
        print("FATAL: no server contact for %.0fs - rebooting this unit "
              "(buffered readings are on disk and survive)" % silent, flush=True)
        # A reboot is the one action allowed to be a fresh start. Without
        # this the unit would come back up, adopt a clock already past the
        # reboot rung, and reboot again immediately - a tight loop, which is
        # far worse than the stall this whole change exists to fix.
        forget_persisted_contact()
        # ...but leave a note first. The Pi has no hardware clock, and the only
        # thing that separates "I rebooted myself to recover" from "someone
        # pulled the plug" is whether this unit chose it. Across our own reboot
        # the restored clock is correct and the readings taken afterwards can
        # be dated properly; across a power cut it is stale and must not be
        # believed. See clock.note_planned_reboot().
        try:
            import clock
            clock.note_planned_reboot()
        except Exception:
            pass
        if reboot():
            return STAGE_RESTART
        print("reboot not permitted (missing sudoers rule) - restarting the "
              "process instead", flush=True)
        exit_fn(1)
        return STAGE_RESTART

    if silent >= restart_at and _stage < STAGE_RESTART:
        _stage = STAGE_RESTART
        # Written BEFORE the process is told to die, or the successor inherits
        # the outage without the memory of this rung and repeats it.
        _remember_stage(STAGE_RESTART)
        print("no server contact for %.0fs - restarting the process so "
              "systemd rebuilds every socket" % silent, flush=True)
        exit_fn(1)
        return STAGE_RESTART

    if silent >= radio_at and _stage < STAGE_RADIO:
        _stage = STAGE_RADIO
        _remember_stage(STAGE_RADIO)
        print("no server contact for %.0fs - forcing a WiFi reconnect" % silent,
              flush=True)
        if on_radio_reset is not None:
            on_radio_reset()
        return STAGE_RADIO

    return STAGE_NONE
