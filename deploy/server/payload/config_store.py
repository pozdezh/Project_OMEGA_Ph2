"""Device roster, revocation, and per-type heartbeat config for brick4.

Owns device_config.json: which device ids may connect, which are revoked,
and the heartbeat interval served back to each device type inside its ACK.
Must never know about sockets - the DTLS layer hands it a device_id it has
already authenticated and asks yes/no questions.
"""

import json
import os

NMU_PREFIX = "NMU"

# Idle timeout FLOOR, for a device whose heartbeat config cannot be read.
# A flat 30 s used to be the timeout for EVERY session, and it was the root
# cause of a real field symptom (2026-08-17): the NMU only transmits when it
# hears noise, so it is routinely quiet far longer than 30 s. The server hung
# up while the device still believed the session was open - and UDP gives no
# hang-up notification, so the device only found out by sending into the void,
# burning all 3 retry attempts before buffering the event and re-handshaking.
# Visible on the unit as one long LED blink (the failure) followed by a burst
# of triple blinks (the buffer draining). The AMU hid the same bug simply by
# transmitting more often.
IDLE_TIMEOUT_FLOOR_S = 30.0

# A session must survive a quiet spell longer than the device's own heartbeat,
# or the heartbeat can never do its job of keeping the session warm. 2.5x
# tolerates two missed heartbeats before the server gives up on a peer.
IDLE_HEARTBEAT_MULTIPLIER = 2.5

# ...but never pin a server thread to a session idle beyond this, however long
# a heartbeat the dashboard asks for. Set well clear of any realistic heartbeat
# (3 h covers 2.5x a 60-minute beat) so that in normal operation it never
# binds and session reuse still works - an idle thread parked in select()
# costs almost nothing, whereas clamping below the heartbeat would throw away
# the reuse this whole design exists to get.
#
# When an absurd dashboard value DOES hit this cap, nothing breaks: the device
# is TOLD the resulting value in every ACK (see acks.build_ack) and
# re-handshakes before it expires, so it degrades to one handshake per record
# rather than to the failure-and-buffer cycle that motivated all of this.
IDLE_TIMEOUT_CEILING_S = 10800.0


def write_atomic(path, doc):
    """Replace device_config.json in one indivisible step.

    Opening the real path with "w" truncates it before a single byte is
    written, so a crash, a full disk, or two overlapping writers can leave an
    empty or half-written file. This file holds the REVOCATION LIST, and the
    listener deliberately refuses to start without a parseable one - so that
    window turns a routine dashboard click into a fleet-wide outage on the
    next restart. Temp file plus fsync plus rename cannot be half-done: the
    reader sees either the old file or the new one.
    """
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(doc, handle, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


class MissingDeviceConfig(Exception):
    """The device config (allow-list + revocation list) is absent or unreadable.

    Fatal on purpose. Without this file the revocation list is empty, so a
    revoked - stolen, retired, compromised - device would still be admitted and
    the ban list would be decoration. Failing loudly at startup surfaces the
    problem on the installer's desk instead of silently in a residence.
    """


class ConfigStore:
    """Per-type heartbeat config (NMU vs AMU) served inside the ACK, plus the
    device allow-list and revocation list. Hot-reloads on device_config.json
    change so the dashboard can push a new heartbeat or revoke a unit with no
    restart. A revoked device_id is refused even while its certificate is still
    cryptographically valid (a lightweight certificate revocation list)."""

    def __init__(self, config_path, require_file=True):
        self._path = config_path
        self._mtime_ns = -1
        self._doc = {}
        if require_file:
            self._require_readable()
        self._maybe_reload()

    def _require_readable(self):
        """Fail closed: refuse to construct without a parseable config."""
        try:
            with open(self._path, "r", encoding="utf-8") as handle:
                json.load(handle)
        except OSError as error:
            raise MissingDeviceConfig(
                "device config not readable at " + str(self._path) + ": " +
                str(error) + " - refusing to start, because with no config the "
                "revocation list is empty and revoked devices would be admitted")
        except ValueError as error:
            raise MissingDeviceConfig(
                "device config at " + str(self._path) + " is not valid JSON: " +
                str(error) + " - refusing to start rather than run with an "
                "empty revocation list")

    def _maybe_reload(self):
        try:
            mtime_ns = os.stat(self._path).st_mtime_ns
        except OSError:
            return
        if mtime_ns == self._mtime_ns:
            return
        try:
            with open(self._path, "r", encoding="utf-8") as handle:
                self._doc = json.load(handle)
            self._mtime_ns = mtime_ns
        except (OSError, ValueError) as error:
            print("Config reload failed: " + str(error))

    def roster(self):
        self._maybe_reload()
        return list(self._doc.get("devices", []))

    def is_allowed(self, device_id):
        """May this device connect?

        Three separate questions, in this order:

          1. Has it been revoked? Then no, whatever else is true.
          2. Is the roster EMPTY? Then yes - an empty roster means "any
             device holding a certificate this CA signed is welcome", which
             is the normal setting. Entry is gated by the certificate.
          3. Otherwise the roster is a closed guest list, and only the names
             written on it may connect.

        The trap in step 3, which has bitten once: adding the FIRST name to
        an empty roster does not admit one more device, it switches the door
        from open to invitation-only and locks out every unit already in the
        field. See provisioning/pendrive/make-units, which refuses to do it.
        """
        self._maybe_reload()
        if self.is_revoked(device_id):
            return False

        roster = self.roster()
        roster_admits_everyone = not roster
        this_device_is_listed = device_id in roster
        return roster_admits_everyone or this_device_is_listed

    def is_revoked(self, device_id):
        self._maybe_reload()
        return device_id in set(self._doc.get("revoked", []))

    def config_for(self, device_id):
        self._maybe_reload()
        block = "nmu" if device_id.startswith(NMU_PREFIX) else "amu"
        return dict(self._doc.get(block, {}))

    def idle_timeout_for(self, device_id):
        """How long this device may stay quiet before the server closes its
        session, derived from the heartbeat the server itself hands that device.

        Deriving it (rather than hardcoding) means the dashboard's heartbeat
        control cannot silently desynchronise the two ends: raise the heartbeat
        and the tolerated idle window rises with it, automatically.
        """
        try:
            hb_minutes = float(self.config_for(device_id).get("hb", 0))
        except (TypeError, ValueError):
            hb_minutes = 0.0
        derived = hb_minutes * 60.0 * IDLE_HEARTBEAT_MULTIPLIER
        return max(IDLE_TIMEOUT_FLOOR_S, min(derived, IDLE_TIMEOUT_CEILING_S))
