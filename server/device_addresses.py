"""Where each device was last seen, learned from the device itself.

A live query has to know an address to call. Keeping that in a hand-written
"endpoints" map meant every new unit needed a config edit before it could be
reached - and a DHCP change silently broke the entry, because nothing
re-checked it.

Nothing needs to be typed. Every device dials the SERVER for telemetry, so at
the moment its handshake completes the listener holds both facts at once: the
identity proved by the certificate, and the source address the datagram came
from. This records that pairing.

It is the mirror image of what each device already does with the server's
address (see amu/server_discovery.py and nmu/omega_discovery.cpp): learn it
by being talked to, remember the last one that worked, and let it be
overwritten the moment a newer one authenticates.

SAFETY. An address recorded here is NOT a credential and grants nothing. It
is only ever used as somewhere to place an outbound call, and that call is
itself a mutual-authentication DTLS handshake - so a wrong or stale address
produces a failed handshake, never a wrong device answering. The address is
recorded only AFTER the certificate has been verified, so a stranger cannot
poison another unit's entry by sending a datagram.
"""

import json
import os
import threading
import time

_lock = threading.Lock()

ADDRESS = "address"
LAST_SEEN = "last_seen"


def _load(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def _save(path, doc):
    """Temp file plus rename, so a crash cannot leave a half-written map."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(doc, handle, indent=2, sort_keys=True)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def remember(path, device_id, address, now_ts=None):
    """Record where an AUTHENTICATED device just called from.

    Returns True if this changed what was stored, so the caller can log a
    move rather than every routine reconnection.
    """
    if not path or not device_id or not address:
        return False
    with _lock:
        doc = _load(path)
        previous = doc.get(device_id, {}).get(ADDRESS)
        doc[device_id] = {
            ADDRESS: address,
            LAST_SEEN: int(now_ts if now_ts is not None else time.time()),
        }
        # Writing on every reconnection would be pointless flash traffic on a
        # fleet that re-handshakes hourly; write only when something changed.
        if previous != address:
            _save(path, doc)
            return True
        return False


def lookup(path, device_id):
    """The address this device last authenticated from, or None."""
    if not path or not device_id:
        return None
    entry = _load(path).get(device_id)
    if not isinstance(entry, dict):
        return None
    return entry.get(ADDRESS) or None


def all_known(path):
    """Every device with a learned address: {device_id: {address, last_seen}}."""
    doc = _load(path)
    return {k: v for k, v in doc.items() if isinstance(v, dict) and v.get(ADDRESS)}


def forget(path, device_id):
    """Drop one device's learned address, so the next call re-learns it."""
    with _lock:
        doc = _load(path)
        if device_id in doc:
            del doc[device_id]
            _save(path, doc)
            return True
        return False
