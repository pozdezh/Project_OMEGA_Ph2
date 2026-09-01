"""Refuses a second, different certificate claiming an existing device name.

See ARCHITECTURE.md 17. Must never know about sockets - callers hand it a
device_id and the peer's DER certificate bytes.
"""

import hashlib
import json
import os
import threading

_lock = threading.Lock()

FIRST_SEEN = "first_seen"
FINGERPRINT = "fingerprint"


def fingerprint(der_bytes):
    if not der_bytes:
        return None
    return hashlib.sha256(der_bytes).hexdigest()


def _load(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def _save(path, doc):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(doc, handle, indent=2, sort_keys=True)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def check(path, device_id, der_bytes, now_ts):
    """Return (allowed, reason). Records the identity on first sight."""
    current = fingerprint(der_bytes)
    if current is None:
        return True, "no certificate bytes available - not checked"
    with _lock:
        doc = _load(path)
        known = doc.get(device_id)
        if known is None:
            doc[device_id] = {FINGERPRINT: current, FIRST_SEEN: int(now_ts)}
            _save(path, doc)
            return True, "first certificate recorded for %s" % device_id
        if known.get(FINGERPRINT) == current:
            return True, "known certificate"
    return False, ("DUPLICATE IDENTITY: %s presented a different certificate "
                   "(%s...) than the one first seen (%s...). Two units are "
                   "provisioned with the same name; their data would merge. "
                   "Re-provision one with an unused name."
                   % (device_id, current[:12], known.get(FINGERPRINT, "")[:12]))


def forget(path, device_id):
    """Allow a deliberate re-provision of this device_id."""
    with _lock:
        doc = _load(path)
        if device_id in doc:
            del doc[device_id]
            _save(path, doc)
            return True
    return False


def main(argv):
    """Re-provisioning CLI. Re-flashing a unit necessarily gives it a NEW
    certificate, which the clone check then refuses until its old fingerprint
    is cleared. Without this, every re-flash strands the unit."""
    if len(argv) >= 4 and argv[1] == "forget":
        if forget(argv[2], argv[3]):
            print("cleared pinned identity for " + argv[3])
            return 0
        print("no pinned identity for " + argv[3] + " (nothing to clear)")
        return 0
    if len(argv) >= 3 and argv[1] == "list":
        for device_id, entry in sorted(_load(argv[2]).items()):
            print("%-10s %s" % (device_id, entry.get("fingerprint", "?")))
        return 0
    print("usage: identity_guard.py list <state.json>")
    print("       identity_guard.py forget <state.json> <device_id>")
    return 2


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv))
