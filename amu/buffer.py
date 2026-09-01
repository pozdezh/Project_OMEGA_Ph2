import json
import os
import threading

import amu_config

_lock = threading.RLock()


def _read_unlocked():
    if not os.path.exists(amu_config.BUFFER_FILE):
        return []
    try:
        with open(amu_config.BUFFER_FILE, "r") as handle:
            return json.load(handle)
    except Exception:
        return []


def _write_unlocked(buffer_list):
    tmp = amu_config.BUFFER_FILE + ".tmp"
    try:
        with open(tmp, "w") as handle:
            json.dump(buffer_list, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, amu_config.BUFFER_FILE)
    except Exception as error:
        print("Failed to write buffer: %s" % error)
        try:
            os.unlink(tmp)
        except OSError:
            pass


def load_buffer():
    with _lock:
        return _read_unlocked()


def save_buffer(buffer_list):
    with _lock:
        _write_unlocked(buffer_list)


def append_to_buffer(payload):
    if payload.get("hb", False):
        return
    with _lock:
        buf = _read_unlocked()
        buf.append(payload)
        while len(buf) > amu_config.MAX_BUFFER_SIZE:
            buf.pop(0)
        _write_unlocked(buf)


def remove_delivered(event_ids):
    delivered = set(event_ids)
    if not delivered:
        return 0
    with _lock:
        buf = _read_unlocked()
        kept = [record for record in buf if record.get("event") not in delivered]
        removed = len(buf) - len(kept)
        if removed:
            _write_unlocked(kept)
        return removed
