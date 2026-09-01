"""A log of this unit's own account of itself, that survives its own reboot.

The recovery ladder's last rung reboots the box. With the journal in RAM -
which is what these cards were doing, /var/log/journal existing but empty, so
Storage=auto quietly meant volatile - the single most interesting event in the
whole system ERASES ITS OWN EVIDENCE. That happened for real on 2026-08-27:
AMU_15 rebooted itself unaided for the first time ever, and the rung-1 and
rung-2 lines that led up to it were already gone by the time anyone looked.

Making journald persistent is the tidier fix, but it needs root, and a
deployed unit's service account deliberately has almost none - the sudoers
rule grants exactly `reboot` and `nmcli` and nothing else. So the service
keeps its own log instead, in its own directory, needing no privilege at all.

Bounded on purpose. These are SD cards in a building nobody visits: a log that
fills the card turns a diagnostic aid into the outage it was meant to explain.

Nothing here may ever raise. A unit that cannot write its diary must keep
measuring air quality regardless - losing the log is an inconvenience, losing
the sensor is the job.
"""

import os
import sys
import threading

DEFAULT_MAX_BYTES = 4 * 1024 * 1024
DEFAULT_KEEP = 3

_lock = threading.Lock()


def _rotate(path, keep):
    """oldest is discarded, the rest shift down, current becomes .1"""
    try:
        os.remove("%s.%d" % (path, keep))
    except OSError:
        pass
    for index in range(keep - 1, 0, -1):
        try:
            os.replace("%s.%d" % (path, index), "%s.%d" % (path, index + 1))
        except OSError:
            pass
    try:
        os.replace(path, path + ".1")
    except OSError:
        pass


class _Tee(object):
    """Writes to the original stream AND to the file. The original stream
    still goes to journald, so nothing that worked before stops working."""

    def __init__(self, stream, path, max_bytes, keep):
        self._stream = stream
        self._path = path
        self._max_bytes = max_bytes
        self._keep = keep

    def write(self, text):
        try:
            self._stream.write(text)
        except Exception:
            pass
        if not text:
            return
        try:
            with _lock:
                if os.path.exists(self._path) and \
                        os.path.getsize(self._path) >= self._max_bytes:
                    _rotate(self._path, self._keep)
                with open(self._path, "a", encoding="utf-8", errors="replace") as handle:
                    handle.write(text)
        except Exception:
            # A full, read-only or missing card must not take the unit down.
            pass

    def flush(self):
        try:
            self._stream.flush()
        except Exception:
            pass

    def isatty(self):
        return False


def start(path=None, max_bytes=DEFAULT_MAX_BYTES, keep=DEFAULT_KEEP):
    """Begin mirroring stdout and stderr to a file. Returns the path, or None
    if no log could be opened - in which case the unit simply carries on."""
    if path is None:
        path = os.environ.get(
            "OMEGA_APP_LOG",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs",
                         "amu.log"))
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "a", encoding="utf-8"):
            pass
    except Exception:
        return None
    sys.stdout = _Tee(sys.stdout, path, max_bytes, keep)
    sys.stderr = _Tee(sys.stderr, path, max_bytes, keep)
    return path
