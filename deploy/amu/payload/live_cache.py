"""Latest-sample cache shared between the AMU's two processes.

main.py owns the sensors and WRITES. live_agent.py owns the inbound DTLS
listener and READS. They never share memory, and that is the entire point:
running the listener inside main.py meant two wolfSSL objects in one
process, which wolfSSL forbids and which segfaulted this unit three times
on 2026-08-20 (FINDINGS #32). Two processes have separate address spaces,
so the library state that crashed cannot be shared by construction - not by
a lock that has to be remembered and got right.

A plain file on tmpfs is the whole IPC mechanism. No socket, no shared
memory segment, no lock: the reader can never block the sampling loop, and
there is no shared state to corrupt.

Writes are atomic - a temp file in the same directory, then os.replace(),
which POSIX guarantees is atomic within one filesystem. A reader therefore
always sees a complete sample, either the previous one or the new one,
never a half-written mixture.
"""

import json
import os
import tempfile
import time

SCHEMA_TS = "ts"
SCHEMA_READING = "reading"


def write_sample(path, reading, ts=None):
    """Atomically publish the newest reading. Never raises: a cache write
    failing must not take down the sampling loop, which has a real job."""
    payload = {SCHEMA_TS: int(ts if ts is not None else time.time()),
               SCHEMA_READING: reading}
    directory = os.path.dirname(os.path.abspath(path)) or "."
    handle = None
    temp_path = None
    try:
        fd, temp_path = tempfile.mkstemp(dir=directory, prefix=".omega_live_")
        handle = os.fdopen(fd, "w")
        json.dump(payload, handle)
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        handle = None
        os.replace(temp_path, path)
        return True
    except Exception as error:
        print("live cache: write failed: %s" % error)
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass
        if temp_path is not None and os.path.exists(temp_path):
            try:
                os.unlink(temp_path)
            except Exception:
                pass
        return False


def read_sample(path):
    """Return {"ts": int, "reading": dict} or None if there is no usable
    sample yet. None is a normal state, not an error: the agent can start
    before the sampling loop has published anything."""
    try:
        with open(path, "r") as handle:
            payload = json.load(handle)
    except (IOError, OSError, ValueError):
        return None
    if not isinstance(payload, dict):
        return None
    if SCHEMA_TS not in payload or SCHEMA_READING not in payload:
        return None
    return payload
