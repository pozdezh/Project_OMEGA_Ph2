"""Brick 4 AMU inbound live-query agent - its own PROCESS, deliberately.

Runs the DTLS 1.3 listener in live_server.py so the server can call the AMU
directly and get its most recent reading on demand. This exists because the
AMU must stay event-driven: it transmits ONLY when a trigger fires (a
requirement from the project's sponsors), so on a quiet site there is no
scheduled transmission for an operator question to ride down on. The only
way to reach a silent device is to call it.

WHY A SEPARATE PROCESS, AND NOT A THREAD

The identical listener was previously started as a thread inside main.py.
It segfaulted the deployed unit three times on 2026-08-20 (FINDINGS #32).
Root cause, from wolfSSL's own documentation: one WOLFSSL object must be
used from exactly one thread, and a process running both the telemetry
client and this listener has two of them alive at once. A lock was added,
then a bounded lock, then a lock watchdog - each reduced the crash rate
without removing the cause, because the cause was structural.

Two processes have two address spaces. main.py's wolfSSL state and this
agent's wolfSSL state are not the same memory and cannot interact, so the
rule is satisfied by construction rather than by remembering to hold a
lock. This is the standard remedy for a library that is not thread-safe,
and it is why this file is an entry point rather than another thread.

The two processes share exactly one thing: a JSON file on tmpfs holding the
newest sample (live_cache.py). This agent NEVER touches a sensor - the I2C
and GPIO buses belong to main.py, and a second reader on them was itself a
previous hardware-conflict bug. It answers from the cache, and reports the
sample's age so a stale answer can never look fresh.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import amu_config
import live_cache
import live_server

STALE_KEY = "stale"
AGE_KEY = "age_s"
SAMPLED_AT_KEY = "sampled_at"


def build_read_fn(cache_path, stale_after_s):
    """Return a read_fn for live_server.handle_command that answers from the
    cache written by main.py, annotated with how old the sample actually is.

    Freshness is reported, never assumed. If main.py has died or wedged, its
    last sample stays on tmpfs and would otherwise be served indefinitely as
    though it were current - which is exactly the kind of silent wrong answer
    this project treats as worse than an error."""

    def read_fn():
        payload = live_cache.read_sample(cache_path)
        if payload is None:
            return {"error": "no sample published yet - the sampling loop "
                             "may still be starting up"}
        sampled_at = payload[live_cache.SCHEMA_TS]
        age = max(0.0, time.time() - sampled_at)
        reading = dict(payload[live_cache.SCHEMA_READING])
        reading[SAMPLED_AT_KEY] = sampled_at
        reading[AGE_KEY] = round(age, 1)
        reading[STALE_KEY] = age > stale_after_s
        return reading

    return read_fn


def resolve_port():
    configured = amu_config.LIVE_PORT_CONFIGURED
    if configured is None:
        return amu_config.LIVE_PORT_DEFAULT
    return int(configured)


def main():
    port = resolve_port()
    read_fn = build_read_fn(amu_config.LIVE_CACHE_FILE,
                            amu_config.LIVE_CACHE_STALE_S)
    # serialize=False: this process has exactly one wolfSSL user, so the
    # cross-thread lock guards nothing here - and its watchdog actively kills
    # the agent when a handshake is merely slow (FINDINGS #35).
    server = live_server.LiveCommandServer(
        amu_config.PKI_DIR, amu_config.DEVICE_ID, read_fn, port=port,
        allowed_operators=amu_config.LIVE_ALLOWED_CALLERS, serialize=False)
    print("AMU live agent starting (own process, pid %d) - device %s, "
          "port %d, callers %s, cache %s"
          % (os.getpid(), amu_config.DEVICE_ID, port,
             ",".join(amu_config.LIVE_ALLOWED_CALLERS),
             amu_config.LIVE_CACHE_FILE))
    server.serve_forever()


if __name__ == "__main__":
    main()
