"""Brick 4 AMU entry point - air-quality sampling over a true-DTLS session.

Wiring only: construct the shared queue, start the background network
worker, run the foreground sensor-sampling loop, shut down cleanly. Sensor
detail lives in sensors.py, alarm/heartbeat policy in triggers.py, the
offline store in buffer.py, and the DTLS session lifecycle in network.py -
this file does not know how any of those work internally, only how to call
them in the right order.
"""

import os
import sys
import time
import threading
import queue
import random

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import amu_config
import applog
import sensors
import triggers
import buffer
import network
import live_cache
import clock
import recovery
import retry_schedule

network_queue = queue.Queue(maxsize=amu_config.MAX_QUEUE_DEPTH)
queue_displaced_count = 0

session_id = clock.new_session_id()
event_counter = 0


def enqueue_newest_wins(payload):
    """Newest data always wins: if the hand-off queue is full, the OLDEST
    queued item moves into the offline buffer (not discarded) to make room
    for the new one, and is sent later on the next flush."""
    global queue_displaced_count
    try:
        network_queue.put_nowait(payload)
        return
    except queue.Full:
        pass
    try:
        displaced = network_queue.get_nowait()
        network_queue.task_done()
        buffer.append_to_buffer(displaced)
        queue_displaced_count += 1
    except queue.Empty:
        pass
    try:
        network_queue.put_nowait(payload)
    except queue.Full:
        buffer.append_to_buffer(payload)


def send_data(payload):
    global event_counter
    event_counter += 1
    payload["event"] = "%d_%d" % (session_id, event_counter)
    enqueue_newest_wins(payload)


def flush_backlog():
    """Send buffered records oldest-first. True if the buffer is now empty.

    A record leaves the buffer only when ITS OWN acknowledgement arrives, so
    an interrupted flush loses nothing and simply resumes where it stopped.
    """
    offline = buffer.load_buffer()
    if not offline:
        return True
    if not network.ensure_session():
        return False

    print("Flushing %d buffered packets..." % len(offline))
    flushed = []
    for past in offline:
        time.sleep(random.uniform(amu_config.FLUSH_PACING_MIN_S,
                                  amu_config.FLUSH_PACING_MAX_S))
        if network.deliver(past):
            flushed.append(past.get("event"))
        else:
            network.mark_ack_failure()
            break
    if flushed:
        buffer.remove_delivered(flushed)
        print("Flushed %d/%d buffered packets" % (len(flushed), len(offline)))
    return len(flushed) == len(offline)


# Stamped by the network worker at the top of every cycle and watched by the
# sampling loop. Two threads, one float, no lock: the assignment is atomic
# under the GIL and a stale read costs one extra tick.
_worker_heartbeat = time.time()

# How long the worker may go without starting a cycle before this process
# gives up and lets systemd rebuild it.
#
# A cycle is normally seconds. The generous ceiling covers the one legitimate
# slow case - a full backlog flush over a poor link, where every record spends
# its whole retry budget. Past that it is not slow, it is stuck, and a stuck
# worker is invisible from outside: sampling continues and live queries keep
# answering while nothing is ever delivered.
WORKER_STALL_LIMIT_S = 900.0


def network_worker():
    """Owns the DTLS session. Must never stop running.

    This is a daemon thread with nothing supervising it, and the recovery
    ladder is advanced from inside it. If it dies, the unit keeps sampling
    and queueing forever, sends nothing, and never escalates - a silent
    zombie that only a person walking over and power-cycling it can fix.
    That is the exact failure this system exists to make impossible, so
    every cycle is wrapped: one bad record can cost a record, never the
    thread.
    """
    backlog_retry = retry_schedule.RetrySchedule(
        amu_config.BACKLOG_RETRY_MIN_S, amu_config.BACKLOG_RETRY_MAX_S)

    while True:
        global _worker_heartbeat
        _worker_heartbeat = time.time()
        try:
            if _network_cycle(backlog_retry):
                return
        except Exception as error:
            print("network worker error (continuing): %s: %s"
                  % (type(error).__name__, error), flush=True)
            time.sleep(amu_config.RECOVERY_TICK_S)


def _network_cycle(backlog_retry):
    """One pass of the sender loop. True means shut down."""
    try:
        payload = network_queue.get(timeout=amu_config.RECOVERY_TICK_S)
    except queue.Empty:
        # The ladder must advance on a silent site too, where nothing is
        # queued for long stretches.
        recovery.check(on_radio_reset=network.hard_network_reset,
                       heartbeat_s=amu_config.heartbeat_interval)
        # A buffered alarm must not wait for the next reading to carry it
        # out. On an event-driven unit that reading can be an hour away.
        now = time.time()
        if backlog_retry.due(now) and buffer.load_buffer():
            if flush_backlog():
                backlog_retry.succeeded(now)
            else:
                backlog_retry.failed(now)
        return False
    if payload is None:
        network_queue.task_done()
        return True
    is_hb = payload.get("hb", False)

    # THE LIVE READING GOES FIRST, before any history.
    #
    # It used to go last, behind a full flush of the backlog. That put a
    # fresh alarm behind up to 100 old records, and worse: if the flush
    # failed part-way it dropped the session, and the live record was
    # then buffered WITHOUT EVER BEING SENT. The one reading that might
    # matter was the one guaranteed not to go out. History can wait; the
    # present cannot.
    delivered = network.deliver(payload)
    if not delivered:
        print("Transmission failed / no ACK")
        if not is_hb:
            buffer.append_to_buffer(payload)
        network.mark_ack_failure()
        backlog_retry.failed(time.time())
    else:
        # The link just proved itself, so this is the cheapest possible
        # moment to catch history up.
        if flush_backlog():
            backlog_retry.succeeded(time.time())
        else:
            backlog_retry.failed(time.time())

    recovery.check(on_radio_reset=network.hard_network_reset,
                   heartbeat_s=amu_config.heartbeat_interval)
    network_queue.task_done()
    return False


def main():
    # First, before anything can go wrong and be forgotten. The ladder's last
    # rung reboots this unit, and until 2026-08-27 that erased the log of why
    # - the journal was in RAM. This one is a file in our own directory and
    # needs no privilege, so it survives both the reboot and the fact that
    # this account is deliberately allowed almost nothing as root.
    log_path = applog.start()
    if log_path:
        print("Persistent log: %s" % log_path, flush=True)
    else:
        print("Persistent log unavailable - continuing without it", flush=True)

    network.connect_wifi_initial()

    sensors.init_hardware()

    engine = triggers.TriggerEngine(amu_config.TRIGGER_THRESHOLDS)

    print("Allowing hardware to stabilize 20 sec...")
    time.sleep(20)
    boot_jitter = random.uniform(0.0, amu_config.BOOT_JITTER_S)
    print("Spreading first handshake by %.1fs" % boot_jitter)
    time.sleep(boot_jitter)

    # wolfSSL only initialises on the MAIN thread of this process, so the
    # context is built HERE, before any worker exists. Proved on the live AMU
    # on 2026-08-25: the same SSLContext() call failed with "wolfSSL library
    # initialization failed" on a worker thread and succeeded on the main
    # thread, in one process, seconds apart. Building it costs no socket, so
    # a server that is down at boot cannot stop it.
    if not network.prepare_crypto():
        print("CRITICAL: could not initialise DTLS - restarting so systemd "
              "retries with a clean process", flush=True)
        raise SystemExit(1)

    # THE WORKER IS THE SOLE OWNER OF THE DTLS SESSION, and it is started
    # only here, once, after the context exists and the boot jitter has
    # passed - never alongside a handshake driven from this thread.
    #
    # The crash this closes was seen live on 2026-08-25 during a deliberate
    # server outage: two SIGSEGVs in a row, each one
    # "Fatal Python error: _PyThreadState_Attach: non-NULL old thread state",
    # both immediately after a failed handshake against the unreachable
    # server. The worker thread used to be started BEFORE this line, so it
    # and this thread could both be inside DtlsClient at once. The shared
    # wolfSSL lock guards the handshake call itself, but NOT close(), NOT
    # socket creation, and NOT the assignment of the connection handles - so
    # one thread could free a wolfSSL object while the other was wrapping a
    # socket around it.
    #
    # Serialising it with yet another lock would have been the smaller
    # change and the worse one. One owner needs no lock and cannot race.
    threading.Thread(target=network_worker, daemon=True).start()

    # The inbound live-query listener is NOT started here. It runs as its own
    # process (live_agent.py, omega-amu-live.service) because hosting it as a
    # thread here put two wolfSSL objects in one process, which wolfSSL
    # forbids and which segfaulted this unit three times (FINDINGS #32).
    # This process publishes each sample to a tmpfs file; that agent reads it.

    print("Starting Smart Ageing AirQ (DTLS)...")

    force_initial_payload = True
    try:
        while True:
            time.sleep(2)
            try:
                current_data = sensors.read_all_sensors()
                # Publish for the live agent BEFORE any trigger logic, so an
                # operator calling in always gets the newest sample even when
                # nothing is worth transmitting - which, on a quiet site, is
                # almost always. tmpfs, so this costs no SD-card wear.
                live_cache.write_sample(amu_config.LIVE_CACHE_FILE, current_data)
                trigger, cause, is_heartbeat = engine.evaluate(current_data, force_initial_payload)

                # A question may arrive piggybacked on an ACK - the mechanism
                # proven on the NMU. It is the FALLBACK for operator queries:
                # the server tries live_agent.py's inbound listener first, and
                # drops back to this when that does not answer promptly. This
                # path uses only code that has been running in production, and
                # it cannot be broken by the live agent being down.
                #
                # It answers from the sample already in hand - the AMU never
                # re-reads its sensors on command - and the answer travels on
                # the device's next transmission, exactly like a reading.
                question = network.take_question()
                if question is not None:
                    cmd = str(question.get("cmd", ""))
                    if cmd == "read_now":
                        answer = {"ok": True, "reading": dict(current_data)}
                    elif cmd == "status":
                        answer = {"ok": True, "status": "online",
                                  "buffered": len(buffer.load_buffer())}
                    else:
                        answer = {"ok": False, "error": "unknown command: " + cmd}
                    answer["ts"] = clock.stamp()
                    print("Operator question answered: %s" % cmd)
                    send_data({"id": amu_config.DEVICE_ID, "type": "airq",
                               "ts": clock.stamp(), "hb": False,
                               "cause": "Operator Query", "sensors": current_data,
                               "qr": answer})
                    trigger = False

                # Event-driven ONLY. The AMU transmits when something warrants
                # it and stays silent otherwise - a requirement from the
                # project's sponsors, not a tuning choice. A scheduled
                # keepalive send was tried on 2026-08-20 to give operator
                # questions a ride down; it worked, but it defeats this
                # requirement and was removed. The cost of honouring it is
                # that a silent AMU is unreachable by ACK, which is precisely
                # what the inbound live agent exists to solve.
                if trigger:
                    payload = {"id": amu_config.DEVICE_ID, "type": "airq",
                               "ts": clock.stamp(), "hb": is_heartbeat,
                               "cause": cause, "sensors": current_data}
                    if not payload["ts"]:
                        # The clock is not trustworthy yet. Record how to date
                        # this reading later rather than letting the server
                        # date it by arrival - which would be wrong the moment
                        # the record spends any time in the buffer.
                        payload["_cap"] = clock.capture_marker()
                    print("Data Queued (Cause: %s)" % payload["cause"])
                    send_data(payload)
                    if force_initial_payload:
                        force_initial_payload = False
                # The network worker is watched from HERE, by a different
                # thread, because the failure guarded against is exactly the
                # one that leaves this loop running and that one wedged. On
                # 2026-08-25 an unbounded nmcli call did precisely that: two
                # units sampled and answered live queries for 45 minutes while
                # delivering nothing and never escalating, because the
                # recovery ladder runs inside the stuck thread. Restarting is
                # safe - queued readings are already on disk.
                stalled = time.time() - _worker_heartbeat
                if stalled > WORKER_STALL_LIMIT_S:
                    print("FATAL: network worker has not started a cycle for "
                          "%.0fs - exiting so systemd rebuilds it. Buffered "
                          "readings are on disk and survive." % stalled,
                          flush=True)
                    sys.stdout.flush()
                    os._exit(1)
            except Exception as error:
                print("[%s] Recoverable loop error: %s" % (time.time(), error))
    except KeyboardInterrupt:
        print("Shutting down...")
        network.close()


if __name__ == "__main__":
    main()
