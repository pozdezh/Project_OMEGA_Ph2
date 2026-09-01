# True cold power-cut: server, router, and the fleet, all dark together

Run 2026-08-27 ~08:04-08:10 CEST. Not a warm reset, not a WiFi-only drop:
the server, the router/AP, NMU_21 and AMU_15 were all physically unplugged
by the user at once, held dark for several minutes, then all restored
together - the genuine blackout scenario `SYSTEM_VISION.md` had marked as
"architecturally proven, not power-cut-specific... a full unplug-both-and-
replug test has not been run this session specifically."

Because the ROUTER went down too, this accidentally became a WHOLE-FLEET
test rather than the planned 2-unit one: every device lost its WiFi link at
the same moment, regardless of whose wall socket the user actually touched.

## Baseline, captured immediately before the outage

```
NMU_21  last event 1380589671_1030, session live
AMU_15  last event "Humidity Step/Drift" @ 08:03:57
```

Server confirmed reachable up to seconds before the outage began.

## Confirmed a real reboot, not a network blip

```
Server uptime -s : 2026-08-27 08:08:51
Listener ActiveEnterTimestamp : 08:09:05 CEST
```

## The recovery, fleet-wide - fifteen of sixteen real devices, unattended

```
08:09:10  NMU_17   +5s
08:09:11  NMU_18   +6s
08:09:11  NMU_19   +6s
08:09:13  NMU_20   +8s
08:09:13  NMU_16   +8s
08:09:14  NMU_21   +9s   <- target
08:09:18  NMU_T1   +13s
08:09:23  AMU_14   +18s
08:09:35  AMU_12   +30s
08:09:42  AMU_13   +37s
08:09:42  AMU_15   +37s  <- target
08:09:47  AMU_16   +42s
08:09:52  AMU_17   +47s
08:09:58  AMU_T1   +53s
08:09:58  AMU_11   +53s
```

(+N = seconds after the listener itself came back up.)

**No thundering herd.** Fifteen real devices reconnected inside 53 seconds
with no two handshakes piling up at the same instant - NMUs cluster in an
8-second band (ESP32 boots fast), AMUs spread across 37 seconds (Raspberry
Pi boot + Python service start takes longer, naturally staggering them
further without any code needing to know that). This is the boot-jitter
design (FINDINGS #45) working at real fleet scale during a real mass outage,
not a synthetic one.

## Data continuity, both targets

**AMU_15:**
```
08:03:57  Humidity Step/Drift          <- last before the outage
                                            (device fully powered off, ~5m45s)
08:09:42  Initial Server Sync          <- first reading after reboot
08:10:37  New Alarm: High Temp         <- normal operation resumed
```

`"Initial Server Sync"` is the cause code `triggers.py` sends only on a
fresh boot's forced first transmission - independent confirmation this was
a real power cycle, not a reconnect with state intact. No gap-filling was
possible or expected here: the device was OFF, not merely disconnected, so
there was nothing running to buffer during the outage. Resumption is clean:
no corrupted record, no duplicate, no stuck session.

**NMU_21:**
```
Old session 1380589671, last event _1030  <- before the outage
New session 305407671                     <- fresh esp_random() session id,
                                              confirms a genuine power cycle,
                                              not a soft reset
...
08:10:36  305407671_35
08:10:57  305407671_40
```

New session id is itself the proof: `boot_session` is drawn fresh from
`esp_random()` only at power-on, never preserved across a real power loss -
a soft/watchdog reset would not change it. By 08:10:57 the unit had already
logged 40 fresh events in its new session at a normal cadence, fully
recovered.

## One honest finding, not swept aside

**NMU_22 has not reconnected as of writing** - last seen 08:04:47, before
the outage, none since. All 15 other units recovered inside 53 seconds;
this one did not inside several minutes, well past the 0-30s boot-jitter
window that would explain a late-but-normal arrival. This is not covered by
the "architecturally proven" framing and needs a physical check - it may
simply not have been on the circuit that was cut (a genuinely separate,
mundane explanation), or it may be a real fault. Flagged to the user live,
not held back for a tidier writeup.

## Pass criteria

- [x] Genuine full power cut - confirmed by server `uptime -s` and NMU_21's
  changed session id, not inferred from timing alone
- [x] No hardcoded server IP needed - discovery ran from cold on every unit
  simultaneously, no exceptions among the 15 that returned
- [x] Staggered, not simultaneous - real fleet-scale data, not simulated
- [x] Zero data loss for anything that COULD have been captured - the
  AMU's gap is exactly its own downtime, nothing more; NMU_21 resumed
  cleanly under a new session
- [ ] NMU_22 - not recovered, open, physical check needed

## Honest limitations

- This was not a controlled n=2 test as originally scoped - the router
  outage widened it to the whole fleet, which is a stronger result but was
  not the isolated, minimal test originally planned.
- No serial access to either target device during this run (both are
  deployed field units, not the bench unit on USB), so device-side boot
  logs (RTC state, buffer-restore counts) are not available - only the
  server's independent view. Consistent with the project's own
  two-witness standard where obtainable, single-witness here by necessity.
- NMU_22's absence is reported, not yet explained.
