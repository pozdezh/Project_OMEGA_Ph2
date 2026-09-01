# NMU_22 recovery ladder, serial-level proof (IN PROGRESS - checkpoint)

Started 2026-08-27 08:29:27 CEST. This file is a checkpoint written mid-run
(session low on context) so the setup and partial data are not lost. If this
session ends before the ladder completes, a future session can finish it
from here: read the log tail, compare against the predicted times below.

## Why this run exists

Register #10 (recovery ladder) is marked DONE but explicitly notes a gap:
"intermediate rungs not evidenced this run - the unit's journal does not
survive its own reboot." This run fixes that by watching NMU_22's live
serial output on USB through both an intended radio-reset AND reboot rung,
which DOES survive across its own reboot (Windows re-detects the port).

Also closes the loop on the earlier NMU_22 mystery this session: it was
found silent, alive on the network, discovery-succeeded-but-no-handshake.
Root cause established from code, not guessed: the recovery ladder's rungs
are heartbeat-derived (NMU heartbeat = 5 min, live `device_config.json`),
giving radio-reset at 7.5 min and reboot at 17.5 min silence - NMU_22 was
simply mid-ladder when it was unplugged for transport (~2.5 min before its
own scheduled reboot), not stuck. This run proves that ladder actually fires
end to end.

## Setup

- **Subject:** NMU_22, MAC `f0:9e:9e:78:53:28` (confirmed via `esptool
  read-mac` directly from the chip, not inferred from a possibly-stale ARP
  entry - IPs had shuffled after the earlier power-cut test).
- **Method:** MAC-blocked at the router by the user (same method as
  register #13's silent-block test). WiFi association itself fails
  (`WiFi: status=4`, WL_CONNECT_FAILED) - harsher than a server-side drop,
  and matches how register #13 tested it.
- **Watching:** NMU_22 connected via USB to this PC (brought here earlier
  this session after the power-cut test). A PowerShell logger
  (`scratchpad/ladder_watch.ps1`) re-detects the serial port by USB VID
  every reconnect (ports renumber across a reset - see nmu-flash-workflow
  skill) and appends every line to `scratchpad/nmu22_ladder.log`, so the
  boot banner across the reboot rung is captured live, not lost.

## Predicted rung times (from code, not assumption)

`server/device_config.json` live config: NMU heartbeat = 5 min.
`nmu/omega_config.h` / `omega_tasks.cpp outageRung()`:

```
radioAt = max(OUTAGE_RADIO_RESET_MS, heartbeat/10 * 15)  = max(300000, 450000) = 450000ms = 7.5 min
rebootAt = max(OUTAGE_REBOOT_MS,     heartbeat/10 * 35)  = max(900000, 1050000) = 1050000ms = 17.5 min
```

Block applied 08:29:27, last real server contact 08:29:00 (DB-confirmed,
event `586188209_6`).

```
Radio reset rung due  ~08:36:30 (silent > 450s)
Reboot rung due       ~08:46:30 (silent > 1050s)
```

## Data captured so far (checkpoint, run still in progress)

Block took effect immediately and cleanly:

```
08:30:08.602  WiFi: status=4 ip=0.0.0.0 rssi=0 bcast=255.255.255.255
08:30:08.602  Net: WiFi connect failed.
08:30:08.623  Net: no session, buffering event=11
```

Buffer growth curve, offline events accumulating exactly as designed
(RAM ring buffer, per CLAUDE.md's architecture notes - no NVS wear while
this is a fresh outage, not yet at the brownout/mirror threshold):

```
silent= 62s  buffered= 7  qdisplaced=5
silent=119s  buffered=15  qdisplaced=9
silent=215s  buffered=26  qdisplaced=14
silent=246s  buffered=29  qdisplaced=15
```

`qdisplaced` growing alongside `buffered` is expected and correct: the
20-deep send queue is full during a real outage, so newest-wins displacement
(`enqueueEventNewestWins`, `main.py`'s AMU equivalent already evidenced) is
doing its job - nothing is being silently dropped, it is being moved from
queue to the RAM ring buffer.

**Radio-reset rung: CONFIRMED, 08:36:40, silent=459s** (predicted ~450s -
match). Log line: `"Net: no server contact for 459s - full radio reset"`.
**Reboot rung: CONFIRMED, 08:46:42, silent=1061s** (predicted ~1050s -
match). `"Net: no server contact for 1061s after 1 radio reset(s) -
rebooting (buffer is mirrored to flash)"`. Boot banner captured clean:

```
BOOT: reset_reason=3 (1=power-on 4=panic 6=task-wdt 9=brownout 11=usb)
POSTMORTEM: previous boot ended reason=1 after 1097s
BOOT: session=586188209 resuming at event=150
BOOT: spreading first handshake by 27618 ms
```

Minor doc gap noted, not a bug: `reset_reason=3` (ESP_RST_SW, from
`ESP.restart()` itself) is not listed in that comment's enum
(1/4/6/9/11) - worth adding.

Session id and event counter SURVIVED the self-reboot (resumed at 150, not
reset to a fresh session). The mechanism is RTC retained memory, not the flash
mirror: `bootSession` and `eventCounter` are declared `RTC_NOINIT_ATTR` in
`nmu/omega_tasks.cpp`, so they survive a software reset and are cleared only by
a loss of power. The flash mirror preserves the buffered records, each carrying
its own identity; the two mechanisms are separate and an earlier version of
this note wrongly credited the mirror with both. Boot-jitter spread (27.6s) applied even on this
self-triggered reboot, exactly as designed.

## Unplanned finding: lifting the MAC block bounced the whole fleet

User observed other NMUs transmitting the moment the block was lifted, not
just NMU_22. Confirmed from the server log: **7 other NMUs (21, 19, 20, 18,
17, T1, 16) all re-handshook within the same 9-second window** (08:48:01-
08:48:10) as NMU_22's own reconnect. Root cause: editing a router's MAC
filter/ACL list commonly bounces the whole WiFi radio while the new table
applies - not a per-device action. Not a bug in this system; a property of
the test method itself, worth noting for anyone repeating this test.

One handshake failure followed 50s later (`SSLError: do_handshake failed
with error -308`), traced to NMU_22 itself mid buffer-drain - a transient
collision from the mass-reassociation chaos. Self-recovered within 1s via
its own retry; the resulting duplicate event was correctly caught by the
DB's unique `(device,event)` index (`DUPLICATE noise 586188209_136`), never
stored twice. Two resilience mechanisms (retry-on-failure, dedup-on-retry)
proven stacked under an unplanned secondary disruption layered on the
planned ladder test.

**Why the graph showed all NMUs at once:** each carries an equivalent to
the AMU's "Initial Server Sync" - `SentryTask` (`omega_tasks.cpp`) forces
one real measurement the instant `serverFound` flips back to true:
`"Sentry: initial sync measurement queued, db=59.23, event=162"` at
08:48:10, 2s after NMU_22's own handshake came up. Real asymmetry worth
noting: AMU tags this with `"cause": "Initial Server Sync"`; the NMU's
noise-record JSON has NO cause field at all, so this forced sync event is
indistinguishable from an ordinary detected-noise event once stored -
only the serial log marks it as the reconnect sync, not the DB/dashboard.

NMU_22's own drain: `buffered` peaked at 93, falling to 51 within ~30s,
with repeated `"DTLS: recovered on attempt 2, session kept"` - per-record
retry (not a full re-handshake) absorbing transient send failures during
the mass-reconnect chaos, same resilience layer as the -308 failure above.

## Sync-packet duration: constant by design, not silence-influenced

Checked whether the "initial sync" packet's `duration` field is a real
measurement that silence could affect, per a direct question this session.

`nmu/omega_audio.cpp audioCaptureSyncMeasurement()` has NO silence-detection
loop at all - unconditionally runs `SYNC_MEASUREMENT_CHUNKS` (8) chunks,
then sets `result.duration_s = 1.0f`, a hardcoded constant. The value
happens to equal the nominal chunk math (8 chunks x (1000 samples /
8000 Hz) = 1.0s) but is not derived from any timer - nothing measures
elapsed wall-clock time in this path.

**Verified against the real stored record, not just the code:**

```
Serial: "Sentry: initial sync measurement queued, db=59.23, event=162"
DB:     ('586188209_162', db=59.23, duration=1.0)
```

Exact match on both fields, confirming the code read is the code that ran.

**Contrast confirms this is deliberate, not a bug.** The OTHER capture path,
`audioCaptureTriggeredEvent()` (ordinary noise detection), genuinely does
stop early on silence (`while (silence_count < MAX_SILENCE && ...)`) and
shows real variance already captured this session: `0.12s`, `4.12s`,
`0.25s`, `0.50s`. The sync path is architecturally exempt from that
by design - a fixed-length snapshot, not an event-triggered capture.

**Conclusion:** the sync packet's duration cannot be influenced by silence
or environmental conditions, because the function that produces it has no
code path for that to happen. It will always read 1.0.

## Why NMUs reconnected but AMUs never noticed

Checked whether AMUs also showed a sync burst when the block was lifted -
confirmed **zero** `[AMU_*] session up` lines in the whole window; AMUs
kept transmitting (`AIRQ logged...`) continuously straight through it.

Root cause, code-grounded on both sides:

- **NMU** (`omega_net.cpp ensureSession()`) polls `WiFi.status()` every
  `NetworkTask` loop iteration. The instant the router's ACL reload briefly
  de-authed it, the ESP32 driver reported disconnected, the code tore the
  session down immediately, and ran a full discovery+handshake the moment
  WiFi returned - hence the visible sync-packet burst.
- **AMU** (`dtls_client.py connected()`) has NO link-state check at all -
  purely `time.time() - self._last_exchange <= self._stale_after_s`
  (~480s, from the server's own ACK). As long as the blip was shorter than
  that, the session object never considered itself disconnected; Linux's
  own network stack (NetworkManager/wpa_supplicant) reassociated
  underneath it before the app layer had any reason to care.

Neither is a bug: NMU's aggressive polling is necessary on bare-metal
ESP32 with no OS-level roaming; AMU's time-based tolerance is appropriate
given Linux already handles reassociation transparently. Real tradeoff
worth flagging for the memo: an AMU WiFi blip can pass with zero log
evidence, unlike the NMU's loud, visible reconnect.

Block removal and full drain confirmation were NOT captured in this run: the
serial log ends with `buffered` at 51 and still falling. The drain completing
is evidenced separately, on a different run, in
`2026-08-21_nmu_outage_restore_cycle.md` (91 records restored from flash and
delivered in 6 s, zero duplicates).

## What to check when resuming

1. `Get-Content scratchpad\nmu22_ladder.log | Select-String "radio reset|rebooting|BOOT:|POSTMORTEM"`
2. Confirm radio-reset fired near silent=450s (~08:36-37) - log line:
   `"Net: no server contact for Ns - full radio reset"`
3. Confirm reboot fired near silent=1050s (~08:46-47) - log line:
   `"Net: no server contact for Ns after N radio reset(s) - rebooting
   (buffer is mirrored to flash)"`, followed by a fresh `BOOT: reset_reason=`
   line (expect reason=1, power-on-equivalent restart via `ESP.restart()`
   actually reports differently - confirm the actual value logged, do not
   assume) and a `POSTMORTEM:` line reporting on THIS run's outcome.
4. Remove the router MAC block.
5. Confirm reconnect: fresh `WiFi: status=3`, `Net: DTLS session ... UP`,
   and check the buffer drains (`drainBacklog()` - watch `buffered` fall
   back toward 0 with no gap in event numbering).
6. Cross-check the server side: `journalctl -u omega-listener | grep NMU_22`
   for the post-reboot session and confirm the DB receives the drained
   backlog with original timestamps (same check style as the AMU_15/NMU_18
   revocation-recovery evidence already on file).
7. Update register #10 to remove the "intermediate rungs not evidenced" gap
   note, citing this file.

## Honest note on this checkpoint

Written mid-run because the session was low on context, not because the
test finished. Sections above marked "predicted" or "not yet reached" are
exactly that - do not upgrade them to confirmed results without checking
the actual log lines per the resume steps.
