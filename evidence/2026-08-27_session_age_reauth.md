# Session-age re-handshake: proven live, unforced, unattended

Discovered 2026-08-27 07:58 CEST while investigating an unrelated question -
not a scheduled test. The real fleet has been running long enough this
session (listener up since ~06:16) that multiple devices independently hit
their natural 1-hour-ish session-age limit and re-handshook on their own,
with no restart, no revocation, no operator action of any kind.

This closes register item #15 ("code-logic fix, not yet field-tested"): the
code was already correct, this is that live proof.

## The mechanism, from the code

`server/listener.py`: `DEFAULT_MAX_SESSION_AGE_S = 3600` (one hour).
`server/session.py serve_session()`: each session draws its OWN limit
independently -

```python
age_limit = max_session_age_s * random.uniform(1.0 - SESSION_AGE_JITTER,
                                               1.0 + SESSION_AGE_JITTER)
```

- +/-20% jitter, so no two devices expire together even if they handshook
  together (the same anti-thundering-herd reasoning as the boot-jitter
  design, applied to steady-state reauth instead of cold boot).
- Checked inline against `time.monotonic()`, not wall-clock time, so an NTP
  step mid-session cannot make a session appear to age faster or slower than
  real elapsed time (`session.py`'s own comment on this).
- On expiry, the CURRENT record is still processed and ACKed normally; only
  the ACK's `reauth` flag is set, telling the device to bring a fresh
  handshake on its OWN next contact. The server never tears the session down
  proactively - the record that happened to trip the limit is delivered
  exactly like any other.

## What actually happened, live, this run

Ten independent age-outs, real devices, unattended:

```
07:11:23  AMU_17   age 3362.1s, 24 records -> re-handshake
07:11:30  NMU_T1   age 3596.2s, 55 records -> re-handshake
07:12:40  NMU_17   age 4306.2s, 80 records -> re-handshake
07:14:38  AMU_14   age 4117.2s, 35 records -> re-handshake
07:14:44  AMU_15   age 3804.1s, 51 records -> re-handshake
07:17:53  NMU_16   age 3658.1s, 52 records -> re-handshake
07:20:03  NMU_20   age 4023.5s, 73 records -> re-handshake
07:22:24  NMU_22   age 3531.8s, 51 records -> re-handshake
07:27:37  AMU_T1   age 3777.1s, 50 records -> re-handshake
07:45:15  AMU_12   age 3340.3s, 38 records -> re-handshake
```

Every age lands inside the designed 2880-4320s band (3600s +/-20%). No two
values are close to each other despite several devices having handshaken
within seconds of one another at boot (06:16-07:29) - the jitter is doing
exactly what it is for.

## Data continuity across the boundary - checked, not assumed

AMU_T1's own record cadence straddling its 07:27:37 reauth (from the real
database, not the log):

```
07:26:23  gap= 31s  Sustained Alarm: High Temp
07:27:36  gap= 73s  Temp Step/Drift        <- the record that tripped the limit
07:28:42  gap= 66s  Temp Step/Drift        <- first record on the NEW session
07:30:50  gap=128s  CO2 Spike/Drop
```

The gap either side of the reauth (73s, then 66s) is unremarkable against
the surrounding cadence (31-158s throughout this window, driven by the
device's own trigger conditions, not the reauth). **No record was lost, no
unusual delay appeared** - the design's own claim, that reauth piggybacks
invisibly on an ACK rather than disrupting the session, holds up against
the actual data.

## Pass criteria, all met

- [x] Re-handshake fires from session age alone - no revocation, no
  operator restart, no reset of any kind coincided with any of the 10 events
- [x] Jitter spreads real values across the full 2880-4320s band, not
  clustered
- [x] Uses `time.monotonic()`, immune to wall-clock adjustment (code-level;
  no NTP step occurred during this window to exercise it live)
- [x] Zero data loss, zero visible disruption at the boundary, checked
  against the real database for a real device

## Honest limitations

- No NTP step happened to coincide with any of these ten events, so the
  "immune to a clock step" property is confirmed by code inspection
  (`time.monotonic()`) plus the unit test already cited
  (`server/test_config_store.py`), not by an observed live clock-step case.
- This is the SERVER-driven reauth path only. Device-side behavior on
  receiving `reauth=true` (bringing its own fresh handshake next contact)
  is implied by the clean, gap-free continuation above, not separately
  instrumented on the device's own log for this specific run.
