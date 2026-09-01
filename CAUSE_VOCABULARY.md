# Why a unit transmits: the cause vocabulary

The AMU is event-driven. It measures every two seconds and transmits only
when something warrants it, so every stored record carries a `cause` saying
which rule fired. The vocabulary is CLOSED - the server validates each cause
against a fixed grammar and refuses anything else - because a free-text field
written by a device is a field an attacker can write.

## The six reasons, in strict priority

Exactly one reason is reported per record: the highest that applies.

| # | Cause | Fires when |
|---|---|---|
| 1 | `Initial Server Sync` | first record after a session is established |
| 2 | `New Alarm: X` | a threshold was crossed and the unit was not previously in alarm |
| 3 | `All Alarms Cleared` | the last active alarm ended |
| 4 | `Sustained Alarm: X` | an alarm is still active and its own re-announce interval elapsed |
| 5 | `X Spike/Drop`, `X Step/Drift` | a reading moved enough to be worth reporting |
| 6 | `Routine Heartbeat` | nothing happened; the record exists to prove the unit is alive |

## The `(+N)` modifier

When several items qualify at once, the cause names the FIRST and counts the
rest: `Sustained Alarm: High Temp (+1)` means two alarms are active, not that
one alarm repeated. `(+2)` means three.

Listing every combination would make the vocabulary unbounded and therefore
unvalidatable. Naming the leader and counting the remainder keeps it a
closed set the server can check, at the cost of one lookup on the dashboard
to see which others are active.

## Spike/Drop versus Step/Drift

They answer different questions and both are needed.

**Spike/Drop** compares a reading to the ROLLING AVERAGE of its own recent
history. It catches sudden events - a window opening, a door slamming.

**Step/Drift** compares a reading to the value most recently TRANSMITTED. It
catches slow movement that no single sample would flag: a room warming three
degrees over an hour never spikes, but the server's picture of it goes stale.

Spike/Drop asks "did something just happen?". Step/Drift asks "is what the
server believes still true?".

## Why an alarm outranks ordinary change traffic

Measured 2026-08-24 (FINDINGS #51): a latched High Temp alarm went
unmentioned for HOURS. Every unrelated CO2 drift reset the shared
transmission timer, so the newest record - the one the dashboard shows -
always named the drift instead. The alarm was in the data, but invisible in
the only place a person looks.

An active alarm now re-announces on its OWN clock, evaluated before change
traffic. In a care setting an active alarm must not be maskable by unrelated
chatter, however chatty the room.

## Why this matters beyond tidiness

The cause field is what makes the record self-explaining months later. A
reading of 30 C means little; a reading of 30 C stamped `New Alarm: High
Temp` means the unit decided, at that moment, that a person should know. The
vocabulary is the device's reasoning, preserved.

---

# What the NMU's `db` value actually is

Worth stating precisely, because the obvious guess is wrong.

`noise_data.db` is **not** a peak. It is the **arithmetic mean of chunk-wise
dB SPL** across a detected event.

## How an event is measured

The unit samples continuously at low CPU. When the level crosses its trigger
threshold, `audioCaptureTriggeredEvent` runs:

1. Audio is taken in **125 ms chunks**.
2. Each chunk's level is the **RMS of the microphone's AC voltage** - Welford's
   running variance, so the DC bias is removed rather than counted as signal -
   converted with `REF_DB + 20*log10(rms / MIC_SENS)`.
3. The event ends after **4 consecutive chunks (500 ms)** below
   `ambient + 1.5 dB`, or at a hard ceiling of **80 chunks (10 s)**.
4. The stored value is the mean of the chunk levels with the **confirmed
   trailing silence removed**:
   `final_avg = (total_accum - trailing_silence_accum) / active_chunks`
5. `duration_s` is `active_chunks x 125 ms`.

Only the TRAILING run of quiet is subtracted - the accumulator resets on any
loud chunk - so quiet moments *inside* an event stay in the average. A door
slam followed by a pause and a second slam is reported as one event at the
mean of both.

## The honest caveat

This averages **decibels**, which are logarithmic, rather than acoustic
energy. The mean of dB values is not the dB of the mean power: it sits below
the energy-average and **understates brief loud peaks**. A 2-second event
containing one very loud 125 ms moment reports close to the quieter level
that surrounded it.

That is acceptable for the question this system asks - "was there disruptive
noise, roughly how loud, and for how long" - and it is stable and cheap on an
ESP32. But it should be called **mean chunk-wise dB SPL**, never "peak" and
never plain "sound level".

## Why the live MCP query returns something different

`read_now` on an NMU returns `audioAmbientDb()` - the rolling ambient level,
with no event attached and no averaging window. It answers "how noisy is the
room right now", not "how loud was that". The two share a name and a unit and
measure different things, which is precisely why a live read is never written
into `noise_data`.
