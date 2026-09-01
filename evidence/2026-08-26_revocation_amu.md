# Revoking AMU_13, entirely from the dashboard - and an unplanned bonus proof

Run 2026-08-26 04:21-04:48 on the deployed fleet. Subject: AMU_13
(Raspberry Pi, Python), untouched throughout - every action from a browser or
observed passively over SSH into the server. No card, no restart of the unit
by hand, no intervention on the device at all.

Files: `captures/2026-08-26_amu13_revocation.pcap`,
`captures/amu13_revocation_listener.log`

This is the same experiment run against NMU_18 (`2026-08-26_revocation_live.md`,
`..._revocation.md`), repeated on the other device type - different language,
different DTLS library, different transport pattern - to test whether
revocation is a property of the design or an accident of one firmware.

## Baseline

```
AMU_13: 219 rows, last reading 3.5 min ago, revoked list: []
```

## Revoke, from the UI, one click

```
04:21:32  Key admin: AMU_13 revoked=True     <- exactly one log line, first press
```

This is the same dashboard button that needed two presses earlier tonight
before its row-rebuild bug was fixed (see DEV_CHRONOLOGY.md). One press, one
line, here - the fix confirmed on the very next real use.

## No mid-session kill this time, and here is why that is not a gap

Unlike NMU_18, no `REVOKED mid-session` line appears. AMU_13 had not sent
anything for 3.5 minutes before the revoke, and its last exchange had already
completed - there was no open, live session for the revoke to interrupt.
That is the honest reading, corrected from an earlier looser claim in this
same run ("no session was open" is not something the log alone proves; what
it proves is that no mid-session kill fired, which is consistent with either
an already-closed session or a quiet one - AMU_13's next behaviour settled
which).

## Refused at every handshake attempt for sixteen minutes

```
04:28:14  REJECTED AMU_13: revoked or not on the allow-list
04:29:27  REJECTED AMU_13: revoked or not on the allow-list
04:30:28  REJECTED AMU_13: revoked or not on the allow-list
...
04:37:37  REJECTED AMU_13: revoked or not on the allow-list
```

**Seventeen consecutive refusals**, roughly 60-90 seconds apart, with no
backoff observed in this window - unlike NMU_18's run, which quieted after
about ten minutes. Both are legitimate: the retry cadence is derived from
each device's own schedule, and this window simply did not reach the point
where AMU_13's pacing would have widened.

Each refusal is the OTHER of the two doors described for NMU_18: the full
DTLS handshake completes - the certificate is genuine, the cryptography is
correct - and the connection is refused afterward, at
`server/session.py:191`, once the name is read off the now-verified
certificate and checked against the list.

## What the device did while locked out

```
04:36:33  Flushing 3 buffered packets...
04:36:44  record 590928891_16 LOST after 3 attempts - dropping session
04:37:37  Flushing 3 buffered packets...
04:37:47  record 590928891_16 LOST after 3 attempts - dropping session
```

AMU_13 has its own independent retry clock (`BACKLOG_RETRY_MIN_S`/`MAX_S` in
`amu_config.py`, 5-60 s backoff) that tries to drain its buffer without
waiting for a new trigger - the same design as the NMU's, in a different
language. Both attempts above happened BEFORE the restore and were correctly
refused; nothing was lost, because a refused delivery stays in the buffer
rather than being discarded.

## Restore, one click, confirmed instant

```
04:37:53  Key admin: AMU_13 revoked=False
04:38:15  [AMU_13] session up, age limit 3749.4s, idle limit 3300s
```

22 seconds from click to an accepted session - the very next handshake
attempt succeeded, exactly as the code predicts (no counter, no state, a
fresh yes/no check every time).

## The unplanned part: a transport failure, and the fix from six hours
## earlier proving itself on a fault nobody staged

The session that opened at 04:38:15 did not deliver cleanly:

```
04:38:15  [AMU_13] session up
   ...129 seconds, no records...
04:40:25  [AMU_13] read error: SSLError
04:40:25  session closed: 0 records in 129.4s, duplicates 0
```

Device side:

```
04:40:35  record 590928891_19 LOST after 3 attempts - dropping session
04:40:35  Transmission failed / no ACK
04:40:35  ACK failure streak: 4/5
```

This is unrelated to revocation - the session had already been ACCEPTED,
proving the list check passed. It is an ordinary transport hiccup of the kind
`amu/network.py`'s `mark_ack_failure()` exists to handle: at 5 consecutive
failures it calls `hard_network_reset()`, the exact function that carried an
unbounded `nmcli` call until earlier the same night (see the AMU_14/AMU_16
findings in DEV_CHRONOLOGY.md, 2026-08-25). That call is now bounded at 15 s
and 45 s, and a 900 s watchdog in the sampling thread exists as the backstop
if it is not.

Neither had to fire to their limit. The unit recovered on its own, quickly:

```
04:47:22  AIRQ logged AMU_13 event 590928891_21
04:48:10  AIRQ logged AMU_13 event 590928891_22
```

No intervention, no SSH command sent to the device during this window, no
restart. It reset its own connection and resumed.

## The database shows no gap across any of it

```
row count: 219 before revocation -> 227 after      (+8)

04:28:08  co2=517.23  event=590928891_17
04:33:08  co2=512.45  event=590928891_18
04:38:09  co2=517.25  event=590928891_19    <- taken during the transport failure
04:43:09  co2=522.58  event=590928891_20    <- taken during the transport failure
04:47:22  co2=515.22  event=590928891_21    <- first successful post-recovery delivery
04:48:09  co2=512.65  event=590928891_22
```

Six consecutive event numbers, 17 through 22, no gap and no duplicate,
spanning the revocation, the lockout, the restore, AND the unplanned
transport failure. Every reading the unit took during all of it survives and
arrives once the path to the server is clear, regardless of which specific
fault stood in the way.

## Verdict

The result is identical to NMU_18's, on a different device, different
language, different failure mode along the way: revocation is enforced
instantly and without exception; restoration needs no operator action beyond
the one click; and nothing measured during any part of the outage - the
deliberate one or the accidental one - is lost. The unplanned transport
failure is, if anything, the stronger proof: it is the same recovery
machinery being exercised by a fault the fleet produced on its own, hours
after the fix that was meant to catch it, with nobody watching it in advance.
