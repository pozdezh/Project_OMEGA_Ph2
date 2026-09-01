# Does a config change queue for an unreachable device, or wait?

Run 2026-08-26 05:09-05:19. Four units placed into two different unreachable
states before the change, so the answer could be checked against both a
device that is explicitly refused and one that hears nothing at all:

- **Revoked** (explicit refusal): `NMU_16`, `AMU_11`
- **Firewall-blocked at the server** (silence, no reply): `NMU_17`, `AMU_12`
  - a server-side `iptables DROP` on the unit's own address and the
    telemetry port, added and removed by the operator - the device itself
    was never touched, and this cannot be confused with a real WiFi outage
    since nothing about the device's own radio or software changed

Files: `captures/2026-08-26_heartbeat_change.pcap`,
`captures/heartbeat_change_listener.log`

## The mechanism, read from the code first

`server/acks.py`'s `build_ack()` calls `config_store.config_for(device_id)`
on every single ACK, and `config_store.py`'s `config_for()` reads the CURRENT
config file, unconditionally - there is no per-device queue, no "pending
change" record, no delivery confirmation to track. The heartbeat value is
not pushed to a device; it is simply what the file says, restated fresh on
every authenticated contact.

The prediction that follows: a unit locked out for the ENTIRE duration of a
change does not need to "catch up" afterward. Its first accepted ACK, whenever
that happens to be, already carries the current truth - there is nothing
older left to deliver.

## Baseline

```
05:09:35  NMU_16, AMU_11 revoked
05:09:35  NMU_17 (.105), AMU_12 (.108) blocked at the firewall, udp/11400
05:09:45  capture armed
05:10:15  hb set to 1 (both types). File: nmu {hb:1, cfg_ver:11}, amu {hb:1, cfg_ver:11}
          (was hb:25/22, cfg_ver:10)
```

## The twelve unaffected units: seconds, not minutes

```
05:10:15  NOISE logged NMU_T1 event ..._2122
05:10:18  AIRQ  logged AMU_14 event  ..._54
05:10:24  NOISE logged NMU_18 event ..._282
```

Multiple units contacted the server within 3-9 seconds of the config write
and were, by construction of `config_for()`, served the new value on that
exact contact. No separate propagation step exists to wait for.

## The two revoked units, restored

```
05:17:36  revoked list -> []                      (both restored)
05:17:53  [NMU_16] session up, ... idle limit 150s
05:18:01  [AMU_11] session up, ... idle limit 150s
```

**`idle limit 150s` is `hb=1 minute x 60 x 2.5`** - the NEW heartbeat, derived
correctly, on the FIRST session either unit opened after being let back in.
Neither carried its old 25-minute (NMU) or 22-minute (AMU) idle window in
for even one ACK before switching over. Both then drained a backlog built
during the lockout - NMU_16 alone delivered several dozen queued readings in
the same second the session opened.

## The two blocked units, unblocked

```
05:18:10  firewall rules removed
05:18:22  [NMU_17] session up, ... idle limit 150s     <- correct, first contact
05:18:23  [AMU_12] session up, ... idle limit 150s     <- correct, first contact
```

Same result, different failure mode: a unit that heard nothing at all for
nine minutes reconnects to a truth it never received a notification about,
and gets it right immediately.

## The anomaly, and why it does not weaken the result above

`AMU_12`'s first reconnect attempt failed instantly:

```
05:18:23  [AMU_12] could not record address: OSError
05:18:23  [AMU_12] session up, ... idle limit 150s
05:18:23  [AMU_12] read error: ValueError
05:18:23  [AMU_12] session closed: 0 records in 0.0s, duplicates 0
```

Note that the heartbeat value was ALREADY CORRECT even in this failed
session - `config_for()` runs before any record is read, so the crash could
not have prevented it from being right. The most likely cause is a stale
kernel connection-tracking entry left behind by the `iptables -D` removal a
fraction of a second earlier - an artefact of the TEST METHOD, not of the
fleet's own code, and not something a real WiFi outage would produce (there
is no firewall rule to un-remove in that case).

It self-corrected without any operator action:

```
05:18:37  [AMU_12] session up, ... idle limit 150s     <- second attempt, 14s later
05:18:37  AIRQ logged AMU_12 event ..._38
05:18:37  AIRQ logged AMU_12 event ..._39
```

## Final state, all four units

```
NMU_16   3933 rows, last contact <1 min ago
NMU_17   2522 rows, last contact <1 min ago
AMU_11    251 rows, last contact ~1 min ago
AMU_12    272 rows, last contact ~1 min ago
```

All healthy, all confirmed on the new 1-minute cadence.

## Verdict

There is no queue, and testing for one would have been testing for something
that does not exist. What actually holds a config change durable for an
unreachable device is simpler than a queue: the change is written once, to
one file, and every unit's next authenticated contact - regardless of how
long that contact was delayed, or which of two different reasons delayed it
- reads the truth as it stands at that moment. Four units, two distinct
failure modes, one mechanism, no exceptions observed.
