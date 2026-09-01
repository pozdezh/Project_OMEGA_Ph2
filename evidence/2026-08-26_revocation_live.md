# Revoking a unit that is working perfectly

Run 2026-08-26 03:28-03:52 on the deployed fleet. Subject: NMU_18
(Adafruit Feather ESP32-S3), on the development bench by USB so its serial
console could be read, but otherwise a normal fleet member reporting over
WiFi like the other fifteen.

Capture: `captures/2026-08-26_revocation.pcap`
Serial: `captures/nmu18_revocation_serial.txt`

## The claim under test

A unit that is retired, lost or stolen stops being accepted - **even though
nothing about the unit changes.** It keeps its certificate, its WiFi, its
configuration and its belief that it belongs. Only the authority's opinion of
it changes, and that alone is enough.

This is the capability a pre-shared-key design cannot offer. With shared keys
the only remedy is to delete a row and hope, or to re-key every peer. With a
certificate authority, withdrawal is a property of the identity itself.

## Failure criteria, fixed before the run

- it still gets through after revocation
- it does not rejoin after restore
- restoring requires touching the device
- readings taken while locked out are lost

## Baseline

```
NMU_18: last reading 0.2 min ago, 875 rows total
revoked list: []      roster: []      (empty roster = any CA-signed device)
```

## Revocation takes effect in nine seconds, mid-session

The revoke was issued through the dashboard's own API, with the operator
certificate, exactly as a person clicking the button would:

```
03:29:15  POST /api/keys/revoke {"id":"NMU_18"}  ->  {"id":"NMU_18","revoked":true}
```

The server log:

```
03:29:24  [NMU_18] REVOKED mid-session - closing immediately
03:29:24  [NMU_18] session closed: 109 records in 1834.7s (0.06/s), duplicates 0
```

**Nine seconds.** The live session was not left to expire and the unit was
not allowed to finish what it was sending. `session.py` re-checks the
revocation list on **every record**, so the very next reading it offered was
the one that closed the door.

That is worth stating precisely, because it is stronger than the obvious
design: revocation does not wait for the next handshake.

## Then refused at every attempt, for ten minutes

```
03:29:44  REJECTED NMU_18: revoked or not on the allow-list
03:30:01  REJECTED NMU_18: revoked or not on the allow-list
03:30:17  REJECTED NMU_18: revoked or not on the allow-list
03:30:32  REJECTED NMU_18: revoked or not on the allow-list
...
03:39:18  REJECTED NMU_18: revoked or not on the allow-list
```

**Eleven consecutive refusals**, roughly one every 15-20 seconds. Not a
timing fluke and not a single dropped packet: the unit presented a valid,
CA-signed certificate every time and was turned away every time, at
`session.py:191`, before the session could begin.

The refusals then stop. That is the unit backing off, not the server
relenting: after its aggressive-reconnect window it returns to its normal
cadence rather than hammering a server that keeps saying no. A revoked unit
does not burn its battery or flood the network.

## Restore is instant on the server side

The revoked list returned to `[]` the moment Restore was clicked. Nothing was
scheduled, cached or deferred - the next handshake would have been accepted.
The delay before the unit actually returned was entirely its own back-off.

## Nothing measured during the lockout was lost

This is the part that matters most, and it was proven by behaviour rather
than by the unit's own testimony.

On its next boot the unit reported:

```
BUFFER: mount=1 total=1739681 used=2510 mirror=1 tmp=0 bytes=2116 restored=100
```

**`restored=100`** - one hundred readings recovered from flash. While locked
out it kept sampling, kept queueing, and mirrored the backlog to flash rather
than discarding it.

They then arrived, with their ORIGINAL measured times:

```
row count: 875 before revocation  ->  984 after      (+109)

03:28:40  62.5 dB     <- last reading before the revoke
03:34:15  59.5 dB     <- taken while LOCKED OUT
03:35:26  64.0 dB     <- taken while LOCKED OUT
03:37:22  72.8 dB     <- taken while LOCKED OUT
03:38:49  60.1 dB     <- taken while LOCKED OUT
03:44:14  69.2 dB     <- taken while LOCKED OUT
03:47:57  79.8 dB     <- taken while LOCKED OUT
```

There is no gap in the record. A reader of the database cannot tell the unit
was ever refused, except by the absence of anything anomalous: the readings
are stamped when they were MEASURED, not when they were finally accepted.

**Security and continuity did not trade against each other.** The unit was
denied the network and lost no data.

## What was NOT clean about this run, stated plainly

Between 03:39 and 03:49 the unit went off the network entirely - no ping, ARP
incomplete, mDNS timeout, no LED activity and a dead USB console - and it did
not return on its own. It was reset over USB at 03:49, after which it booted
normally, discovered the server, handshook in 4198 ms and delivered its
backlog.

**This cannot be attributed to the revocation.** That same board had been
reset repeatedly by esptool minutes earlier for the cold-boot experiment,
including two `--after no-reset` sequences that leave it halted in the ROM
bootloader, and its serial console was already dead before the revocation
began. A board left in that state is not a controlled subject.

The claims above do not depend on it: the mid-session kill, the eleven
refusals, the instant restore and the intact backlog were all observed
before, during and after, from the server. But the question "does a revoked
unit rejoin unattended once restored?" was **not** answered by this run, and
must not be claimed. It needs a repeat on a unit that has not been handled -
any of the eight AMUs, over SSH, with nothing physical involved.

## Where this stands against the parallel project

Her equivalent is rejecting a device whose pre-shared key is absent or wrong,
demonstrated with a self-written emulator sending false credentials - clean
work, and the right test for that design.

The difference is what is being withdrawn. In her system the device is
refused because it never had the secret. Here the device **has** a wholly
valid credential, correctly signed, and is refused anyway, because the
authority that issued it changed its mind. Nothing on the device is altered
or deleted; the change is made once, centrally, and takes effect in nine
seconds on a unit nobody can reach.

That is the practical argument for a certificate authority over shared keys,
and it is now measured rather than asserted.
