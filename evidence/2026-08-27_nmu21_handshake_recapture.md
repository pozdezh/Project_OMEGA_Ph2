# NMU_21 handshake, recaptured independently on a second unit

Run 2026-08-27 06:15 local (04:15 UTC). Subject: NMU_21 (Adafruit Feather
ESP32-S3, deployed unit, already provisioned - no NVS erase this time, this
is a live "just reboot it" trigger, not a cold-boot-from-empty test).

Purpose: reproduce the plaintext-vs-encrypted record breakdown from
`2026-08-26_boot_discovery_handshake.md` (NMU_18) on a second, independent
unit, to check whether the earlier "6 plaintext records" result was specific
to that one capture or a repeatable pattern.

Two independent witnesses, same as before:

## 1. What was done, exactly (reproducible)

Capture started on the server FIRST, by the user directly over their own
SSH session (not by Claude - avoids putting the server's sudo password into
any command Claude runs):

```
ssh omega-server
sudo tcpdump -i any -w /tmp/nmu21_recapture_20260827.pcap udp port 11400 or udp port 5001 or udp port 5353
```

Reset triggered from this PC once the capture was confirmed running:

```
esptool --chip esp32s3 --port COM20 --before default-reset --after watchdog-reset read-mac
```

That command enters the ROM bootloader (confirmed: COM20/VID_239A app port
disappeared, COM21/VID_303A bootloader port appeared) but its own final
watchdog-reset step reported a cosmetic `serial exception` from the port
re-enumerating mid-verify - a SECOND command was needed to actually exit
bootloader and resume the app:

```
esptool --chip esp32s3 --port COM21 --before no-reset --after watchdog-reset read-mac
```

This two-step pattern (reset-into-bootloader, then a follow-up to boot back
out) is the same cosmetic-exception behaviour documented in FINDINGS #17 for
the flashing path - it is not specific to this test.

Capture stopped by the user (`Ctrl+C`) a few seconds after the device's
serial log showed it had reconnected. Pulled to this PC and committed here:

```
scp omega-server:/tmp/nmu21_recapture_20260827.pcap evidence/captures/2026-08-27_nmu21_recapture.pcap
```

Parsed with a small standalone script (no Wireshark/tshark dependency,
written because tshark is not installed on this PC - only the Wireshark GUI
is) that reads the raw pcap, reassembles UDP payloads, and classifies each
DTLS record purely by its header bytes: `0xfefd` in bytes 1-2 = legacy
plaintext record header (readable), else a byte with the top three bits
`001` = the DTLS 1.3 unified header (opaque). No decryption of anything -
this only reads what is already sitting in cleartext on the wire.

## 2. Device identity - correlated, not asserted

The pcap has no device-name field (that is the point - identity is inside
the encrypted certificate). Identity is established by correlation:
NMU_21's own serial log timestamped the reboot at 06:15 local (04:15 UTC,
`uptime=19s` when read 19+ seconds after this capture's handshake), and
exactly one new handshake (206-byte ClientHello from a previously-silent
source IP) appears in the capture within that same second, from
`192.168.0.103`. No other device rebooted in this window.

**Serial witness (second read, ~06:15:18 local):**
```
Sentry: alive, ambient=64.4dB serverFound=1 heap=157936 ... uptime=19s
handshakes=1 silent=3s rreset=0 stale=0
```
`handshakes=1` confirms exactly one session was established since this
boot - the capture below is that session in full, not mixed with an older
one.

## 3. The handshake, record by record

```
TIME (UTC)    DIRECTION              LEN   RECORD
--------------------------------------------------------------------------
04:15:08.089  103:50289 -> 112:11400 206   PLAINTEXT  ClientHello   epoch=0
04:15:09.085  103:50289 -> 112:11400 206   PLAINTEXT  ClientHello   epoch=0
04:15:09.085  112:11400 -> 103:50289 144   PLAINTEXT  ServerHello   epoch=0
04:15:09.093  103:50289 -> 112:11400 279   PLAINTEXT  ClientHello   epoch=0
04:15:09.095  112:11400 -> 103:50289 700   PLAINTEXT  ServerHello   epoch=0
04:15:09.096  112:11400 -> 103:50289 175   ENCRYPTED  unified header 0x2e
04:15:10.364  112:11400 -> 103:50289 875   PLAINTEXT  ServerHello   epoch=0
04:15:11.048  103:50289 -> 112:11400 393   ENCRYPTED  unified header 0x2e
04:15:11.300  112:11400 -> 103:50289  40   ENCRYPTED  unified header 0x2e
...
--------------------------------------------------------------------------
6 records readable, rest encrypted
```

(112 = 192.168.0.112, the server; 103 = 192.168.0.103, NMU_21.)

**Result: 6 plaintext records again** - 3 ClientHello, 3 ServerHello, all
`epoch=0`, identical in kind to the NMU_18 capture from the day before. From
the first encrypted record onward (04:15:09.096), everything - certificates,
identity, sensor readings, ACKs - is opaque.

## 4. What this does and does not prove

**Confirmed, now on two independent units on two different days:** the
plaintext portion of a DTLS 1.3 handshake here is not a one-off artifact -
it reproduces. Both captures show the same shape: an initial ClientHello,
a retransmission of it about a second later (no answer arrived in time),
then a real exchange, then one retransmitted ServerHello flight before the
client's response is seen.

**Not proven, and should not be claimed:** that RFC 9147 mandates exactly
three ClientHello/ServerHello pairs. It does not - the standard's only firm
requirement is at least one of each in cleartext, because no shared secret
exists yet to encrypt them with. The repeated 3x pattern seen twice here is
consistent with this specific link's round-trip timing relative to the
retransmission timer, not a protocol constant. A handshake with no packet
loss and no retry would show as few as 2 plaintext records total.

## 5. Incidental finding - useful for experiment #12

This 40-second capture, taken for a single unit's handshake, incidentally
recorded 182 packets from at least 11 other device IPs
(192.168.0.100/101/104/105/107/109/110/114/115/116/118) all exchanging
routine encrypted traffic with the server on port 11400 during the same
window - concurrent fleet activity, unprompted, while this test ran. Worth
reusing as raw material for experiment #12 (scale/timing) rather than
re-capturing separately.

## 6. Root cause of the 3x, found in code rather than guessed

Both ends of this handshake run wolfSSL, and DTLS is required by RFC 9147 to
retransmit any handshake flight that goes unacknowledged (UDP does not resend
lost packets on its own). What varies is only HOW OFTEN a flight actually
misses its window - and both ends here use a short window.

**NMU side** - `nmu/omega_dtls.cpp:16-17`:
```cpp
static const int DTLS_RETRY_INIT_S = 1;   // first retry after 1 s
static const int DTLS_RETRY_MAX_S = 4;    // never wait longer than 4 s
```
Deliberately capped low (`omega_dtls.cpp:124-144`): wolfSSL's uncapped default
backoff doubles forever (1, 2, 4, 8, 16 s...), and against a dead server this
used to outlast the 30 s task watchdog and reboot the whole fleet on a server
outage - a real incident, not a hypothetical. This cap is the fix.

**Server side** - `server/session.py` has no equivalent override, so it runs
wolfSSL's library default retransmit timer, which is also ~1 s initial.

**Matched against this capture's actual gaps:**

| Records | Gap | Explanation |
|---|---|---|
| ClientHello #1 -> #2 | 0.996 s | NMU's 1 s timer fired - #1 went unanswered |
| ServerHello #1 (144B, cookie reply) -> ClientHello #3 (279B, carries cookie) | ~8 ms | normal continuation, not a retry |
| ServerHello #2 (700B, real) -> #3 (875B) | 1.269 s | server's own ~1 s default timer fired - its flight went unanswered |

**Conclusion:** the 3x pattern is a timer artifact, not a protocol constant.
Two flights in this exchange (the opening ClientHello, then the real
ServerHello) each independently missed a 1-second acknowledgement window -
ordinary first-contact WiFi/UDP jitter is enough to explain both misses on
its own. A handshake where neither flight is delayed past 1 s would show the
RFC-minimum 2 plaintext records, not 6.

## Files

- `captures/2026-08-27_nmu21_recapture.pcap`
