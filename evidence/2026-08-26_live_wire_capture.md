# What the fleet actually looks like on the wire

Captured 2026-08-26 02:33 on the live server (`smartageing.local`, 192.168.0.112)
with `tcpdump -i any -s 0 "udp port 11400"`, 100 seconds, while the whole
deployed fleet was running normally. Nothing was staged: these are the real
units sending real readings.

Raw capture: `evidence/captures/2026-08-26_live_fleet_dtls13.pcap`
(open in Wireshark; 50 packets, 10.5 kB)

## Why this exists

The attack suite (`evidence/attack_suite.md`) proves the protocol's properties
in a controlled harness. It proves them over DTLS **1.2**, in memory, on a
development machine, against invented devices - and says so.

This is the other half, and the half a reader believes without trusting any of
our code: **a packet capture of the deployed system, taken from outside it.**
The technique is borrowed from the parallel pre-shared-key project (Rull Ventura), where every claim is shown
as a Wireshark capture rather than a passing test.

## What the capture shows

Multiple units reporting to one server, each from its own address:

```
02:33:46.215886  192.168.0.101.36884 > 192.168.0.112.11400: UDP, length 535
02:33:46.217266  192.168.0.112.11400 > 192.168.0.101.36884: UDP, length 101
02:33:51.747711  192.168.0.107.50902 > 192.168.0.112.11400: UDP, length 132
02:33:51.747713  192.168.0.105.57785 > 192.168.0.112.11400: UDP, length 132
02:33:51.747714  192.168.0.115.49388 > 192.168.0.112.11400: UDP, length 131
02:33:51.747714  192.168.0.100.51823 > 192.168.0.112.11400: UDP, length 130
02:33:51.749553  192.168.0.112.11400 > 192.168.0.105.57785: UDP, length 104
```

Four devices delivering within the same millisecond and four ACKs going back:
the server is holding independent authenticated sessions concurrently, not
serialising them.

## Test 1: is any of it readable?

Every word the system uses, searched for across the whole capture:

| term | occurrences |
|---|---|
| `AMU` | 0 |
| `NMU` | 0 |
| `co2` | 0 |
| `scd` | 0 |
| `temp` | 0 |
| `hum` | 0 |
| `cause` | 0 |
| `alarm` | 0 |
| `heartbeat` | 0 |
| `event` | 0 |
| `ack` | 0 |
| `id` | 0 |

**Zero.** Not the device names, not the sensor names, not the JSON keys, not
even the word `id`. Before this project the same payloads travelled as plain
JSON, where every one of those terms was readable with the same command.

## Test 2: what IS there instead

First bytes of a real 535-byte record from AMU_T1:

```
0x0020:  12a7 039e 7cf5 c03f 7eff 94cc 3517 ffa3
0x0030:  83a0 18f7 58b1 ac5f 3a84 e2c7 7fc2 958d
0x0040:  0cea 7194 9c8a cc80 7547 a449 23e3 419d
```

Measured over the payload:

- **521 bytes**
- **Shannon entropy 7.61 bits per byte** (8.00 would be perfectly random)
- **221 distinct byte values out of 256**

Entropy is a measure of how unpredictable the bytes are. Plain JSON scores
around 4-5 bits per byte, because it is mostly letters, braces and repeated
key names. At 7.61 with 221 distinct values, this is statistically
indistinguishable from random noise - which is what correctly encrypted data
is supposed to look like.

## What this does and does not prove

**Does prove**, on the deployed system, over real UDP:
- payloads are encrypted, not encoded or obfuscated
- no device identity, sensor name or reading is recoverable by watching
- the server serves many authenticated devices concurrently
- ACKs return per-device, so delivery is tracked individually

**Does not prove** - and must not be claimed:
- that the *handshake* is sound. A capture shows opaque bytes; it cannot show
  that the peers verified each other. That is what the attack suite and the
  hardware handshake logs (`DTLSv1.3 / TLS_AES_128_GCM_SHA256`) are for.
- anything about attacks. This is passive observation only - the eavesdropper
  scenario, and nothing more.

An observer on this network learns exactly three things: that a device exists
at an address, roughly when it spoke, and how long the message was.
