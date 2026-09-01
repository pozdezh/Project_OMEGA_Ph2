# Marina's ladder, climbed on our own hardware: one reading, four protections

Run 2026-08-27 06:51-06:57 CEST on the deployed fleet. Subject: **AMU_T1**
(192.168.0.115), a live production unit, while it kept doing its normal job.

This is the OFF-vs-ON comparison Marina Rull's TFG uses throughout her
evaluation chapter, applied to this system - and extended from her two rungs
to four, so it also answers "why not just do what Marina did?"

Scripts (both committed, this is the how):
- `simlab/marina_ladder_emit.py` - ran on AMU_T1
- `simlab/marina_ladder_analyse.py` - ran on the dev PC against the capture

Raw evidence:
- `captures/2026-08-27_marina_ladder.pcap` (534 datagrams, 226 kB)
- `captures/2026-08-27_marina_ladder_analysis.txt` (the analyser's own output)

---

## 1. The four rungs

| Rung | Protection | Represents |
|---|---|---|
| **A** | none, plain JSON over UDP | the pre-security state; EmotiBit's original protocol |
| **B** | AES-128-**ECB** + HMAC-SHA256, pre-shared key | **Marina's actual design** |
| **C** | AES-256-**GCM** | **her stated future work** == Omega brick1 |
| **D** | **DTLS 1.3**, mutual-auth PKI | Omega brick4, in production |

## 2. Method, exactly - and why it is a fair comparison

The hard part of an OFF/ON comparison is making both halves carry the *same*
data. A re-measurement or a mock invites the objection that the two sides were
never comparable.

**How that was solved.** AMU_T1's sampling loop publishes its newest reading
atomically to `/dev/shm/omega_amu_latest.json` (a tmpfs file; see
`amu/live_cache.py`). The emitter reads that same file and re-sends **that
exact payload**, in the exact key order `main.py` builds it, down rungs A, B
and C. Meanwhile the untouched production service carried the identical
reading down rung D over its real DTLS 1.3 session.

So all four rungs carry one unit's real CO2, particulate, temperature and
humidity values, from the same seconds, over the same WiFi.

**Why the cache and not the sensors.** The running service owns the I2C and
serial buses. A second process re-reading the sensors would contend with it on
a deployed unit. Reading the cache cannot disturb sampling and proves the data
is the same data, not a second measurement.

**Nothing listened on the rung A/B/C ports** (11501/11502/11503). That is
deliberate, not an oversight: the claim under test is *what an eavesdropper
can read off the wire*, and a packet is on the wire whether or not anything
accepts it. It also meant the server required no change whatsoever for this
experiment.

**Commands, reproducible.** Capture started by the user over their own SSH
session (the sudo password never enters a command Claude runs):

```
sudo timeout 360 tcpdump -i any -s 0 -w /tmp/marina_ladder_20260827.pcap \
  'udp port 11400 or udp port 11501 or udp port 11502 or udp port 11503'
```

Emitter, on the AMU:

```
LADDER_DEVICE_ID=AMU_T1 LADDER_SERVER=192.168.0.112 \
LADDER_RECORDS=110 LADDER_INTERVAL_S=3.0 python3 marina_ladder_emit.py
```

The analyser **learns** the AMU's address from the rung A packets and filters
rung D to that same address, so the four rungs are same-device by
construction rather than by assertion.

## 3. Results, measured

```
RUNG PROTECTION                   RECS   AVG B  ENTROPY  REPEATED     READABLE
------------------------------------------------------------------------------
A    none (plain JSON)            110    478    4.94     2811/3193    everything
B    AES-128-ECB + HMAC-SHA256    110    512    7.90     2814/3523    nothing
C    AES-256-GCM                  110    506    7.98     0/3303       nothing
D    DTLS 1.3 (mutual-auth PKI)     4    528    7.91     0/129        nothing
------------------------------------------------------------------------------
REPEATED = identical 16-byte blocks / total blocks in that rung
ENTROPY  = Shannon bits per byte (8.00 would be perfectly random)
```

Rung A, in full, is readable with a text search: `"id"`, `"type"`, `"ts"`,
`"cause"`, `"event"`, `"sensors"`, `AMU_`, `airq`, `co2_ppm`, `scd30`,
`dht22`, `pm2_5`, `light_lux`, `temperature_c` - 110 hits each, one per
record. Rungs B, C and D yield **zero hits on every term**.

## 4. The finding that matters

**Entropy alone cannot tell Marina's scheme apart from ours.** Rung B scores
7.90 bits/byte against rung C's 7.98 and rung D's 7.91. On that test - the
test our own earlier evidence file `2026-08-26_live_wire_capture.md` leans on -
AES-128-ECB passes and looks just as encrypted as DTLS 1.3.

**Block-repetition separates them completely.**

- rung A, the plaintext: **2811** of 3193 blocks are repeats
- rung B, the ECB ciphertext: **2814** of 3523 blocks are repeats
- rung C and rung D: **exactly zero**

Rung B's repeat count (2814) reproduces rung A's (2811) almost one for one.
That is the whole point: ECB encrypts each 16-byte block independently with
one fixed key and no IV, so **equal plaintext blocks always become equal
ciphertext blocks**. The ciphertext therefore carries the plaintext's
repetition structure across intact, even though not one byte of it is
readable. An observer who cannot read a single value can still see which
records share content, when a reading returns to a previous state, and where
the constant JSON scaffolding sits in every record.

GCM and DTLS 1.3 score zero because each record gets a fresh nonce, so
identical plaintext never produces identical ciphertext twice.

**This is precisely the weakness Marina names in her own Future Work (7.1)**,
where she identifies AES-GCM as the natural next step. This run measures that
gap on real sensor data from a real deployed unit, rather than restating it.

## 5. What each rung actually guarantees

| Property | A none | B ECB+HMAC | C AES-GCM | D DTLS 1.3 |
|---|---|---|---|---|
| Confidentiality | no | **partial - leaks block equality** | yes | yes |
| Integrity | no | yes (HMAC) | yes (AEAD tag) | yes (AEAD) |
| Authenticity | no | "someone holding the PSK" | "someone holding K_dev" | **this certificate, CA-signed** |
| Replay defence | no | app-level counter | nonce + DB unique index | protocol-level epoch/sequence |
| Forward secrecy | no | **no** | **no** | **yes (ephemeral ECDHE)** |
| Key exchange | n/a | none, pre-shared out of band | none, derived from a master | **full mutual handshake** |
| Compromise blast radius | n/a | that key's devices | **master key = whole fleet** | one device, **revocable** |
| Revocation | n/a | manual key rotation | manual re-derivation | dashboard, takes effect at once |

The bottom two rows are where the categories genuinely differ. A and B and C
all rest on a secret that must be distributed before the fact and that has no
way to be withdrawn except by touching every device. D distributes nothing in
advance except a CA certificate, and can retire one unit from a web page.

## 6. Honest limitations of this run

- **Rung D is n=4.** AMU_T1 transmits about once every 85 seconds (measured:
  7 records in the preceding 600 s, from the server's own database), so a
  5.5-minute window yields four. Four records is enough to show 0/129
  repeats but it is a smaller sample than rungs A-C, and is stated as such.
  It is consistent with rung C's 0/3303 over the same AEAD reasoning.
- **Rung B is our reconstruction**, from Marina's documented design
  (per-device pre-shared key, AES-128-ECB, HMAC-SHA256 over the ciphertext so
  the tag is checked before decrypting). It is **not her source code**, which
  was not available to us. If her implementation differs in a detail, the ECB
  block-equality property is a property of the mode itself and would not
  change.
- **Rung C uses brick1's real frozen code** (`brick1/crypto/omega_crypto.py`)
  unmodified, not a reimplementation.
- The two keys in the emitter are **throwaway experiment constants** committed
  on purpose for reproducibility. No deployed device or server has ever held
  them.
- Sensor values genuinely fluctuated during the run - that is why rung A is
  2811/3193 rather than a flat 100%. A frozen reading would have overstated
  every repeat figure.
- No handshake occurred inside the capture window: all four rung D records
  are DTLS 1.3 application-data records (unified header `0x2f`) with four
  matching ACKs returning. AMU_T1 held **one** session throughout. The
  handshake itself is evidenced separately in
  `2026-08-27_nmu21_handshake_recapture.md`.

## 7. What this does and does not prove

**Proves**, on deployed hardware, over real UDP, with the same payload:
- cleartext telemetry is completely readable, including device identity
- Marina's ECB scheme hides every value but leaks the repetition structure
- AES-GCM (her proposed improvement, our brick1) closes that leak
- DTLS 1.3 closes it too, and additionally replaces a pre-shared secret with
  a per-device certificate that can be revoked

**Does not prove** - and is not claimed:
- anything about handshake soundness; a capture shows opaque bytes, not that
  peers verified each other (see the attack suite and the handshake capture)
- that ECB is broken in the sense of key recovery. It is not. The leak
  demonstrated here is traffic-pattern leakage, which is a real
  confidentiality loss but a different and lesser thing than a broken cipher
- anything about performance. No timing or CPU comparison was made in this run
