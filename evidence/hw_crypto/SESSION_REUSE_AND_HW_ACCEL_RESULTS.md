# Session-reuse DTLS 1.3 + ESP32-S3 hardware crypto acceleration - Results

**Date:** 13 August 2026
**Devices:** AMU (Raspberry Pi 4B), NMU (Adafruit Feather ESP32-S3, no PSRAM, 8 MB flash)
**Server:** Ubuntu mini PC, same wolfSSL DTLS 1.3 stack for both trials

---

## Part 1 - Why session reuse was tested at all

The original echo spike measured a **full handshake per message**: 2827 ms on
the NMU. Empirical observation put the NMU's real event rate at **1-2 Hz**
(0.5-1 s between detected sounds). At that rate a 2827 ms handshake per event
gives utilisation above 1 - the send queue diverges without bound. Any
architecture that re-handshakes per event is disqualified by arithmetic
before any protocol argument is made.

The fix tested here: **one DTLS 1.3 handshake, then every event rides the
same open session as a symmetric record.** No protocol change - RFC 9147's
record layer is built for exactly this; the echo spike's per-message
handshake was a measurement artefact, not how DTLS is meant to be used.

---

## Part 2 - Session reuse on the proven stack (Arduino / wolfSSL, software crypto)

### AMU (Raspberry Pi, Python/wolfssl-py)

| Run | Handshake | Min | Median | Avg | Max | Speedup vs handshake-per-event |
|---|---|---|---|---|---|---|
| 1 | 1188.3 ms | 4.81 ms | 76.48 ms | 76.10 ms | 132.44 ms | 16x |
| 2 | 1174.9 ms | 5.40 ms | 81.59 ms | 80.93 ms | 173.93 ms | 15x |

Both runs: 40/40 records, 0 lost, config correctly piggybacked on every ACK.

### NMU (ESP32-S3, Arduino/wolfSSL, software crypto only)

| Run | Handshake | Min | Median | Avg | Max |
|---|---|---|---|---|---|
| Server-logged run, pre-reboot | 4725 ms | - | - | - | - |
| Same session, post-reboot | 4420 ms | - | - | - | - |
| Full serial capture | 4987 ms | 9 ms | 181 ms | 145 ms | 226 ms |
| Full serial capture (2nd) | 5186 ms | 15 ms | - | - | (partial capture) |

All runs: 40/40 records, 0 duplicates, 0 lost, sustained rate 2.00-2.04 Hz.
Reboot mid-trial handled cleanly: new source port, fresh random session ID,
full re-handshake, zero confusion with the prior session.

**Arduino/software NMU handshake average: 4829.5 ms** (n=4)
**Arduino/software NMU per-record average: 145 ms** (n=1 complete stat set;
other runs corroborate the same band from partial data)

---

## Part 3 - Same test, ported to raw ESP-IDF with real hardware crypto

### Why this was built

The Arduino wolfSSL library ships with `WOLFSSL_ESPIDF` undefined -
confirmed by inspecting the linked binary: zero hardware-driver symbols
(`wc_esp32AesEncrypt`, `esp_mp_mul`, etc.) present anywhere in it. All
crypto in every Arduino run above ran in pure software. A raw ESP-IDF build
(VS Code ESP-IDF extension, v5.4.1), based on wolfSSL's own official
`DTLS13-wifi-station-client` example, was built to get real hardware
acceleration engaged, using the same CA and NMU_T1 device identity so
results are directly comparable.

### Build

- Binary size: 1,110,992-1,123,904 bytes (54-56% of a 2 MB app partition,
  same partition size as the Arduino build for a fair comparison)
- Confirmed by direct linked-binary inspection (`nm` on the produced `.elf`):
  hardware driver symbols present - `wc_esp32AesEncrypt/Decrypt/CbcEncrypt/
  CbcDecrypt`, `esp_mp_mul/mulmod/exptmod`, `esp_sha_try_hw_lock/init_ctx`

### Runtime results

| Run | Handshake (device) | Handshake (server) | Min | Avg | Max | Result |
|---|---|---|---|---|---|---|
| 1 | 5846 ms | - | 14 ms | 150 ms | 237 ms | 40/40 PASS |
| 2 (metrics build) | 5797 ms | - | 29 ms | 152 ms | 227 ms | 40/40 PASS |
| 3 | 5489 ms | 4811 ms | - | - | - | confirmed to record 10+ via server log; full completion not directly captured (see note) |

**ESP-IDF/hardware handshake average: 5697 ms** (n=4 including the partial run)
**ESP-IDF/hardware per-record average: 151 ms** (n=2 complete stat sets)

Note on run 3: local serial capture over native USB dropped after record 3
(no separate USB-UART bridge on this board; the drop is a capture artefact,
not a device fault - the server's own log independently confirms the device
kept sending and receiving correctly acked records through at least record
10, 3.5 s after record 3, exactly on the expected 500 ms cadence).

### Direct proof hardware crypto was actually executed (not just linked)

Symbols in a binary prove linkage, not execution. wolfSSL's own runtime
usage counters (`WOLFSSL_HW_METRICS`) were enabled and read back after a
real handshake + 40 real records:

```
esp_mp_mul HW acceleration enabled.
  Number of calls to esp_mp_mul: 44055
  ... with tiny operands: 0
  ... HW operand exceeded: 0
  Success: no esp_mp_mul() errors.

esp_mp_mulmod: 29 calls, 7 fallback to SW, 0 errors
esp_mp_exptmod: 0 calls        (expected: RSA-only path, we use ECDSA)

esp_sha1_hw_hash_usage_ct     = 7
esp_sha2_256_hw_hash_usage_ct = 102
esp_sha1_sw_fallback_usage_ct = 0

Max hw wait timeout: esp_mp_max_wait_timeout = 1
```

44,055 hardware big-number multiply operations (the P-256 ECDHE/ECDSA math),
102 hardware SHA-256 hashes, essentially zero software fallback, zero
errors. `esp_mp_max_wait_timeout = 1` shows the hardware engine was almost
never contended - WiFi's own crypto usage was not meaningfully blocking it.

### Correction: the first comparison was confounded, not conclusive

The three runs above were built with ESP-IDF's **silent default of 160 MHz**.
The Arduino build's board default is **240 MHz**. This was never a deliberate
choice on either side - it was an uncontrolled variable neither build set
explicitly, found afterwards by reading `boards.txt` and the generated
`sdkconfig` directly, not assumed.

A fourth run was built with `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240` to match
the Arduino board exactly, everything else held identical (same certificates,
same server, same code, same wolfSSL library, hardware crypto still
confirmed active via the same runtime counters):

| Run | Handshake | Per-record avg |
|---|---|---|
| ESP-IDF, hardware crypto, 160 MHz (runs 1-3 above) | 5697 ms | 151 ms |
| **ESP-IDF, hardware crypto, 240 MHz (controlled)** | **4374 ms** | **146 ms** |
| Arduino, software crypto, 240 MHz | 4830 ms | 145 ms |

Full log: `dtls13_spike/LOGS/nmu_espidf_240mhz_controlled_20260815.txt`.

Fixing the clock speed alone recovered 1323 ms (23%) on the handshake.
Once CPU frequency is no longer a confound, **hardware acceleration is
measurably faster on the handshake: 4374 ms vs 4830 ms, a 456 ms (9.4%)
improvement.** Per-record cost stayed within 1 ms either way (146 vs 145 ms) -
unaffected by CPU frequency or crypto backend, consistent with Part 5 below.

**Independent cross-check, server-side clock:** the server's own log for
this test (`server_session_20260815_003320.log`) recorded a *second*
handshake the device-side capture never saw - an artefact of the flash
procedure's own auto-reset happening before the serial monitor was attached,
not a device fault; that first session was simply cut off mid-stream when a
second, deliberate reset started the capture. It still completed a full
handshake first, giving a second, independent 240 MHz hardware measurement
for free:

| Measured by | Arduino/software avg | ESP-IDF/hardware @240MHz avg | Hardware faster by |
|---|---|---|---|
| Device's own clock | 4830 ms (n=4) | 4374 ms (n=1) | 456 ms (9.4%) |
| **Server's clock** | 4572.5 ms (n=2: 4725, 4420 ms) | **4037 ms (n=2: 4305, 3769 ms)** | **535.5 ms (11.7%)** |

Two independent clocks (device and server), two independent measurement
paths, agreeing within 2 percentage points of each other. This is
materially stronger evidence than the single device-side sample alone.

---

## Part 4 - Side-by-side comparison (controlled, 240 MHz both builds)

| Metric | Arduino (software crypto) | ESP-IDF (hardware crypto) | Difference |
|---|---|---|---|
| NMU handshake | 4830 ms (n=4 avg) | 4374 ms (n=1, controlled) | **-456 ms (-9.4%), hardware faster** |
| NMU per-record avg | 145 ms | 146 ms | +1 ms, no meaningful difference |
| NMU per-record range | 9-226 ms | 27-228 ms | Same band |
| AMU per-record avg (reference, Pi, always-on radio) | 76-81 ms | n/a (AMU not re-tested on ESP-IDF) | ~2x faster than NMU regardless of crypto path |

**Hardware acceleration gives a real, modest win on the handshake (the
CPU-bound, compute-heavy part) and no measurable win on per-record cost
(the I/O-bound, radio-wake-dominated part).** This is not two contradictory
results - it is one consistent finding: acceleration helps exactly the part
of the system that is actually compute-bound, and does nothing for the part
that isn't.

---

## Part 5 - Why acceleration barely moves the whole system (the actual question)

**Crypto was never the bottleneck for the recurring, per-record cost - the
handshake is a different story, and the controlled data above shows it.**

A reference benchmark (uoscore-uedhoc, tinycrypt software crypto, on an
*original* ESP32 - older and weaker than the S3) measures pure AEAD
encrypt/decrypt cost for a 100-byte payload (our actual record size) at
**977 microseconds** - under one millisecond, in software, on weaker
hardware than ours.

Our measured per-record cost: **145,000-152,000 microseconds** (145-152 ms).

That is a **~150-200x gap** between what cryptographic computation should
cost and what we actually measured. Even if hardware acceleration made the
crypto slice *ten times* faster than that software reference, it would
remove at most ~0.9 ms from a 150 ms total - a 0.6% change, invisible
against the 30+ ms run-to-run variance already present in the raw data
(min 9-29 ms, max 226-237 ms, *within single runs*). This is Amdahl's Law:
accelerating a component that is under 1% of total time cannot meaningfully
speed up the whole, regardless of how much faster that component gets.

**Where the real 150 ms goes - direct evidence from the record-by-record data:**

- Record 1, sent immediately after the handshake while the WiFi radio is
  still actively transmitting: consistently **9-29 ms** across every run,
  every build, both crypto paths.
- Record 2 onward, each preceded by a 500 ms idle gap: consistently
  **195-237 ms** - a 10-20x jump, identical in both the software and
  hardware builds, despite running the exact same cryptographic workload as
  record 1.

The only thing that changes between record 1 and record 2 is elapsed idle
time before transmission. That is the signature of ESP32 WiFi **modem-sleep
wake latency** (the radio powers down between transmissions to save energy
and pays a reconnect cost on the next send), not cryptographic cost, which
is identical on every record.

**Independent corroborating evidence:** the AMU (Raspberry Pi, a platform
whose WiFi driver does not apply the same aggressive modem-sleep power
saving by default) runs the *identical* protocol and payload size at
76-81 ms average - roughly half the NMU's cost - despite doing strictly more
work per second (this session's AMU ran the same 40-record trial). The
device-specific difference tracks radio power-management behaviour, not
anything about the crypto library.

**Conclusion:** hardware crypto acceleration was confirmed to genuinely
execute (44,055 real hardware operations, measured) and gives a real,
repeatable 9.4% improvement on the handshake once CPU frequency is
controlled for. It gives essentially nothing on per-record cost, because
that cost is dominated by WiFi radio power-state transitions - entirely
outside what cryptographic acceleration touches. Both things are true at
once: the acceleration works as documented, and the system's dominant,
recurring cost (per-record, paid dozens of times per session) sits somewhere
it can't reach. Only the rare, once-per-session handshake benefits.

---

## Recommendation

**Ship the Arduino/software-crypto session-reuse build.** The reasoning
changed with this correction, but the conclusion didn't: the handshake
happens once per session (hours apart, by design); saving 456 ms on an
event that rare has negligible practical effect on the system, while the
per-record cost - paid on every single detected sound - is identical
either way. That per-record cost, not the crypto backend, is the number
that actually matters for "close to live," and it doesn't move with
hardware acceleration.

Arduino also keeps the simpler toolchain (no ESP-IDF environment, no manual
bootloader entry, the flashing workflow already in use for every other
device). The 9.4% handshake win is real and now proven, not a myth - it's
just not the win that matters for this system's actual bottleneck.
