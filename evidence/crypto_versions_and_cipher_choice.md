# DTLS library versions, and the AES-128 cipher-suite question

Memo reference sheet. Written 2026-08-28 against the frozen code base - no new
hardware runs, nothing here needs the fleet reconnected. Every claim below
names either a file and line in this repo, or a primary external source, so it
can be checked rather than believed.

Two questions are answered:

1. Exactly which DTLS library, at which version, runs on each of the three
   device types.
2. Whether running `TLS_AES_128_GCM_SHA256` rather than
   `TLS_AES_256_GCM_SHA384` is defensible, and on what evidence.

---

## 1. Library versions per device type

All three device types run **wolfSSL**. That is itself a deliberate result:
it is the only stack that speaks DTLS 1.3 on all three, and the mbedTLS
DTLS 1.2 path crashed the NMU
(`dtls13_spike/LOGS/OPEN_ISSUE_dtls12_nmu_panic.md`).

| | Library | Version | Pinned? | Where that is proven |
|---|---|---|---|---|
| **NMU** (ESP32-S3) | wolfSSL Arduino library (C) | **5.8.4** | **Yes** | `deploy/server/install_hub.sh:186` |
| **Server** (Ubuntu) | `wolfssl` PyPI package (Python binding + bundled wolfSSL C) | **5.9.2.post0** | **No - floor only** | `deploy/server/payload/requirements.txt:10` |
| **AMU** (RPi 4B) | `wolfssl` PyPI package (same) | **5.9.2.post0** | **No - floor only** | `deploy/amu/payload/requirements_delta.txt:7` |

### 1.1 NMU - wolfSSL 5.8.4, genuinely pinned

The hub installer pins the Arduino library explicitly:

```
deploy/server/install_hub.sh:186
ARDUINO_LIBS="wolfssl@5.8.4 ArduinoJson@7.4.3"
```

The reason is recorded at the time the decision was made
(`DEV_CHRONOLOGY.md:1003`):

> "PINNED to the tested versions (wolfssl 5.8.4, ArduinoJson 7.4.3), because
> these are compiled into a device: an API change in a newer release is
> discovered only after flashing, and undone only by reflashing every unit by
> hand."

Confirmed in the installed library on this PC:

- `~/Arduino/libraries/wolfssl/library.properties` -> `version=5.8.4`
- `~/Arduino/libraries/wolfssl/src/wolfssl/version.h` ->
  `LIBWOLFSSL_VERSION_STRING "5.8.4"`, `LIBWOLFSSL_VERSION_HEX 0x05008004`

Independently corroborated by the trial log written at the time:
`dtls13_spike/LOGS/nmu_dtls13_timed.txt:1` - "NMU trial - DTLS 1.3 via wolfSSL
Arduino library 5.8.4" - and `dtls13_spike/LOGS/RESULTS.md:31`.

Note for accuracy: the NMU source itself (`nmu/omega_dtls.cpp`) only does
`#include <wolfssl.h>`; it does not assert a version. The pin lives in the
installer, which is the right place for it, but it means a hand-installed
library on some other machine would not be pinned. Also cosmetic: the
library's own `user_settings.h` header comment still reads
`WOLFSSL_USER_SETTINGS_ID "Arduino user_settings.h v5.8.2"` - a stale label
inside wolfSSL's own template, not the library version.

### 1.2 Server and AMU - wolfSSL 5.9.2.post0, now PINNED

**UPDATE 2026-08-29:** the pin was applied. Both requirement files, and the
AMU live-agent build script, now specify the exact version:

```
deploy/server/payload/requirements.txt      wolfssl==5.9.2.post0
deploy/amu/payload/requirements_delta.txt   wolfssl==5.9.2.post0
deploy/amu/build_live_py.sh                  pip install ... wolfssl==5.9.2.post0
```

Verified against the running server (`ssh omega-server`): the listener's
venv (`/home/smart/omega_spike13/venv`) has `wolfssl 5.9.2.post0`, built from
a fixed local source tree - so the live host already matches the pin and no
reinstall is needed; the pin protects any future install. Installer banners
bumped to `2026-08-29a-wolfssl-pinned`.

The pre-pin history follows.

```
deploy/server/payload/requirements.txt:10       wolfssl>=5.9.0   (was)
deploy/amu/payload/requirements_delta.txt:7     wolfssl>=5.9.0   (was)
```

The version actually installed and tested is recorded twice, in two
independent contexts:

- `DEV_CHRONOLOGY.md:300` - a single-variable experiment run across both
  hosts: "same code, same **wolfSSL 5.9.2-0**, server's Python 3.10 vs AMU's
  3.13".
- `FINDINGS.md:1014` - "PyPI ships this package as source only
  (**`wolfssl-5.9.2.post0.tar.gz`**, no `.whl`)".

Cross-checked against PyPI's own release metadata (fetched 2026-08-28):
**5.9.2.post0, released 2026-07-16, is the latest release**, and every recent
release is source-only with no binary wheels. So `>=5.9.0` resolves to
5.9.2.post0 today - the floor and the tested version currently coincide.

**Position now:** the deployed and tested version is 5.9.2.post0, and as of
2026-08-29 it is pinned exactly in both requirement files and the live-agent
build script (see the update box above). The NMU was already closed (compiled
into the device; a bad version costs a reflash); the two Linux hosts are now
closed too.

### 1.3 Why the versions differ between NMU and the Linux hosts

They are two different distributions of the same library: wolfSSL publishes an
**Arduino library** (5.8.4) and a **PyPI package** (5.9.2.post0) on separate
release cadences. Nothing forces them to match, and interoperability is not
version-locked - it is protocol-locked by RFC 9147. This is proven, not
assumed: NMU 5.8.4 and server 5.9.2 complete mutual-auth DTLS 1.3 handshakes
continuously in every live capture in `evidence/`.

### 1.4 Why the Python package is compiled from source on the Pi

Not a preference - a constraint. PyPI publishes `wolfssl` **source-only, no
wheels** (verified above, and recorded independently at `FINDINGS.md:1014`),
so `pip install wolfssl` compiles the bundled wolfSSL C library on the target
machine. That is why `deploy/amu/install_amu.sh` installs autotools first
(`install_amu.sh:58-78`, comment: "The wolfSSL binding has no prebuilt wheel
for this Pi's Python, so pip compiles the wolfSSL C library from source").

Build flags used for that bundled library, from wolfssl-py's own build script
(`wolfssl/_build_ffi.py`, github.com/wolfSSL/wolfssl-py): `--disable-shared`,
`--disable-examples`, **`--enable-dtls13`**, `--enable-dtls`, `--enable-crl`,
`--enable-opensslextra`, `--enable-tlsx`, `--enable-opensslall`. This is a
full-featured build, unlike the deliberately trimmed embedded one - which is
exactly the asymmetry section 2 turns on.

### 1.5 How wolfSSL is obtained and built, per device type - and the NMU is not special

All three device types run **wolfSSL compiled from C source on the build
machine**. The mechanism differs, the principle does not.

| | Package | Fetched from | Compiled | Config |
|---|---|---|---|---|
| Server, AMU | `wolfssl` (PyPI) | pip, source tarball (no wheels) | once, at `pip install` -> a `_ffi.so` the Python process loads | `configure` flags in `wolfssl-py/_build_ffi.py` - full build |
| NMU | `Arduino-wolfSSL` library, pinned `wolfssl@5.8.4` | Arduino Library Manager index | with every sketch build (subject to the build cache), linked into the firmware | `src/user_settings.h` - a trimmed embedded config |

So the NMU is the **same "fetch source, compile locally, use" model** as the
Linux hosts - just at sketch-build time instead of install time, and with a
smaller default feature set.

**Arduino IDE vs `arduino-cli` - no difference.** `arduino-cli lib install
wolfssl@5.8.4` and searching "wolfssl" in the IDE's Library Manager pull the
**same files from the same index** ([Arduino library-registry FAQ](https://github.com/arduino/library-registry/blob/main/FAQ.md)).
The library is **C source, not a precompiled blob** - it is
[`Arduino-wolfSSL`](https://github.com/wolfSSL/Arduino-wolfSSL), "a
restructured copy of wolfSSL for the Arduino environment", configured through
`libraries/wolfssl/src/user_settings.h` with `WOLFSSL_USER_SETTINGS` defined
project-wide ([wolfSSL: Getting Started on Arduino](https://www.wolfssl.com/getting-started-with-wolfssl-on-arduino/)).
Both IDE and CLI compile that same `user_settings.h`.

**Choosing AES-128 vs AES-256** is not an IDE-or-CLI setting and not a
runtime choice - no cipher list is set anywhere. Whether
`TLS_AES_256_GCM_SHA384` is *available to negotiate* is decided at compile
time by `#define WOLFSSL_SHA384` in `user_settings.h` (AES-256 auto-enables
with it - section 2.2). Editing that file and rebuilding works identically
under the IDE and the CLI; the project left it at the default (SHA-384 off ->
AES-128 only). Practical note: both toolchains cache compiled libraries, so
after editing `user_settings.h` force a clean rebuild.

**Concretely, to enable AES-256 on the NMU** (not done in this project, kept
here so the option is documented):

1. Open the library's local config file. Under Arduino IDE 2.x on Windows
   this is `%LOCALAPPDATA%\Arduino15\libraries\wolfssl\src\user_settings.h`;
   under `arduino-cli` it is `<sketchbook>/libraries/wolfssl/src/user_settings.h`.
   Same file, same content - the Library Manager and `arduino-cli lib
   install` populate it from the same registry entry.
2. Add one line: `#define WOLFSSL_SHA384`. Nothing else is needed -
   `WOLFSSL_AES_256` is already set (`settings.h:3229`), and
   `internal.h:814` compiles `BUILD_TLS_AES_256_GCM_SHA384` once both are
   defined.
3. Delete the build cache (or use "Clean" / `arduino-cli cache clean`) and
   rebuild, or the edited config is ignored.

`arduino-cli` can instead inject the define without touching the file:
`arduino-cli compile --build-property
"compiler.cpp.extra_flags=-DWOLFSSL_SHA384" ...`, or the same line in a
`platform.local.txt`. Editing `user_settings.h` is the form wolfSSL
documents. Cost of the change on the no-PSRAM S3: the SHA-512/384 code path
(a few KB of flash, about 150 bytes of RAM per hash context) plus the
AES-256 key schedule (about 64 bytes per context). Small, not zero.

**Enabling hardware crypto acceleration** is likewise not a version or a
toolchain question - it is a *distribution-form* question. wolfSSL's ESP32
hardware-crypto port is an **ESP-IDF component** feature (configured in the
component `user_settings.h` / `idf.py menuconfig`); the `Arduino-wolfSSL`
library does not activate it and wolfSSL documents no supported Arduino path
for it. HW-accelerated wolfSSL crypto on the ESP32-S3 means a **bare ESP-IDF
project**, not an Arduino sketch - true for 5.8.4, 5.9.x, any version. Full
treatment and sources in section 2.6.

### 1.6 Why wolfSSL and not the ESP32 Arduino core's mbedTLS

The Arduino-ESP32 core ships its own bundled mbedTLS (with hardware crypto
compiled in). The project does **not** use it for the DTLS layer, for two
reasons, in order of weight:

1. **Target is DTLS 1.3.** The stack had to speak RFC 9147 on all three
   device types. wolfSSL does; the ESP-IDF/Arduino mbedTLS in play does DTLS
   1.2 (DTLS 1.0/1.1 removed).
2. **The mbedTLS DTLS 1.2 attempt crash-looped on the NMU.** A comparison
   sketch built on the core's mbedTLS reboot-looped on the ESP32-S3 -
   "PANIC or exception" ~13-15 s in, *before* the handshake timeout, so a
   crash not a clean give-up; raising the task stack 16 KB -> 32 KB did not
   fix it (`dtls13_spike/LOGS/OPEN_ISSUE_dtls12_nmu_panic.md`). It was
   **parked, not root-caused** - the log's own analysis is that the fault was
   in how that sketch set mbedTLS up rather than in mbedTLS itself - because
   wolfSSL DTLS 1.3 had **already** completed handshakes reliably on the NMU
   (2871 ms), the AMU (202-257 ms) and the server. wolfSSL was adopted for
   all three.

**For the memo, phrase it as (2) reads here:** an mbedTLS DTLS 1.2 path on
the NMU crash-looped and was not root-caused in the time available, while
wolfSSL was already working end to end at DTLS 1.3 - so wolfSSL was chosen.
Not "mbedTLS is broken on the S3".

### 1.6a What most likely caused the mbedTLS reboot

Not root-caused at the time, so this is a ranked reading of the evidence in
`dtls13_spike/LOGS/OPEN_ISSUE_dtls12_nmu_panic.md` and the two source files
`dtls13_spike/nmu/dtls12_nmu/{dtls12_nmu.ino,omega_dtls.cpp}`, not a proven
fault.

**What the log rules in and out:**

| Observation | What it means |
|---|---|
| Reset reason is `ESP_RST_PANIC`, about 13-15 s after "connecting", before the 15 s handshake timeout | A crash mid-handshake, not a clean timeout |
| Task stack raised from 16 KB to 32 KB with no change; brick3 runs the same transport in a 12 KB task | Plain stack overflow is unlikely, despite the panic banner text |
| Free heap about 199 KB, largest block about 155 KB at the crash point | `mbedtls_ssl_setup()` allocation is not the direct trigger |
| wolfSSL DTLS 1.3 completes a handshake on the same board, certs, WiFi and server (2871 ms) | Chip, certificates, network and server are all good; the fault is in this sketch's mbedTLS path |
| Earliest crashes happened with no server listening, and the crash persisted once the server was up | The fault sits on the retransmission path, not on parsing a server reply |
| 13-15 s matches mbedTLS DTLS retransmit backoff (1 + 2 + 4 + 8 s) | The crash lands around the fourth retransmit round |

**Most probable cause.** A fault in the hand-written transport glue in
`omega_dtls.cpp` (`bioSend` / `bioRecv` over `WiFiUDP`, and the millis-based
retransmit timer) under real packet loss. This is the same class of bug the
DTLS 1.3 trial already hit and documented as "DTLS over a lossy link needs a
socket receive timeout", which passed every loopback and 1500-MTU test and
only failed on real WiFi (`dtls13_spike/LOGS/RESULTS.md`, defect 2).
Specifically, `bioRecv()` calls `udp_.parsePacket()` on every entry and
returns `WANT_READ` whenever no new datagram is queued. When one datagram
carries several handshake fragments, mbedTLS reads the first fragment, calls
`bioRecv` again, and this code discards the unread remainder of that
datagram. The server's fragmented Certificate flight is then never
reassembled, mbedTLS retransmits on a loop, and after a few rounds a
length or offset value mishandled in the Arduino core's older bundled
mbedTLS DTLS-reassembly code corrupts the heap or a return address, which
surfaces as a panic a few operations later. The no-server case crashes too
because retransmitting the ClientHello flight exercises the same timer and
flight-buffer code.

**Contributing factor.** The Arduino core's mbedTLS defaults
`MBEDTLS_SSL_IN_CONTENT_LEN` and `MBEDTLS_SSL_OUT_CONTENT_LEN` to 16384
bytes each. That is about 32 KB of session buffers taken from a heap already
about 60 KB lighter after certificate load, on a chip with no PSRAM. It
narrows the margin rather than being the trigger by itself.

**Ruled out or low priority:** task-stack overflow (tested); anything in the
chip, certificates, network or server (wolfSSL succeeds on the identical
setup); `udp_.begin(WiFi.localIP(), 0)` failing silently (would fault at
once, not at 13-15 s, and is worth only a five-minute check).

**Why it stayed parked.** `OmegaDtls::connect()` returned a bare `false` and
hid which call failed, so pinning the exact line needed per-step return
logging that was never added. wolfSSL DTLS 1.3 was already passing on all
three device types, so the mbedTLS path was dropped instead of debugged.

**Memo sentence:** an mbedTLS DTLS 1.2 client on the NMU crash-looped on
real WiFi, most likely a fault in the datagram transport and retransmission
glue that the in-memory test suite could not exercise, and since wolfSSL
DTLS 1.3 was already completing handshakes on the server, the Raspberry Pi
and the ESP32-S3, the project adopted wolfSSL rather than spend the
remaining time isolating the mbedTLS fault.

---

## 2. The cipher suite: what is proven, and what is not

### 2.1 What the evidence actually shows - including a gap

This is the part to get exactly right, because the gap is findable.

**Proven, repeatedly, on the NMU:** `DTLSv1.3 / TLS_AES_128_GCM_SHA256`.
Printed by the device itself from wolfSSL's own `wolfSSL_get_cipher()`
(`nmu/omega_dtls.cpp:250-255`), captured in at least six independent logs
across three builds and two different physical units:

- `evidence/captures/nmu18_coldboot_serial.txt:15`
- `evidence/captures/nmu18_warmboot_serial.txt:14`
- `evidence/2026-08-25_cold_discovery.md:19`
- `dtls13_spike/LOGS/soak_nmu_serial_20260815.txt` (every handshake, 70+)
- `dtls13_spike/LOGS/nmu_espidf_hw_metrics_20260813.txt:117` (ESP-IDF build)
- `dtls13_spike/LOGS/nmu_dtls13_timed.txt:25` (Arduino build)

**UPDATE 2026-08-29 - now OBSERVED on the wire:
`TLS_AES_256_GCM_SHA384` (`0x1302`).** Read from a ServerHello in a
server-side capture on the minimal bench rig, alongside the NMU's
`TLS_AES_128_GCM_SHA256` (`0x1301`) in the same run.
Evidence: `evidence/2026-08-29_amu_cipher_observed.md`,
`captures/2026-08-29_hs_amu16.pcap`, `captures/2026-08-29_hs_nmu_t1.pcap`.
The deduction below (kept for the record) was confirmed on all four grounds.
The rest of this subsection is the pre-observation history.

---

**Pre-observation history: the cipher suite negotiated on the AMU<->server
sessions had never been successfully recorded.** The spike's Python code
tried to log it (`dtls13_spike/amu/echo_client.py:52-65`,
`server/echo_server.py:56-69`), but the binding returned something the code
could not resolve, so every AMU and server log read:

```
dtls13_spike/LOGS/final_test_amu_20260815.log:7
  HANDSHAKE ok  server=omega-spike-server  DTLSv1.3/unknown  3306.2 ms

dtls13_spike/LOGS/final_test_server_20260815.log:20
  [AMU_T1] HANDSHAKE ok in 3227 ms (DTLSv1.3, unknown)
```

`dtls13_spike/LOGS/RESULTS.md:18` records this honestly at the time: cipher
suite "(not reported)" for both the server and the AMU. The shipped brick4
server and AMU never log the cipher at all (no `cipher()` call anywhere in
`server/listener.py`, `server/session.py`, `amu/dtls_client.py`).

**Corrections this affects (now resolvable with the observed values):**

1. `dtls13_spike/LOGS/SOAK_1HOUR_RESULTS.md:231` reads "DTLS 1.3 negotiated on
   both device types | proven - DTLSv1.3 / TLS_AES_128_GCM_SHA256". Correct
   reading: the protocol *version* (DTLS 1.3) is proven on both;
   `TLS_AES_128_GCM_SHA256` is the **NMU** suite, and the **AMU** suite is
   `TLS_AES_256_GCM_SHA384` (both now observed, 2026-08-29).
2. Write it as two suites, not one: **NMU -> `TLS_AES_128_GCM_SHA256`,
   AMU -> `TLS_AES_256_GCM_SHA384`**, both negotiated, both AEAD, both TLS
   1.3. The heterogeneity is a feature - see the note at the end of this
   subsection.

**The deduction (confirmed 2026-08-29): `TLS_AES_256_GCM_SHA384` for the
AMU<->server sessions.** It was tight on four grounds:

1. **No cipher list is set anywhere.** `server/listener.py:104` and
   `amu/dtls_client.py:153` both build `wolfssl.SSLContext(PROTOCOL_DTLSv1_3)`
   with no `set_ciphers()` call - grep-confirmed across all of `server/` and
   `amu/` in the shipped brick4 tree. So the suite is wolfSSL's own default
   preference, negotiated.
2. **Both ends are the full-featured wolfssl-py build** (section 1.4:
   `--enable-dtls13 --enable-opensslall --enable-opensslextra
   --enable-tlsx`), so SHA-384 and AES-256 are compiled in on both - unlike
   the NMU's trimmed Arduino build where SHA-384 is absent (section 2.2).
   Both therefore offer all three TLS 1.3 suites.
3. **wolfSSL's default TLS 1.3 preference order places
   `TLS_AES_256_GCM_SHA384` first** (ahead of `TLS_AES_128_GCM_SHA256` and
   `TLS_CHACHA20_POLY1305_SHA256`) when all are built, and the server
   selects by its own preference by default. Two full builds with no
   override therefore land on AES-256-GCM-SHA384.
4. **Direct empirical support** (2.2a): a native C client on the dev
   machine - a full build, like the AMU - selected `TLS_AES_256_GCM_SHA384`
   against this same server, while the constrained NMU selected AES-128.
   Same server, the outcome tracked the client build.

All four grounds held: the ServerHello capture (2026-08-29) shows
`cipher_suite = 0x1302` on the AMU session and `0x1301` on the NMU session,
against the same unmodified server, in the same run.

### 2.1a  Two suites, one server - state it as a feature

The fleet does not impose a cipher; it negotiates one, with **no cipher list
configured anywhere** (verified: no `set_ciphers` / `ciphers=` in
`server/listener.py`, `amu/dtls_client.py`, `nmu/omega_dtls.cpp`). A
constrained noise unit and a capable air-quality unit therefore reach
*different* AEAD suites against one server, automatically:

- **NMU -> `TLS_AES_128_GCM_SHA256`** - the RFC 8446 section 9.1
  mandatory-to-implement suite, matched to the ~128-bit security level of
  its P-256 keys. AES-256 there would be cost with no gain (argument 3 of
  section 2.3).
- **AMU -> `TLS_AES_256_GCM_SHA384`** - because on a Raspberry Pi it costs
  nothing.

Why this is a strength, in the accompanying report:

1. **The negotiation is real, not a rubber stamp** - two clients, two
   outcomes, one unmodified server. Adding a device class with different
   capabilities needs no server change.
2. **Each tier is provisioned to its own budget** - same security floor,
   each device paying only what it needs, with zero configuration.
3. **The worst case is bounded for free** - every negotiable TLS 1.3 suite
   is an AEAD at >=128-bit security (RFC 8446 Appendix B.4; the
   downgrade-to-weak attack class of TLS 1.2 has no target). A guarantee can
   be stated about a session that was never captured.
4. **Cross-build interoperability is demonstrated** - wolfSSL 5.8.4 (Arduino,
   trimmed) and wolfSSL 5.9.2.post0 (PyPI, full) interoperate over DTLS 1.3
   because they are protocol-locked by RFC 9147, not version-locked.

Honest framing (consistent with section 2.5): this was not designed as two
suites - it emerged from the NMU's trimmed build lacking SHA-384, was
evaluated after the fact, and kept deliberately. The feature is that the
system does the right thing per device without being told to.

### 2.1b  Key-exchange group: the AMU link is already post-quantum hybrid (observed 2026-08-29)

Read from the same two bench captures with `tshark`
(`evidence/captures/WIRESHARK_FIELD_EXPORTS.txt`), from `key_share` in the
ServerHello and `supported_groups` in the ClientHello. No group is configured
anywhere in the project (no `set_groups` / `curves=` in any endpoint); this
is entirely the library default.

| Path | Offered groups | Negotiated group | Meaning |
|---|---|---|---|
| AMU_16 <-> server | `SecP384r1MLKEM1024`, `SecP256r1MLKEM768`, secp521/384/256/224r1, ffdhe2048 | **`SecP384r1MLKEM1024` (0x11ed)** | NIST P-384 ECDHE **+ ML-KEM-1024** (FIPS 203) hybrid |
| NMU_T1 <-> server | secp521/384/256/224r1 only | **`secp256r1` (0x17)** | classical P-256 ECDHE |

Consequences for the memo, stated plainly:

- **"P-256 ECDHE" is only correct for the NMU.** The AMU<->server session
  establishes its keys with a hybrid post-quantum group by default, because
  wolfSSL 5.9.2 offers and prefers it and the server (same build) accepts it.
  The NMU's Arduino wolfSSL 5.8.4 build does not offer any MLKEM group, so
  that path stays classical.
- **Certificates are unaffected and remain P-256 ECDSA** on both paths:
  `provisioning/omega_pki.py` sets `CURVE = ec.SECP256R1()`, and the AMU
  ClientHello's `signature_algorithms` list includes
  `ecdsa_secp256r1_sha256`. The key-exchange group and the certificate curve
  are independent.
- **Future Work "post-quantum readiness" must be reworded.** Hybrid PQ key
  exchange is not a to-do on the AMU path - it is already running. The real
  future-work items are (a) make it explicit and pinned rather than a silent
  library default, (b) decide whether the constrained NMU build should also
  carry a hybrid group or stay classical with that stated as a deliberate
  limit, and (c) note that the record cipher (AES-128/256-GCM) was never the
  quantum-weak link.
- This was not observed until the 2026-08-29 capture was dissected field by
  field; earlier evidence files that say "P-256 ECDHE" for the AMU are
  imprecise and are corrected by this section.

### 2.2 Why the NMU is capped at AES-128 - traced, not guessed

Three layers of evidence, from strongest to weakest.

**(a) Empirical, from this project's own trial.**
`dtls13_spike/LOGS/RESULTS.md:56-59`, observation 2, written at trial time:

> "The Feather selected `TLS_AES_128_GCM_SHA256`; a native C client on the
> development machine selected `TLS_AES_256_GCM_SHA384` against the **same
> server**. The constrained build offers a lighter default suite."

Same server, two clients, two outcomes. This is a controlled comparison and
it isolates the cause to the **client**, not the server. It is also the
evidence that the server supports AES-256 - so the cap is not the server's.

**(b) Traced through the wolfSSL 5.8.4 source, and confirmed at the
preprocessor** (2026-08-28, on the installed Arduino library):

```
wolfssl/internal.h:814
    #if defined(WOLFSSL_SHA384) && defined(WOLFSSL_AES_256)
        #define BUILD_TLS_AES_256_GCM_SHA384
    #endif
```

The AES-256 suite is compiled only if **both** macros are set.

- `WOLFSSL_AES_256` **is** set - auto-enabled at
  `wolfssl/wolfcrypt/settings.h:3229-3231`. AES-256 itself is present.
- `WOLFSSL_SHA384` is **not** set. It is `#define`d in exactly one file in
  the whole source tree - `wolfssl/options.h:220` - and the Arduino build
  compiles with `-DWOLFSSL_USER_SETTINGS`, which is precisely the mode that
  does **not** include `options.h` (`settings.h:56-63`). Nothing else
  enables it: not `settings.h`, not the ESP32 branch, not the TLS 1.3 branch,
  and not the stock `user_settings.h`.

Confirmed by running the preprocessor against the real build configuration
rather than reading it:

```
$ gcc -I<lib>/src -DWOLFSSL_USER_SETTINGS -E probe.c
    RESULT WOLFSSL_SHA384_IS_OFF
    RESULT AES256_SUITE_NOT_BUILT
```

**(c) Protocol-level consequence.** TLS 1.3 defines exactly five cipher
suites (RFC 8446 Appendix B.4), and **only one contains AES-256**:
`TLS_AES_256_GCM_SHA384`. It is bound to SHA-384 - there is no
"AES-256 + SHA-256" suite. So losing SHA-384 removes the only route to
AES-256, and every remaining suite the device can offer uses AES-128 or
ChaCha20. DTLS 1.3 inherits this list unchanged (RFC 9147: cipher suites are
"same as for TLS 1.3").

**Conclusion, precisely worded:** the NMU is capped at AES-128 because the
wolfSSL Arduino library ships with SHA-384 disabled in its default
`user_settings.h`, and the only AES-256 suite in TLS 1.3 requires SHA-384.
It is a build-configuration default of a third-party embedded library, not a
protocol limitation, not a server limitation, and not a coding error.

### 2.3 The defense: why AES-128-GCM-SHA256 is the right answer anyway

Six arguments, each independently sufficient, ordered strongest first. All
rest on existing evidence or published standards - none requires a new test.

**1. It is the one suite the standard makes mandatory.**
RFC 8446 §9.1: *"A TLS-compliant application MUST implement the
TLS_AES_128_GCM_SHA256 cipher suite and SHOULD implement the
TLS_AES_256_GCM_SHA384 and TLS_CHACHA20_POLY1305_SHA256 cipher suites."*
The fleet runs the **MUST**, not a **SHOULD**. It is the most widely
implemented, most reviewed, most interoperable code path in the protocol.
Choosing it is compliance, not compromise.

**2. In TLS 1.3 a weak suite cannot be negotiated at all - so the
un-instrumented AMU session is still provably strong.**
This turns the gap in 2.1 from a hole into a bounded statement. RFC 8446
Appendix B.4 defines only five suites, and the RFC states: *"The list of
supported symmetric encryption algorithms has been pruned of all algorithms
that are considered legacy. Those that remain are all AEAD algorithms."*
There is no RC4, no 3DES, no CBC, no NULL, no EXPORT grade - the entire class
of downgrade-to-weak attacks that plagued TLS 1.2 has no target. So even
without having logged the AMU's suite, it is certain that whatever was
negotiated is an AEAD construction at >=128-bit security. **The protocol
guarantees the floor; configuration cannot lower it.** That is a stronger
statement than most projects can make about a cipher they *did* log.

**3. 128 bits is the balanced choice, given P-256.**
The certificates and the key exchange are ECDSA and ECDHE on NIST P-256
(`provisioning/omega_pki.py:41-42`), which is a **~128-bit security level**
(NIST SP 800-57 Part 1 Rev. 5, Table 2: a 256-bit elliptic curve provides
128 bits of security strength). A system is bounded by its weakest link.
Pairing AES-256 with P-256 keys would raise the symmetric half to 256 bits
while the asymmetric half stays at 128 - more cost, identical real security
floor. AES-128 with P-256 is the *matched* design. This is the answer that
shows the whole system was reasoned about, not one parameter maximised.

**4. Under quantum assumptions the argument gets stronger, not weaker.**
Grover's algorithm gives at most a quadratic speed-up against symmetric
ciphers, so AES-128 retains ~64 bits of quantum security; Shor's algorithm
breaks P-256 **completely**. Upgrading AES to 256 while keeping P-256 would
harden the half that is not the problem. The honest post-quantum roadmap item
is a hybrid PQ key exchange (e.g. ML-KEM alongside ECDHE), not AES-256 - and
naming that correctly demonstrates threat-model literacy.

**5. The cost is real on this hardware, and the benefit is not - but be
honest that the runtime cost is small.**
The NMU is a salvaged, mains-powered ESP32-S3 with **no PSRAM** whose DTLS
stack already costs ~63.6 KB RAM and 57% of the app partition
(`dtls13_spike/LOGS/RESULTS.md`, footprint table). The NMU ships the
**software-crypto** build (no hardware AES/SHA - that path needs ESP-IDF,
not the Arduino core). Measured software cost on this class of chip
(wolfSSL wolfCrypt benchmark, ESP32 @240 MHz, `fastmath`; the S3's LX7 is
within ~10-20% of the LX6 for scalar software crypto):

| Operation | AES-128 | AES-256 | delta |
|---|---|---|---|
| AES-CBC encrypt (raw round loop) | 1146 KB/s | 1000 KB/s | AES-256 ~13% slower |
| **AES-GCM encrypt** (what DTLS uses) | 331.8 KB/s | 312.2 KB/s | **AES-256 ~6% slower** |
| SHA-256 vs SHA-384/512 | 1747 KB/s | ~1161 KB/s | SHA-384 ~1.5x slower (64-bit words emulated on a 32-bit core) |

The AES-GCM delta is only ~6%, not the ~40% the round count implies,
because GHASH (the Galois-field tag) is a large, **key-size-independent**
share of the software cost. Mapped onto the real workload:

- **Per record:** AES-256-GCM vs AES-128-GCM is ~6% on a ~300 us crypto
  slice -> **~20 us extra per record**, against a measured ~150 ms
  per-record cost dominated by WiFi modem-sleep radio wake
  (`dtls13_spike/LOGS/SESSION_REUSE_AND_HW_ACCEL_RESULTS.md`). That is
  ~0.01% - unmeasurable. SHA-384 does not touch the record path (GCM's tag
  is GHASH, not HMAC).
- **Per handshake:** SHA-384 in the transcript hash + HKDF adds roughly
  ~1 ms to a ~4400 ms handshake (which is dominated by P-256 ECDHE and
  ECDSA bignum arithmetic, not hashing). ~0.02%.
- **Footprint:** AES-256 context +64 B RAM; enabling SHA-384/512 pulls in
  `sha512.c` - a few KB of flash and ~150 B more RAM per hash context. On a
  no-PSRAM chip this is the more tangible cost, though still not decisive.

**Conclusion:** the runtime penalty of AES-256 here is negligible, so this
argument carries less weight than it first appears. It is a supporting
point, not the main one - the main one is argument 3 (P-256 caps the whole
system at ~128-bit, so AES-256's extra key bits are unreachable). The
right framing at a defence is a clean security-matching decision with a
small footprint bonus, not a performance rescue.

**6. It is a configuration knob, not a design lock-in - and that is
demonstrable.**
Adding one line, `#define WOLFSSL_SHA384`, to the Arduino library's
`user_settings.h` compiles the AES-256 suite in, and the server already
supports it (proven empirically in 2.2a). So the system is cipher-agile: the
choice can be changed for a future deployment with a rebuild and reflash, no
protocol or architecture change. Deliberately **not** done here, because of
arguments 3 and 5, and because the practical phase is frozen and an untested
crypto change before a defense is a bad trade.

### 2.4 The two-sentence answer to give a panel

> The fleet's NMU negotiates `TLS_AES_128_GCM_SHA256`, which RFC 8446 §9.1
> makes the mandatory-to-implement suite for TLS/DTLS 1.3, and which matches
> the ~128-bit security level of the P-256 certificates and key exchange - so
> AES-256 would raise cost without raising the system's real security floor.
> The cap itself comes from the wolfSSL Arduino library shipping with SHA-384
> disabled, and since TLS 1.3's only AES-256 suite requires SHA-384, enabling
> it is a one-line build-flag change I chose not to make after the code
> freeze.

### 2.5 If asked "did you choose it, or did it just happen?"

Answer honestly: **it was not chosen, it was negotiated** - no cipher list is
set anywhere in the code base (verified: no `set_cipher`/`ciphers=` call in
`nmu/omega_dtls.cpp`, `server/listener.py`, or `amu/dtls_client.py`). Then
add the substance: the negotiated outcome was **verified** on the NMU across
three builds and two units, the *reason* for it was traced to source, and it
was **evaluated after the fact against the alternative and kept deliberately**
for the reasons in 2.3. Standard practice for TLS 1.3 is precisely to leave
suite selection to negotiation - the protocol was designed so that all
available outcomes are safe (argument 2). Overriding it with a hand-written
cipher list is how misconfigurations get introduced, not avoided.

### 2.6 Hardware crypto acceleration, and why the Arduino build is the right call

Three separate claims, each with an external source, plus this project's own
measurement.

**(a) The NMU's DTLS stack (wolfSSL) does not use the ESP32-S3 crypto
hardware, and turning it on means leaving the Arduino core.** wolfSSL's
Espressif hardware-acceleration port is delivered and configured **as an
ESP-IDF component** - "All settings should be adjusted in the respective
project component `user_settings.h` file"; the port documentation "focuses
exclusively on the ESP-IDF framework and Managed Component distribution"
and gives no Arduino-library configuration path
([wolfSSL wolfCrypt Espressif port README](https://github.com/wolfSSL/wolfssl/blob/master/wolfcrypt/src/port/Espressif/README.md);
[wolfSSL ESP-IDF README](https://github.com/wolfSSL/wolfssl/blob/master/IDE/Espressif/ESP-IDF/README.md)).
The wolfSSL **Arduino library** this project builds against compiles with
`-DWOLFSSL_USER_SETTINGS` and a `user_settings.h` that does not define
`WOLFSSL_ESP32_CRYPT`; nothing in the Arduino build wires in the ESP-IDF
crypto driver headers the port needs (traced through the installed 5.8.4
source and the preprocessor - section 2.2). This project demonstrated the
switch empirically: the software-crypto build is the Arduino sketch; the
hardware-crypto build is a separate bare-ESP-IDF project, and its serial log
carries the runtime line `esp_mp_mul HW acceleration enabled`
(`dtls13_spike/LOGS/nmu_espidf_hw_metrics_20260813.txt`,
`SESSION_REUSE_AND_HW_ACCEL_RESULTS.md`).

Not the same as "Arduino cannot do ESP32 hardware crypto" - the
Arduino-ESP32 core's *bundled mbedTLS* has `CONFIG_MBEDTLS_HARDWARE_AES/_SHA`
compiled in, so code that calls the core's mbedTLS gets acceleration. The
point is narrower and exact: **this project's DTLS layer is wolfSSL, and
wolfSSL's ESP32 hardware port is an ESP-IDF-component feature that the
Arduino distribution does not carry.**

**(b) Even with the hardware on, AES gains nothing at full clock and GCM
only partly benefits.** Espressif's own ESP-IDF configuration guide states,
verbatim, for `CONFIG_MBEDTLS_HARDWARE_AES`: *"Note that if the ESP32 CPU is
running at 240MHz, hardware AES does not offer any speed boost over software
AES."* And for `CONFIG_MBEDTLS_HARDWARE_GCM`: *"Enable partially hardware
accelerated GCM. GHASH calculation is still done in software."*
([esp-idf/components/mbedtls/Kconfig](https://github.com/espressif/esp-idf/blob/master/components/mbedtls/Kconfig)).
The SHA accelerator is real and does cover the SHA-512 family on the S3
([ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf),
SHA Accelerator chapter: SHA-1/224/256/384/512 and the SHA-512/t variants) -
which is exactly why the AES-128 cap in section 2.2 is a *build-config*
default, not a hardware limit.

**(c) This project measured the whole hardware build and it barely moved.**
Controlled at 240 MHz, hardware vs software
(`dtls13_spike/LOGS/SESSION_REUSE_AND_HW_ACCEL_RESULTS.md`):

| | Software (Arduino) | Hardware (ESP-IDF) | delta |
|---|---|---|---|
| Handshake | 4830 ms | 4374 ms | **-9.4%** (server clock: -11.7%) |
| Per-record avg | 145 ms | 146 ms | **+1 ms, none** |

The handshake is once per multi-hour session; the per-record cost - paid on
every detected sound, and the number that decides "close to live" - does not
move, because it is dominated by WiFi modem-sleep radio-wake latency, not
crypto. Corroborating public benchmark (wolfCrypt, ESP32 @240 MHz): AES-256-GCM
software 312.2 KB/s vs hardware 344.5 KB/s (~10%), while SHA-256 goes
1.747 -> 15.234 MB/s and SHA-512 1.161 -> 17.512 MB/s - i.e. the accelerator
is transformative for hashing and nearly irrelevant for the AEAD record
layer at full clock
([wolfSSL wolfCrypt Espressif port README](https://github.com/wolfSSL/wolfssl/blob/master/wolfcrypt/src/port/Espressif/README.md)).

**The engineering conclusion.** Moving the NMU to a bare ESP-IDF build to
enable hardware crypto costs the simpler Arduino toolchain (the
flashing/serial workflow already in use for every device) and buys ~9% on a
once-per-session event and nothing on the recurring cost. It is a poor
trade for this system, and it is stated as such rather than treated as an
unrealised win. (Full ESP-IDF migration remains a Future Work item -
`FUTURE_WORK.md` section 11 - for the ULP and deep-sleep features, not for
the crypto.)

---

## 3. Corrections to make in the repo before submission

| File | Line | Issue |
|---|---|---|
| `dtls13_spike/LOGS/SOAK_1HOUR_RESULTS.md` | 231 | Claims the cipher suite is proven "on both device types". Version is; suite is NMU-only. |
| `CLAUDE.md` | 313 | Describes brick3 as "DTLS 1.2, ECDHE-ECDSA-AES256-GCM-SHA384" - correct for brick3, but the file's status map is stale and reads as current. |
| `deploy/*/requirements*.txt` | - | DONE 2026-08-29: pinned to `wolfssl==5.9.2.post0` (was `>=5.9.0`). |

`evidence/attack_suite.md:13` shows `ECDHE-ECDSA-AES256-GCM-SHA384`, but that
file already discloses on line 9 that its channel is the DTLS 1.2 gate and
quotes the real hardware suite. It is defensible as written; no change needed.

---

## Sources

Repository evidence is cited inline by file and line. External sources:

- RFC 8446, *The Transport Layer Security (TLS) Protocol Version 1.3*, §9.1
  (mandatory-to-implement suites) and Appendix B.4 (the five suites) -
  https://www.rfc-editor.org/rfc/rfc8446.html
- RFC 9147, *The Datagram Transport Layer Security (DTLS) Protocol Version
  1.3* (cipher suites "same as for TLS 1.3"; `"dtls13"` key-schedule label) -
  https://www.rfc-editor.org/rfc/rfc9147.html
- NIST SP 800-57 Part 1 Rev. 5, Table 2 (security strengths; 256-bit elliptic
  curve = 128-bit security strength) -
  https://csrc.nist.gov/pubs/sp/800/57/pt1/r5/final
- PyPI release metadata for the `wolfssl` package (latest 5.9.2.post0,
  2026-07-16; source-only distributions) - https://pypi.org/project/wolfssl/
- wolfssl-py bundled-library build flags, `wolfssl/_build_ffi.py` -
  https://github.com/wolfSSL/wolfssl-py
- wolfSSL 5.8.4 Arduino library source as installed on this PC
  (`internal.h:814`, `settings.h:3229`, `options.h:220`)

External sources for section 2.6 (hardware acceleration):

- FIPS 197, *Advanced Encryption Standard (AES)*, §5 - Nr = 10 rounds for a
  128-bit key, 14 for 256-bit; and the key-expansion sizes -
  https://nvlpubs.nist.gov/nistpubs/FIPS/NIST.FIPS.197-upd1.pdf
- FIPS 180-4, *Secure Hash Standard* - SHA-256 is defined on 32-bit words,
  SHA-384/512 on 64-bit words (the reason SHA-384 is slower on a 32-bit
  core) - https://nvlpubs.nist.gov/nistpubs/FIPS/NIST.FIPS.180-4.pdf
- wolfSSL wolfCrypt Espressif port README - HW acceleration is an ESP-IDF
  component feature configured in the component `user_settings.h`; no
  Arduino path documented; ESP32 @240 MHz benchmark table (AES-256-GCM sw
  312.2 / hw 344.5 KB/s; SHA-256 sw 1.747 / hw 15.234 MB/s; SHA-512 sw
  1.161 / hw 17.512 MB/s) -
  https://github.com/wolfSSL/wolfssl/blob/master/wolfcrypt/src/port/Espressif/README.md
- wolfSSL ESP-IDF integration README -
  https://github.com/wolfSSL/wolfssl/blob/master/IDE/Espressif/ESP-IDF/README.md
- Espressif ESP-IDF, `components/mbedtls/Kconfig` - verbatim: hardware AES
  "does not offer any speed boost over software AES" at 240 MHz; hardware
  GCM leaves "GHASH calculation ... done in software"; hardware SHA covers
  "SHA1, SHA256, SHA384 & SHA512" -
  https://github.com/espressif/esp-idf/blob/master/components/mbedtls/Kconfig
- ESP32-S3 Technical Reference Manual, SHA Accelerator chapter - the S3 SHA
  block supports SHA-1/224/256/384/512 and the SHA-512/t variants -
  https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
- wolfCrypt benchmark methodology and full ESP32 result set -
  https://www.wolfssl.com/docs/benchmarks/ and
  https://www.wolfssl.com/docs/espressif/
- Arduino-ESP32 core bundles ESP-IDF's mbedTLS with hardware crypto compiled
  in (so "Arduino cannot do ESP32 HW crypto" is false in general; the
  narrow claim is that wolfSSL's HW port is not in the Arduino distribution)
  - https://docs.espressif.com/projects/arduino-esp32/en/latest/ and
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/protocols/mbedtls.html
