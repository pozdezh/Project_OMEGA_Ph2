# AMU<->server cipher suite: observed on the wire (2026-08-29)

Closes the one open item in `crypto_versions_and_cipher_choice.md` section
2.1. The AMU<->server DTLS 1.3 cipher suite - previously deduced, never
captured - is now read directly from a ServerHello.

## Rig

Minimal bench setup: server (Intel NUC7PJYH, `192.168.0.112`), TP-Link
TL-WR841N router, one AMU (AMU_16, `192.168.0.118`), one NMU (NMU_T1,
`192.168.0.100`), all headless over SSH. No code change - the DTLS 1.3
ServerHello carries the negotiated suite in cleartext.

## Method

Server-side capture while both devices were forced to re-handshake:

```
sudo timeout 120 tcpdump -i any -s0 -Z root -w /tmp/hs_<ts>.pcap 'udp port 11400'
```

- AMU: Raspberry Pi power-cycled -> cold boot -> handshake at 02:12:15
  (server log: `[AMU_16] session up`).
- NMU: USB replugged -> cold boot -> handshake at 02:13:48 (server log:
  `[NMU_T1] session up`, new session id `1308927042`).

Two pcaps captured across the two windows:
`captures/2026-08-29_hs_amu16.pcap` (40 pkt) and
`captures/2026-08-29_hs_nmu_t1.pcap` (12 pkt).

The `cipher_suite` field was read by walking the packet at fixed offsets
(pcap rec hdr -> SLL2 20B -> IPv4 -> UDP -> DTLS handshake record
content_type 22 -> ServerHello -> 2B legacy_version + 32B random + 1B
session_id_len + session_id -> 2B cipher_suite). Wireshark shows the same
value under `DTLSv1.3 Record -> Server Hello -> Cipher Suite`.

## Result

```
AMU_16 (192.168.0.118) <-> server (192.168.0.112)
  ServerHello  cipher_suite = 0x1302  TLS_AES_256_GCM_SHA384   (seen 2x)

NMU_T1 (192.168.0.100) <-> server (192.168.0.112)
  ServerHello  cipher_suite = 0x1301  TLS_AES_128_GCM_SHA256   (seen 3x)
```

(The parser also emitted a spurious `0x0b00` for one packet - a Certificate
handshake message, msg_type 11, mis-framed as a ServerHello by the
byte-walker. Ignored; the real ServerHellos are unambiguous.)

## Reading the DTLS version on the wire - do not be fooled by "DTLSv1.2"

Both handshakes are **DTLS 1.3**. Wireshark's protocol column can still show
`DTLSv1.2` for one of them, and that is expected, not a downgrade.

RFC 9147 freezes two fields at the DTLS 1.2 code (`0xfefd`) for middlebox
compatibility: the record-layer `legacy_record_version` and the ServerHello
`legacy_version`. The **negotiated** version is carried only in the
`supported_versions` extension. Read directly from these two captures with
`tshark`:

```
                         record.version   handshake supported_version
  NMU_T1 ClientHello      0xfefd           ->        0xfefc  (DTLS 1.3)
  NMU_T1 ServerHello      0xfefd           ->        0xfefc  (DTLS 1.3)
  AMU_16 ClientHello      0xfefd           ->        0xfefc  (DTLS 1.3)
  AMU_16 ServerHello      0xfefd           ->        0xfefc  (DTLS 1.3)
```

`0xfefc` is DTLS 1.3 (version codes: 1.0 `0xfeff`, 1.2 `0xfefd`, 1.3
`0xfefc`). Wireshark labels the AMU flow `DTLSv1.2` in the summary column
only because its capture starts mid-conversation (first frame is 25) with a
fragmented ServerHello it does not fully stitch, so it falls back to the
record `legacy_version`. In the packet detail, expand
`Server Hello -> Extensions -> supported_versions` and it reads
`DTLS 1.3 (0xfefc)` for both. tshark field:
`dtls.handshake.extensions.supported_version`.

**Wireshark's own dissector says so.** On the AMU ServerHello (`-V`):

```
Handshake Type: Server Hello (2)
Version: DTLS 1.2 (0xfefd)
  [Expert Info (Chat/Deprecated): This legacy_version field MUST be ignored.
   The supported_versions extension is present and MUST be used instead.]
Cipher Suite: TLS_AES_256_GCM_SHA384 (0x1302)
Extension: supported_versions (len=2) DTLS 1.3
  Supported Version: DTLS 1.3 (0xfefc)
```

Two further proofs in the same capture:

- `dtls.handshake.extensions.supported_version == 0xfefd` matches **zero
  packets** - DTLS 1.2 is never selected anywhere in the AMU flow.
- Frame 27 is a **HelloRetryRequest** (its Random is the fixed
  `cf21ad74e5...c8a8339c` "HelloRetryRequest magic"). HRR exists only in
  TLS 1.3 / DTLS 1.3; DTLS 1.2 has no such message (it uses
  HelloVerifyRequest). The NMU flow does the same (its frame 3).

### Both DTLS endpoints are pinned to 1.3 in the shipped code

Not a negotiation that *happened* to land on 1.3 - every context is
constructed 1.3-only:

| Endpoint | File | Constant |
|---|---|---|
| Server telemetry listener | `server/listener.py:104` | `wolfssl.PROTOCOL_DTLSv1_3, server_side=True` |
| AMU telemetry client | `amu/dtls_client.py:153` | `wolfssl.PROTOCOL_DTLSv1_3, server_side=False` |
| AMU live-query server (MCP) | `amu/live_server.py:137` | `wolfssl.PROTOCOL_DTLSv1_3, server_side=True` |
| Server -> device live client | `server/device_live.py:47` | `wolfssl.PROTOCOL_DTLSv1_3, server_side=False` |
| NMU | `nmu/omega_dtls.*` | wolfSSL C, `WOLFSSL_DTLS13` (default on ESP32) |

`PROTOCOL_DTLSv1_3` binds wolfSSL's version-specific
`wolfDTLSv1_3_*_method()`, which does not downgrade. A peer that could only
do DTLS 1.2 would fail the handshake, not fall back. So any session that
completes at all is DTLS 1.3, on both the AMU<->server and the NMU<->server
paths. The device-side serial logs (`wolfSSL_get_version()` /
`get_cipher()`) and the 11 Aug hardware trial (`../../dtls13_spike/LOGS/RESULTS.md`)
independently report `DTLSv1.3` for both device types.

The only DTLS 1.2 anywhere in the project is the in-memory gate / attack
suite on the Windows dev machine, whose OpenSSL predates DTLS 1.3. That is
the test harness, explicitly flagged as such; it is not the deployed
system.

### The ClientHello cipher lists corroborate the SHA-384 trace

The offered-suites lists differ exactly as
`crypto_versions_and_cipher_choice.md` section 2.2 predicts:

```
  NMU_T1 ClientHello offers: 0x1301, 0xc02b, 0xc02f, 0xc027, 0xc023, 0xc00a, ...
                             (has TLS_AES_128_GCM_SHA256; NO 0x1302, NO 0x1303)
  AMU_16 ClientHello offers: 0x1302, 0x1301, 0x1303, 0xc02c, 0xc02b, 0xc030, ...
                             (all three TLS 1.3 suites)
```

The NMU's trimmed Arduino wolfSSL build cannot even *offer* `0x1302`
(`TLS_AES_256_GCM_SHA384`) because `WOLFSSL_SHA384` is not compiled in. The
server then has only `0x1301` in common with it. The AMU's full PyPI build
offers everything and the server picks `0x1302`. Same server, no cipher list
configured, two outcomes - visible in the ClientHello, not just inferred.

## What this confirms

- **AMU<->server negotiates `TLS_AES_256_GCM_SHA384`** - exactly as deduced
  in `crypto_versions_and_cipher_choice.md` section 2.1 on four grounds (no
  cipher list set; both full wolfssl-py builds; wolfSSL default TLS 1.3
  preference; the dev-machine trial). Deduction -> observation.
- **NMU<->server negotiates `TLS_AES_128_GCM_SHA256`** - matches the 6+
  prior serial logs from `wolfSSL_get_cipher()` on the device itself.
- All AMU units run the identical `wolfssl` PyPI build (5.9.2.post0, full
  `--enable-opensslall`), so the PSK-ladder rung D session (AMU_T1,
  `2026-08-27_psk_ladder.md`) used this same suite:
  **`TLS_AES_256_GCM_SHA384`**. The ladder's entropy (7.91 bits/byte) and
  block-repetition (0/129) results for rung D were therefore measured on an
  **AES-256-GCM** session - and would be identical on the NMU's AES-128-GCM,
  because those properties come from GCM's fresh per-record nonce, not the
  key size.

## The heterogeneity is a feature, not an inconsistency

The two device types negotiate two different AEAD suites against the same
server, with **no cipher list configured anywhere** (verified: no
`set_ciphers` / `ciphers=` in `server/listener.py`, `amu/dtls_client.py`,
`nmu/omega_dtls.cpp`). This is worth stating positively:

1. **The negotiation is real, not a rubber stamp.** A constrained client and
   a capable client reach *different* results against one unmodified server,
   automatically. Adding a new device class with different capabilities
   needs no server change.
2. **Each tier is provisioned to its own budget.** The NMU (salvaged, no
   PSRAM, 32-bit Xtensa) takes the RFC 8446 section 9.1 *mandatory* suite,
   AES-128-GCM-SHA256, matched to the ~128-bit security level of its P-256
   keys - AES-256 there would be cost with no gain. The AMU (Raspberry Pi)
   takes AES-256-GCM-SHA384 because it costs it nothing. Same security
   floor, each device paying only what it needs.
3. **The worst case is bounded for free.** Every negotiable TLS 1.3 suite is
   an AEAD at >=128-bit security (RFC 8446 Appendix B.4 - no CBC, no RC4, no
   export grade). So a guarantee can be stated about a session that was
   never captured: whatever any device negotiates, it is a strong AEAD.
4. **Cross-build interoperability is demonstrated.** wolfSSL 5.8.4 (Arduino,
   trimmed) on the NMU and wolfSSL 5.9.2.post0 (PyPI, full) on the
   AMU/server - different distributions, versions and feature sets -
   interoperate over DTLS 1.3 because they are protocol-locked by RFC 9147,
   not version-locked. Two independent builds agreeing on a handshake is a
   stronger correctness signal than one build talking to itself.

**Honest framing** (per `crypto_versions_and_cipher_choice.md` section 2.5):
this was not designed as two suites - it emerged from the NMU's trimmed
wolfSSL build lacking SHA-384. It was then evaluated after the fact against
the alternative and kept deliberately. The feature is that the system does
the right thing per device without being told to.

## Update applied

`crypto_versions_and_cipher_choice.md` section 2.1: "deduced" -> "observed"
for the AMU suite, citing this file and the two pcaps.
