# Prior-work reading: Marina's TFG and Constantin's Phase-1 TFG

Read 2026-08-26 from the two supplied PDFs (text extracted with pypdf, not
image analysis). This file is the durable summary for the memo and defense -
what each did, how it was tested, and where Omega (Phase 2) stands against
both. Written to be reused, not re-derived.

---

## 1. Marina Rull Ventura - "Implementació d'una capa de seguretat en
## dispositius EmotiBit" (110 pp, Telecos, 2025, same tutors: Batista +
## Martínez-Ballesté)

### What it is
A **security layer bolted onto an existing IoT device** (EmotiBit, an
open-source physiological wearable on ESP32) and its receiver software
(Oscilloscope). The original protocol sent everything in **cleartext over
UDP** with no authentication. Marina added confidentiality, integrity and
authenticity without breaking the original architecture.

### The cryptographic design (this is the "Marina style" baseline)
- **Pre-shared keys (PSK)** - symmetric. Each EmotiBit has its own key; the
  Oscilloscope has its own key (skOS). No key exchange over the air.
- **AES-128, ECB mode**, plus **HMAC-SHA-256** for integrity/authenticity.
  She explicitly chose ECB for simplicity/efficiency on the constrained
  device and compared modes in Annex A.
- **Keys managed centrally** in a **MySQL database + REST API**; the device
  reads its key from a **file on its SD card**.
- **Replay protection** via monotonic **packet numbers** (reject number <=
  last seen).
- **Order of operations, defended explicitly**: on the data channel, verify
  the short HMAC (32 B) BEFORE decrypting the long payload (~500 B) - cheaper
  to reject a forgery early. On the announce channel, decrypt first (messages
  are short ~40 B and identity is not yet known), then verify. This is a
  genuinely good piece of reasoning and worth citing as prior art.

### How she tested it (her evaluation chapter - the part to emulate)
Three named attack scenarios, each shown **with security OFF then ON**, with
**Wireshark captures as evidence**:
1. **Passive eavesdropping** - OFF: physiological data read directly off the
   wire. ON: ciphertext only.
2. **Replay** (single + continuous) - OFF: the graph is polluted by repeated
   values. ON: rejected by packet-number control.
3. **Invalid-traffic injection** (short packets, literal text "ATAC", and a
   subtle data-tamper multiplying PPG:Red by 1.01) - OFF: tamper accepted and
   plotted as if real. ON: rejected (too short to hold an HMAC, or HMAC
   mismatch).

Plus **two emulation harnesses she wrote herself**, which is the strongest
methodological move in the whole document:
- **A fake EmotiBit** (Python script) sending HE/EC/PN/PO and data packets.
  With correct keys+ID it is accepted; with a **fake ID**, **wrong key for a
  valid ID**, or **wrong Oscilloscope PSK** it is rejected, and the console
  prints the exact reason. Real ID MD-V5-0000142, simulated MD-V5-0000158.
- **A malicious Oscilloscope** (the real software is a free download, so this
  is a realistic threat) - OFF: it discovers the device, connects, reads all
  data, and even monopolises the single-connection EmotiBit (a
  denial-of-service). ON: its HH has no valid PSK, so it is ignored.

### What she states honestly as limits (her Future Work, 7.1)
- **ECB is weak** - it does not hide repeated patterns; the natural next step
  is **AES-GCM** (encryption + authentication in one pass). She flags it must
  be re-validated for real-time fluidity on the device. **This is exactly the
  step Omega's brick1 took, and then brick3/4 went past.**
- Move the key database **off the Oscilloscope machine** (needs auth'd,
  restricted access).
- Support **multiple simultaneous devices**; explore other IoT (drones).
- Honestly scopes OUT lower-layer attacks (ARP-spoofing that just breaks the
  channel) - an application-layer protocol cannot see them.

---

## 2. Constantin Shoot Vayevoda - "Development of a Contextual Sensing Kit..."
## (61 pp, Biomedical Eng, 2025) - Project Delta PHASE 1, the base of Omega

### What it is
Two modular IoT sensing kits, validated in a smart-classroom framework:
- **AirQ kit** (Raspberry Pi 4, Python): Sensirion **SCD30** (CO2),
  Plantower **PMS5003** (particulates), **DHT22** (temp/hum), light; local
  **SQLite** logging; Wi-Fi time sync. ~50 MB/month at 1-min cadence.
- **Noise kit** (ESP32, C/C++, ESP-IDF/RTOS): **44.1 kHz** audio, **STFT**
  for voice-band spectral activity, local **.BIN** frames (512 B-aligned
  float32 freq/mag pairs), **peak data over BLE on button press**. PSRAM
  double-buffering; ~0.9 GiB/day; ~24 h battery.

### The scientific result (worth one sentence in the memo)
Three deployment days. Minute-level **forward-lag** analysis (Noise leading
Air), autocorrelation-corrected, BH-FDR across lags. Finding: **speech-like
activity is associated with the rate-of-change of CO2 (dCO2), concentrating
near +4 minutes**, Pearson **r ≈ 0.11-0.19** - modest, reported as
association not causation. i.e. **noise can act as a proxy for indoor CO2
change** - which is the whole justification for building a fleet.

### Its own Future Work section IS the Omega spec
Phase 1 literally names, as Phase-2 work: self-organisation / peer discovery /
liveness / role election; a lean versioned data plane; ephemeral ring buffers
with event-triggered persistence; adaptive duty cycle; **signed parameter
packs; per-unit remote diagnostics; robust time handling; encryption in
transit and at rest; identifiers scoped to a local collector**. Omega
delivered on essentially all of the operations/privacy line and the
coordination line.

---

## 3. How Omega (Phase 2) compares - the honest verdict

The user's real question: *am I overcomplicating this, or is it genuinely more
sophisticated?* Straight answer, in both directions:

### Where Omega is unambiguously beyond Marina's project
- **True asymmetric mutual auth (DTLS 1.3, RFC 9147)** with a real handshake,
  per-device certificates and forward secrecy - vs. her **pre-shared
  symmetric keys**. Her own Future Work names AES-GCM as the aspiration;
  Omega's brick1 was that (master-derived AES-GCM), and brick3/4 went a whole
  category further to certificate PKI. **This is the single biggest gap.**
- **A certificate authority + provisioning chain** (issue, adopt, revoke,
  identity-guard binding a name to its first cert) vs. a **key file on an SD
  card and a MySQL row**.
- **Scale and autonomy as first-class**: 16 live units, mDNS discovery,
  learned addresses, a recovery ladder, self-healing after total
  disconnection, a one-command replication pack, keys-only SSH fleet
  management. Marina's is one device + one receiver, single connection.
- **Live cryptographic attack suite in the gate** (7/7 defeated with byte
  evidence) that runs every build - vs. her manual, one-off Wireshark
  captures.

### Where Marina's project is stronger, and Omega should BORROW
- **The security-OFF-vs-ON framing.** Her evaluation is persuasive precisely
  because every attack is shown breaking the insecure system first, then
  failing against the secure one. Omega's attack suite proves the ON side;
  it should also **capture the OFF side** (brick1-style or cleartext) for the
  same side-by-side. This is the point of the user's planned "Marina style"
  2-unit symmetric experiment - do it, and film both halves.
- **Self-written emulators as a test method.** Her fake-device and
  fake-receiver scripts, each printing the exact rejection reason, are a
  clean, legible way to demonstrate identity/integrity checks. Omega has the
  equivalent internally (SimLab, cross-device spoof) but should surface a
  **human-readable "why it was rejected" line** in the field evidence, the
  way she does.
- **Wireshark as the evidence medium.** For the defense, a real packet
  capture showing ciphertext where cleartext used to be is more visceral than
  a passing test. Plan a capture on the live fleet.
- **Mode-comparison annex (her Annex A on AES modes).** A compact appendix
  comparing what each brick's crypto actually provides (ECB vs GCM vs DTLS
  1.3 AEAD) would strengthen the memo and directly answers "why not just do
  what Marina did".

### The fair framing for the defense
Marina secured **one existing device**; the project is a **model piece of
scoped, well-tested security engineering** on a fixed target. Omega secures
**a scalable, self-organising fleet from scratch**, and its Phase-1 base
(Constantin's own TFG) already did the sensing science and *predicted* this
exact Phase-2 scope. They are not the same weight class - **not because
Marina's is weak, but because Omega's scope (autonomy + scalability + PKI) is
a superset**. The burnout is justified; the work is genuinely broader. Say
that plainly, and use her testing rigor to make Omega's evaluation chapter as
convincing as hers.

---

## 4. Concrete things to copy into the memo / evaluation plan
1. Every attack demo as **OFF then ON**, side by side (drives the "Marina
   style" 2-unit symmetric experiment the user planned).
2. A **Wireshark capture** of live fleet traffic: cleartext-era vs
   DTLS-encrypted, same payload.
3. A **human-readable rejection reason** in field logs for each defeated
   attack (spoof, replay, tamper, revoked).
4. A short **crypto-modes appendix**: PSK+AES-ECB+HMAC (Marina) -> AES-GCM
   (brick1) -> DTLS-style symmetric (brick2) -> true DTLS 1.3 PKI
   (brick3/4), what each does and does not guarantee.
5. Cite Phase-1's **+4 min noise->dCO2 proxy** result as the scientific
   motivation for the fleet.
6. Borrow her **explicit "order of verification" reasoning** (verify-then-
   decrypt vs decrypt-then-verify) as a design-decision worth defending -
   Omega's DTLS does this inside the library, but naming it shows command of
   the tradeoff.
