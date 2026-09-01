# Project OMEGA, Phase 2

**A fleet of salvaged-hardware environmental sensors that talks to its server
over mutually authenticated DTLS 1.3, recovers from faults without anyone
visiting the building, and can be operated by an AI assistant that is
structurally unable to do anything it was not given a tool for.**

This is the source repository for the second-phase final degree project
(Treball de Fi de Grau, Universitat Rovira i Virgili) titled *Securing a
Distributed, Self-Organising IoT Sensing Fleet: Mutual Certificate-Based
DTLS 1.3, Revocation, and Autonomous Recovery*. Every result reported in that
document has its evidence file here.

---

## In a nutshell

Two kinds of sensing unit report to one server:

- **NMU**, a noise monitoring unit on an **ESP32-S3** microcontroller, in
  C++ on FreeRTOS, with sampling and networking pinned to separate cores.
- **AMU**, an air-quality monitoring unit on a **Raspberry Pi 4B**, in
  Python, measuring CO2, particulates, temperature and humidity.
- **Server**, an Ubuntu machine running the DTLS listener, a SQLite store, a
  web dashboard, and a Model Context Protocol server.

In the first phase these units sent **plaintext JSON over UDP with no
authentication**: anyone on the network could read every reading and inject
convincing fake ones. This phase replaces that transport and builds the
operational layer a real deployment needs around it.

**What changed:**

| | Before (Phase 1) | Now (Phase 2) |
|---|---|---|
| Transport | Plaintext UDP | Mutually authenticated **DTLS 1.3** (RFC 9147) |
| Identity | A name in the payload | A **CA-signed certificate** per unit |
| Key exposure | n/a | Fresh key per unit; forward secrecy per session |
| Retiring a unit | Not possible | **Revoked from a dashboard**, refused on its next record |
| Finding the server | IP compiled into firmware | **mDNS service discovery**, cached, with fallbacks |
| Network or power loss | Manual intervention | **Autonomous recovery ladder**, buffered and replayed |
| Operations | Read the dashboard | Dashboard **plus a closed-tool AI operator layer** |

**Scale reached:** 22 units of each type were built by hand; 16 were
provisioned and running during the measurement period; sustained continuous
operation was evidenced at two units. Those boundaries are stated plainly
rather than rounded up, here and in the memo.

---

## How the security works, briefly

Every unit and the server hold a certificate signed by one offline
certificate authority. At each handshake both sides prove possession of their
private key and check the other's signature against that authority, so
**being on the Wi-Fi is not the same as being in the system**. Session keys
come from an ephemeral Diffie-Hellman exchange and are discarded afterwards,
so a capture recorded today stays unreadable even if a unit's long-term key
is stolen later.

DTLS **1.3** specifically, not 1.2, because 1.3 derives its handshake keys
before either certificate is sent. The certificate exchange is therefore
itself encrypted, and a passive observer cannot see which named device is
connecting.

Revocation does not use a certificate revocation list. A revoked unit's
certificate stays cryptographically valid, so its handshake still succeeds;
what stops it is a **name check against an allow-list on every single
record**, which is why a revoke takes effect in seconds rather than at the
unit's next scheduled re-handshake.

---

## Repository map

| Path | What is in it |
|---|---|
| `nmu/` | ESP32-S3 noise-unit firmware (C++, FreeRTOS, wolfSSL) |
| `amu/` | Raspberry Pi air-quality unit (Python) |
| `server/` | DTLS listener, session handling, discovery, Flask dashboard and API, MCP server, retention and daily-statistics jobs |
| `provisioning/` | Certificate authority and per-unit provisioning tools, including the noise-unit factory |
| `deploy/` | One-command installers and the plain-language manuals |
| `simlab/` | Verification gate and the attack suite |
| `evidence/` | One file per experiment, plus the packet captures |
| `diagrams/` | Figure generators used by the memo |
| `reference/` | `brick1_omega_crypto.py`, the frozen AES-GCM module reproduced as rung C of the four-rung comparison |

Start with **`START_HERE.md`** to rebuild the system, and
**`REPLICATION_AUDIT.md`** for an honest account of what that procedure has
and has not been observed doing end to end.

---

## Reproducing the evidence

`evidence/` holds one dated file per experiment. Each records the claim under
test, the exact commands run, the pass and fail criteria **fixed before the
run**, and the raw artefact. The evaluation standard throughout is that *a
claim is only evidence when it is observed from outside the system that makes
it*: a packet capture, a device serial log, or a database query, never a
passing self-test alone.

Packet captures are in `evidence/captures/`. They were taken on the server
with `tcpdump -i any`, which is why every one carries a Linux cooked-mode
(SLL2) link header.

One capture deliberately contains readable telemetry.
`2026-08-27_psk_ladder.pcap` transmits one unit's live readings four ways
at once, and its first rung is unprotected plaintext on purpose, because
demonstrating a defence means showing the undefended case beside it. The data
in it is environmental readings from the author's own test unit.

---

## What is deliberately not here

**No private key of any kind.** Not the certificate authority's key, not any
unit's key, not the server or operator keys. Not Wi-Fi credentials. Every one
of these is generated during provisioning and stays on the machine that made
it; `.example` templates are committed in their place.

This means you cannot clone this repository and impersonate a unit. It also
means a working deployment requires running the provisioning tools yourself,
which is what `START_HERE.md` walks through.

---

## Related repositories

- **Phase 1**, the sensing platform this builds on:
  [Project_DELTA_Ph1](https://github.com/pozdezh/Project_DELTA_Ph1)

## Licence

MIT, with one exception: the air-quality-unit enclosure derives from a
third-party design by AlfaChris, included with that author's explicit
permission and **not** covered by the MIT licence. See `LICENSE`.
