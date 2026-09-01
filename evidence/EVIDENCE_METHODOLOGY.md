# Evidence methodology - what counts as proof in this project, and why

Written so this section can be lifted close to verbatim into the memo's
evaluation-methodology chapter. It explains two things: what SimLab is and
what it does and does not prove, and the rule that decides whether a claim
in this project may be called "proven."

---

## Two different kinds of test, not one

**1. SimLab (`simlab/`), run by `simlab/run_gate.py`.**

An analogy: this is a flight simulator for the software, not a flight. It
runs the REAL server code and REAL device-side code against each other
in-memory on a development PC - no physical ESP32, no physical Raspberry Pi,
no real network. A simulated "device" and the real server exchange real
protocol messages over a loopback (self-to-self) network channel. This is
fast (the whole suite runs in seconds), repeatable, and catches logic bugs
long before hardware is involved.

`run_gate.py` runs 20+ such test groups in sequence (handshake proof, attack
suite, discovery, revocation, buffer/recovery logic, installer payload
sync...) and only prints `GATE: PASS` if every one passes. It is the
mandatory step before any change is handed off for deployment (see
`CLAUDE.md`'s Verification gate section).

**What SimLab does NOT prove**, stated by the gate itself every time it
passes (`run_gate.py`'s own printed SCOPE block, not an afterthought added
for the memo):

- It runs in memory, on this Windows PC - not over a real network. Green
  means the logic is sound, not that it works on the desk.
- The loopback channel speaks DTLS 1.2 (a security protocol version), not
  1.3, because the crypto library available on this development machine
  (OpenSSL 3.0.13) predates 1.3 support. What IS proven version-independently
  this way: mutual authentication, forward secrecy, certificate-bound
  identity, cross-authority refusal, revocation. DTLS 1.3 ITSELF - the actual
  protocol version this project ships - is proven separately, only on real
  hardware (see below).
- It cannot see real-world failure modes: a stuck file handle, a starved
  CPU task, a dead radio, a power cut.

**2. Field evidence (`evidence/*.md`), indexed in `EXPERIMENT_REGISTER.md`.**

A test observed from OUTSIDE the system under test: a packet capture taken
independently on the network (Wireshark/tcpdump), a device's own serial
console log, a database query run against the live production database, a
screenshot of the dashboard. This is the only kind of result this project
calls "proven" in the memo's evaluation chapter.

---

## The rule: a claim is only evidence when observed from outside

Method borrowed deliberately from the parallel pre-shared-key project (Rull Ventura) (final-year thesis),
credited as such in `EXPERIMENT_REGISTER.md`: **a passing test proves the
code agrees with itself; a capture, a log, or a query proves the deployed
thing behaved.**

Analogy: a company auditing its own books is not the same as an independent
auditor checking the bank statements. SimLab is the company checking its own
books - necessary, and it catches most mistakes early, but it is not
sufficient proof on its own. The `evidence/` files are the independent
audit: something recorded by a tool that has no stake in the system's own
claim about itself (a network sniffer, a device's raw serial output, a
direct SQL query against the database file).

**Every entry in `EXPERIMENT_REGISTER.md` follows the same shape**, which is
what makes it citable: `# | Experiment | What it proves | Evidence file |
Status`. The evidence file itself states the claim under test, the exact
setup, the exact steps, and - critically - the pass/fail criteria written
BEFORE the test ran, so the method is not shaped by the result. Several
entries also record a stated failure in advance: what result would have
counted as a failure, named up front, so a favorable result can't be quietly
redefined after the fact.

## For the memo

- Cite `EXPERIMENT_REGISTER.md`'s table directly as the evaluation chapter's
  index. Each row points to one evidence file.
- Where a SimLab gate result is cited as supporting evidence (e.g., "the
  attack suite is proven to defeat 7 attack classes"), quote or paraphrase
  the gate's own SCOPE block alongside it, so the claim is exactly as strong
  as the evidence and no stronger.
- Keep the raw artifacts (`.pcap` files, serial logs, screenshots) as
  appendix material. The written evidence `.md` files summarize them, but the
  raw files are what makes a claim independently checkable rather than
  merely asserted.
- Two corrections already recorded in `DEV_CHRONOLOGY.md` (the NMU's "mean
  chunk-wise dB SPL" correction, and the AMU discovery-fallback-order
  correction) are worth citing as evidence the process itself is honest, not
  just the results - a claim that survives being checked and corrected is
  stronger than one that was merely stated once.
