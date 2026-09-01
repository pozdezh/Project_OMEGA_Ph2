# Future work

Limits this system has that are known, understood, and deliberately not
closed in this cycle. Each one states what it costs, why it was left, and
what closing it would take. Nothing here is a surprise waiting to be found.

---

## 1. Two exact clones of one unit cannot be told apart

**The gap.** The duplicate-identity guard catches a board that presents a
DIFFERENT certificate for a name the server already knows. It cannot catch
two boards flashed with the SAME name, the SAME certificate and the SAME
private key, because there is nothing to distinguish them: cryptographically
they are one identity. Both complete the handshake, both match the recorded
fingerprint, and both are admitted.

**Proven, not assumed.** The listener tracks sessions by network address
(`known_peers` in `server/listener.py`), not by device id, so two clones at
two addresses hold two concurrent sessions under one name.

**What it costs.** Their readings merge into a single device id. Their event
ids do not collide either - each board draws a random session number at boot
- so the `(id, event)` uniqueness index does not notice. The visible symptom
is one unit reporting at roughly twice its expected rate with values that
jump between two physical locations. The data is silently wrong, which is
the worst kind of wrong.

**Why it is not a protocol failure.** The guard exists to separate a genuine
unit from an impostor. Holding a real private key is not impersonation - it
means the key was copied from a device or from the provisioning machine, and
whoever did that had already succeeded at something the protocol was never
positioned to prevent. This is a manufacturing and process failure.

**Why the tooling makes it unlikely but not impossible.**
`omega_pki.py device` refuses to issue a second certificate for an existing
name without `--force`, and `make-units` refuses the same name twice inside
one batch. What it cannot see is two separate runs: re-flashing a name
deliberately REUSES its certificate, which is correct when replacing a dead
board and would produce genuine clones if two different boards were flashed
with the same name in two sittings.

**What closing it would take.** The certificate cannot distinguish them, but
physics can: twins may share a passport, but not stand at two border posts
at once. Roughly fifteen lines in `handle_peer` - a map of device id to the
address currently holding a session, warning loudly and surfacing on the
dashboard when a second address appears for a name while the first is still
live. It would detect and report rather than block, because the legitimate
case (a unit that moved to a new address before its old session expired)
looks identical for a few seconds.

**Why it was not done in this cycle.** It was identified during the field
test, with units arriving, and touching the listener at that moment risked
the run for a detection that reports a mistake nobody had yet made.

---

## 2. Enrolment is manual

A new unit gets its identity from the provisioning machine and is flashed
with it. The design for automatic enrolment - device generates its own key
pair, requests a certificate, appears as "pending" on the dashboard until a
human approves it - is written up in `CLAUDE.md` and not built. The manual
path is sound and auditable; it simply does not scale past a fleet a person
can hold in their hands.

---

## 3. Power draw has never been measured with an instrument

The sleep work is proven correct in behaviour - the radio enters the state
asked of it, and the DTLS session survives every sleep and wake cycle,
confirmed by a handshake counter that never moved across a 165 s run. The
milliamps saved are unquantified. Every unit in this deployment is mains
powered, so this is an efficiency and design result, never a battery-life
claim.

---

## 4. Long-idle behaviour is unobserved

The dominant re-handshake checkpoint is the server's session age limit
(1 hour, plus or minus 20 percent jitter). Observing it needs an hour-plus
continuous run that has not been done. Shorter checkpoints - the idle
timeout and the device's own retry ladder - have all been exercised.

---

## 5. The in-memory gate runs DTLS 1.2

OpenSSL on the development machine predates DTLS 1.3 and wolfSSL has no
Windows build, so the gate's channel is 1.2. What it proves is
version-independent: mutual authentication, forward secrecy,
certificate-bound identity, refusal of an unknown authority, revocation,
replay absorption and detection of an altered record. **DTLS 1.3 itself is
proven only on hardware**, where the live server and the ESP32 both report
`DTLSv1.3 / TLS_AES_128_GCM_SHA256`. This is a limit of the test bench, not
of the system.

---

## 6. Records are sent one at a time

A unit sends one record and waits for its receipt before sending the next.
DTLS imposes no such restriction - unlike TLS over TCP, a lost datagram does
not stall the ones behind it - so the transport could carry several records
in flight at once. Stop-and-wait was chosen because the receipt is what
makes "no reading is ever silently lost" provable: nothing leaves a unit's
buffer until its own acknowledgement arrives. Pipelining would require
tracking a window of unacknowledged records and matching receipts out of
order, for throughput a fleet sending a handful of records an hour does not
need.

---

## 7. The certificate authority lives on the server in the hub build

`install_hub.sh` puts the signing key on the same machine as the server,
which means anyone who gains root there can mint an identity the fleet will
accept. This is a deliberate prototype trade, stated in that file's header
rather than hidden, and the alternative is partly built: the NMU pendrive
tool (`provisioning/pendrive/make-units`) reads the CA key straight from the
drive (`omega_pki.py ... --ca-key-dir`) and never copies it to the server, so
noise units can be added while a person is physically present and the key is
never resident on disk. The AMU card tools (`clone-amu-card`,
`prepare-amu-card`, `make-amu-bundle`) do not yet take `--ca-key-dir`; they
require `ca-key.pem` in the server's PKI directory, so a fully key-off-server
build needs the key present only while AMU cards are written and removed
afterward, or the AMU certificates issued by hand. Closing this is one
`--ca-key-dir` pass-through per tool. What neither choice changes is the
protocol - a hub compromise lets an attacker JOIN the fleet; it does not let
them read or forge traffic between units that already exist.

---

## 8. Neither device can keep time through a power cut

Every reading is dated by the device that took it, and neither device can
tell the time on its own after losing power. Both work around it in
software; neither workaround survives the plug being pulled.

**The AMU.** A Raspberry Pi 4B has no real-time clock at all - confirmed on
the hardware (`/dev/rtc` absent) and stated by Raspberry Pi themselves: an
RTC "is not included" to keep cost and size down, and all Pis set their time
from the internet. The Pi 5 is the first model to add one, with a battery
connector and an ML2020 cell sold for it. What fills the gap on a Pi 4 is
software writing the time to the SD card periodically and on shutdown, and
restoring it at boot - so a CLEAN reboot comes back with the right time and
a POWER CUT comes back stale by however long the unit sat unpowered.

**The NMU.** The ESP32-S3 keeps time in its RTC power domain, which a
software reset does not clear. So the clock genuinely survives the reboot the
recovery ladder performs - and is lost the moment power is removed, because
nothing on the board is battery-backed.

**What was built instead.** Each unit distinguishes the two cases and refuses
to guess when it cannot tell: the AMU leaves itself a note immediately before
a deliberate reboot and consumes it once on the way back up; the NMU keeps
its clock-trust flag in the same power domain as the clock, so the flag and
the clock are lost together and can never disagree. A reading whose time is
genuinely unknown is sent as unknown and dated by the server on arrival,
which is correct for a live reading and admitted to be approximate for a
buffered one.

**The hardware fix, for both.** A DS3231-class RTC module costs a few euros,
connects over I2C on either device, and holds the time on its own cell
through an outage of any length. It would make the software workarounds
unnecessary and, more importantly, make timestamps correct through the one
case the current design cannot cover: a building-wide power failure, which is
precisely when the noise and air-quality record matters most.

Worth noting for the NMU specifically: such a module has an interrupt output,
so it can WAKE OR SIGNAL the device on an alarm rather than being polled.
That keeps the design event-driven, consistent with the rest of the system,
and avoids both a polling loop and the low-level ULP co-processor
configuration under ESP-IDF that an on-chip alternative would require. The
Arduino core the NMU is built on can attach an interrupt to that pin
directly - no change of framework.

Sources: Raspberry Pi documentation (Pi 5 RTC battery connector and the RTC
battery product page) and Raspberry Pi's own statements that earlier models
omit an RTC by design.

---

## 9. No sustained test at full fleet scale

**The gap.** The deployment provisions 40-plus units per type, but the
evidence runs use 16 real units at most, and only 2 of those run
continuously for a long window. Concurrency (four handshakes in one
millisecond), discovery, revocation distribution and a simultaneous
whole-fleet blackout are all shown at 16; the dashboard is shown to hold up
at 40 in a synthetic test (`server/test_dashboard_scale.py`), not against 40
real radios.

**What it costs.** Three things are unquantified above 16 real units: the
server's per-session memory and CPU under a genuine 40-plus concurrent load;
the mDNS query storm when a large fleet powers on together; and whether the
boot-jitter spread (0-30 s) is wide enough to keep a 40-unit reconnect from
briefly overrunning the listener's accept path. The design has margin for
all three and the 16-unit results trend the right way, but that is a
projection, not a measurement.

**What closing it would take.** A bench rack of the reserve units, all on
one access point, driven through a cold start and a mid-run AP bounce while
the server's own resource counters and the listener's accept latency are
logged. A day of setup, a day of running.

**Why it was not done in this cycle.** The reserve units are unflashed
stock and the field test window was spent on the units going into
residences. Scale evidence beyond 16 was ranked below getting the
deployment right.

---

## 10. Firmware updates still need a cable per unit

**The gap.** Every firmware change is flashed over USB, one board at a time.
There is no over-the-air path, so a fix shipped after the units are in
residences means a site visit and an opened enclosure for each one.

**What is already done.** The ESP32 partition table (`nmu/partitions.csv`)
reserves two equal 3 MB application slots and an `otadata` partition
specifically for this - the current image is ~1.2 MB, so there is room. That
layout had to be fixed before any unit was installed, because repartitioning
erases the flash; reserving the space cost nothing now and not reserving it
would have cost a site visit per unit later. The groundwork is in place; the
update mechanism on top of it is not.

**What closing it would take.** An OTA service that (i) carries the new
image over the same mutually-authenticated DTLS channel the telemetry uses,
so no new trust path is introduced; (ii) verifies a signature on the image
before it is marked bootable, so a tampered or truncated download cannot
brick a unit; (iii) writes into the idle slot, switches `otadata`, and rolls
back automatically if the new image fails to check in within a few minutes.
The A/B slot layout already supports the rollback; the signing key and the
check-in handshake are the new parts. On the AMU side the equivalent is a
signed package pulled over the same channel, which is a smaller job because
the Pi already runs a package manager.

**Why it was not done in this cycle.** OTA is only useful once the fleet is
too large or too dispersed to cable, which is exactly the point this
deployment is now reaching but had not reached during development. The
partition decision was the part that could not wait; the service could.

---

## 11. The NMU senses by continuous sampling, and rides the Arduino core

**The gap.** The noise unit detects events by keeping one core busy the
whole time: `SentryTask` runs a continuous DMA capture off the ADC
(`esp_adc/adc_continuous`) and computes a chunk-wise dB level around the
clock, comparing each chunk to a threshold. Nothing on the board sleeps
between sounds. The rest of the firmware - WiFi, mbedTLS, the FreeRTOS
setup, mDNS - is built on the Arduino-ESP32 core rather than bare ESP-IDF,
which was the right call for development speed but leaves several of the
chip's power and offload features out of easy reach.

**What it costs.** On a mains-powered fleet, nothing measurable today - this
is a design-elegance and headroom point, not a battery claim. What it
forecloses: a future battery or PoE-budget deployment; the ability to put
the main cores into deep sleep between acoustic events; and direct use of
the ULP co-processor, finer RTC-peripheral power domains, and native
`esp-tls`/mbedTLS DTLS 1.3 tuning, all of which the Arduino core either
hides or makes awkward.

**What closing it would take, in two independent steps.**

- *Hardware-triggered detection.* Put the acoustic threshold in front of the
  main SoC - an analog comparator or an envelope detector on the mic
  amplifier raising a GPIO, or the ULP co-processor sampling the ADC at low
  power - so the application cores wake only when sound crosses the
  threshold instead of polling for it. This keeps the design event-driven,
  consistent with the rest of the system, and is the same principle as the
  RTC-interrupt idea in section 8.
- *Move the firmware to bare ESP-IDF.* The audio path already uses IDF
  drivers directly, so this is a migration of the networking and lifecycle
  code, not a rewrite of the sensing. It unlocks ULP programming, deep-sleep
  with selective RTC peripheral retention, explicit power-domain control,
  hardware-crypto-accelerator configuration for the DTLS record layer, and
  removing the Arduino abstraction that currently sits between the code and
  those knobs.

**Why it was not done in this cycle.** The Arduino core was chosen
deliberately for commodity - faster iteration, a known-good mbedTLS
integration, `ESPmDNS` and the sensor libraries available without porting.
Continuous sampling is simple and correct on a mains-powered unit. Both
choices trade a capability the deployment does not currently need for
development speed it did need. The path off both is incremental, not a
restart.

---

## 12. Operator certificate revocation is coarse

**The gap.** A device certificate can be withdrawn from a web page and takes
effect on the device's next record (evidence: `2026-08-26_revocation_live.md`,
nine seconds mid-session). There is no equivalent for an operator
certificate. The dashboard and the MCP tools authenticate at the nginx gate,
which only checks that the client certificate is signed by the fleet CA
(`ssl_verify_client on`, `ssl_client_certificate ca-cert.pem`). No
certificate revocation list is loaded, and `app.py` does not inspect the
client common name. So a leaked operator key cannot be individually retired.

**What it costs.** If an operator laptop or its key is compromised, the only
ways to lock that key out today are to reissue the whole CA (which retires
every device certificate too) or to accept that the key stays valid.
Contained, not closed: a stolen operator key reaches the server only, where
every action is logged, and cannot reach a device directly because each unit
trusts the `omega-server` identity in its allow-list, not the operator
identity.

**What closing it would take.** The CA already carries `crl_sign`
(`provisioning/omega_pki.py:100`). A small `gen-crl` step writes a revocation
list naming the bad serial; one line in the nginx server block
(`ssl_crl /…/omega-crl.pem;`) plus a reload makes nginx reject that serial at
the TLS handshake, before any application code runs. A lighter stopgap: give
each operator a distinct common name, have nginx pass `X-SSL-Client-CN`
(already wired), and check it against a denylist in `app.py`.

**Why it was not done in this cycle.** One operator identity, one machine,
physically controlled. The CRL path is standard PKI plumbing with no design
question attached, so it was ranked as deployment hardening rather than
research.

---

## 13. Smaller known gaps, grouped

Each is understood, low-risk in this deployment, and left for the same
reason: it costs nothing measurable now and closing it is plumbing, not
research.

- **NMU secure boot and flash encryption are off.** The chip supports both
  (one-time eFuse features). Device identity currently sits in NVS and
  `omega_certs.h` readable with a flash dump. Physical possession of a unit
  is outside the current threat model, and a stolen unit is handled at the
  server by revocation. Enabling both is the standard step before a
  deployment where enclosures are not physically trusted.
- **The NMU ships a software-crypto build.** wolfSSL's ESP32 hardware
  acceleration is an ESP-IDF component feature with no supported path in the
  Arduino library. Measured gain if enabled: about 9 percent on the
  once-per-session handshake, nothing per record
  (`crypto_versions_and_cipher_choice.md` 2.6). Tied to the ESP-IDF
  migration in section 11.
- **The daily-statistics push has no real destination yet.** The sender,
  retry and idempotency logic is built and soaked (`2026-08-27_boss_endpoint_reconfirm.md`);
  `OMEGA_STATS_URL` and `OMEGA_STATS_TOKEN` are placeholders until the
  research unit supplies an endpoint. Until then it prints the report rather
  than losing it.
- **The AMU live agent runs a second Python interpreter.** The wolfSSL
  binding mismanages thread state on CPython 3.12+, so the inbound live-query
  listener runs under a locally built 3.11 (`deploy/amu/build_live_py.sh`,
  FINDINGS #35). The sampling and telemetry path keeps the system Python and
  is untouched. Upstream has no fix; 5.9.2.post0 is the newest release.
