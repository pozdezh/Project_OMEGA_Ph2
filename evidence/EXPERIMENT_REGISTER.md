# Experiment register - Omega field evidence

One row per experiment, so the memo's evaluation chapter can be assembled from
facts rather than recollection. Every entry names what was claimed, how it was
tested, and what the evidence file is.

Method borrowed from the parallel pre-shared-key project (Rull Ventura): **a claim is only evidence when it is
observed from OUTSIDE the system that makes it.** A passing test proves our
code agrees with itself; a packet capture, a serial log or a database query
proves the deployed thing behaved.

## Status key

- **DONE** - run, evidence written
- **RUNNING** - in progress
- **READY** - procedure fixed, waiting on hardware or a quiet moment
- **PLANNED** - agreed, not yet specified in detail

---

| # | Experiment | What it proves | Evidence | Status |
|---|---|---|---|---|
| 1 | Live wire capture | Payloads are encrypted on the deployed system; sessions are concurrent | `2026-08-26_live_wire_capture.md` + `.pcap` | **DONE** |
| 2 | Attack suite | 7 attack classes defeated against shipped server code | `attack_suite.md` | **DONE** |
| 3 | NMU cold discovery (inside view) | A unit with no memory of the server finds it unattended | `2026-08-25_cold_discovery.md` | **DONE** |
| 4 | MCP live isolation | An operator query never writes to the database | `2026-08-25_mcp_live_isolation.md` | **DONE** |
| 5 | AMU restart (outside view) | Handshake on the wire; revealed the AMU uses a configured address, not discovery | `2026-08-26_boot_discovery_handshake.md` s.5 | **DONE** |
| 6 | NMU cold + warm boot, both views at once | Serial log and packet capture of the SAME boots; cold finds the server with empty memory, warm uses its cache first | `2026-08-26_boot_discovery_handshake.md` s.1-2 | **DONE** |
| 7 | Certificate exposure, 1.2 vs 1.3 | DTLS 1.3 hides identity: 6 plaintext records (ClientHello/ServerHello only), 34 encrypted; zero certificate names in the capture | `2026-08-26_boot_discovery_handshake.md` s.3-4 | **DONE** |
| 7b | Handshake recapture on a SECOND unit (NMU_21) | Reproduces #7's split on different hardware, different day: again exactly 6 plaintext records (3 ClientHello, 3 ServerHello), rest opaque. Root cause of the 3x found in code - both ends run a ~1s retransmit timer (`omega_dtls.cpp` DTLS_RETRY_INIT_S), two flights each missed one window. RFC 9147 mandates at least one of each, NOT three | `2026-08-27_nmu21_handshake_recapture.md` | **DONE** |
| 8 | Revocation, live (NMU_18) | Mid-session kill in 9 s, 11 consecutive refusals, instant restore, 100 buffered readings recovered with original timestamps. Unattended rejoin NOT proven - board had been handled | `2026-08-26_revocation_live.md` | **DONE, one gap** |
| 8b | Revocation from the UI, both device types, untouched | NMU_18 and AMU_13: instant kill/refuse, unattended rejoin proven both devices; AMU_13 also self-healed an UNPLANNED transport failure via the same-day nmcli fix, no intervention. No data lost across either device or fault. | `2026-08-26_revocation_live.md`, `_revocation_amu.md` | **DONE** |
| 9 | Server outage and gap fill | Nothing is lost while the server is down; the gap fills on return | DEV_CHRONOLOGY.md 2026-08-26 "Lights out" + internet-only outage entries | **DONE** |
| 10 | Recovery ladder | A unit that loses the network escalates and recovers itself, unattended | DEV_CHRONOLOGY.md 2026-08-26 (NMU_21 pass; AMU side re-run pending) | **DONE, PASSED both sides** - AMU_15 rebooted itself at 00:37:38 on 2026-08-27, ~49 min into a MAC block, with nobody touching it; recovered via the fixed WiFi path and flushed 41 buffered records with their original timestamps. Three real bugs were found and fixed to get there. Intermediate rungs not evidenced this run (the unit's journal does not survive its own reboot) |
| 11 | OFF-versus-ON OFF vs ON, extended to 4 rungs | One live AMU_T1 reading sent four ways at once. Cleartext fully readable; the predecessor's AES-128-ECB readable-free but repeating 2814 of 3523 blocks against the plaintext's own 2811 - it carries the plaintext's repetition structure one for one; AES-GCM (her future work, our brick1) and live DTLS 1.3 both exactly 0. Entropy CANNOT tell ECB (7.90) from DTLS 1.3 (7.91) - only block repetition separates them | `2026-08-27_psk_ladder.md`, scripts `simlab/psk_ladder_emit.py` + `_analyse.py` | **DONE** (rung D n=4 at the AMU's real 85s cadence, stated in the file) |
| 12 | Scale and timing | Real 16-unit fleet: reuses the 4-way same-millisecond concurrency proof (08-26) + this session's 16-distinct-device capture; handshake 4190/4415ms (NMU_18, device-logged) cross-checked against a 3rd, independently network-observed ~4.12s (NMU_21) | `2026-08-27_scale_timing.md` | **DONE** |
| 16 | MCP stored-data tools read-only | Static: zero write statements in mcp_server.py/device_api.py. Dynamic: 320 real calls (16 devices x 4 tools x 5 rounds) against a frozen snapshot, byte-identical before/after; live-DB pass explains the one incidental row-count change as real concurrent fleet traffic, not a write | `2026-08-27_mcp_readonly_stored_data.md` | **DONE** |
| 17 | DB compaction, forced live | Forced against a real DB copy (500MB budget spoofed to 1MB): pruned exactly 15% of both tables (oldest rows only, newest 5 byte-identical before/after), file shrank 6.6MB->5.3MB, integrity_check ok | `2026-08-27_db_compaction.md` | **DONE** |
| 18 | Boss-endpoint send, re-confirmed | Found already built and running a week unattended (2026-08-16+), then found SILENTLY BROKEN 4 nights (2026-08-23 https-only fix shipped without updating cron's plaintext sim URL) - fixed with the one documented override, catch-up delivery confirmed on the sim's OWN log (not the sender's claim); retry-then-print against a genuinely unreachable port proven separately, checkpoint untouched on failure, recovery covers the exact failed window with no duplication | `2026-08-27_stats_endpoint_reconfirm.md` | **DONE** |
| 13 | Heartbeat distribution, revoked + firewall-blocked units | No queue exists - every ACK reads config fresh, so a unit locked out the WHOLE change still gets it correctly on its first contact. Proven on 4 units, 2 failure modes (revoke, silent firewall block) | `2026-08-26_heartbeat_distribution.md` | **DONE** |
| 14 | Heartbeat/alarm rate coupling (found from the DB, not planned) | A short heartbeat was multiplying visible alarm records (24 in 30 min vs the 6 alarm_repeat_s allows) - fixed in triggers.py, AMU only. Regression test proven to fail against the old code | DEV_CHRONOLOGY.md 2026-08-26 | **DONE, fixed, deployed 8/8 AMU** |
| 15 | Server clock floor + monotonic session timing | Server withholds "t" rather than distributing a provably wrong date; session age immune to an NTP step mid-session (code-level, monotonic clock). Live proof: 10 independent unforced session-age reauths this run, jitter spread 3340-4306s across the designed 2880-4320s band, zero data loss at the boundary (checked against the real DB) | DEV_CHRONOLOGY.md 2026-08-26, `server/test_config_store.py`, `2026-08-27_session_age_reauth.md` | **DONE** |

---

## Experiment 6 - procedure, fixed in advance

Written before the run so the method is not shaped by the result.

**Claim under test.** A unit with no stored knowledge of the server finds it by
itself and reaches an authenticated session, unattended - and an observer on
the network sees discovery in the clear but nothing of the identity or the
readings.

**Setup.** One NMU (Adafruit Feather ESP32-S3) on this PC by USB. Server
capturing on ports 11400 (telemetry), 5001 (discovery probe) and 5353 (mDNS).

**Steps.**
1. Start the capture on the server.
2. Erase the unit's NVS partition (0x9000, length 0x5000) with esptool 5.3.1.
   That is the ONLY place it remembers a server address; firmware carries no
   server IP (`OMEGA_SERVER_IP ""`).
3. Reset the unit and record its serial output at 115200 from power-on.
4. Stop the capture once the first reading is acknowledged.

**What counts as a pass.** Both views agree: the serial log shows discovery
then a DTLS 1.3 session, and the capture shows the same exchange at the same
timestamps, with no device name or reading readable in it.

**What would count as a failure, stated in advance.** A device name, a
certificate subject or any sensor value appearing in cleartext in the capture;
or the unit failing to find the server without help.
