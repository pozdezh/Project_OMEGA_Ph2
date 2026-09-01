# Remaining evidence-gathering tests - procedures fixed in advance

Written before running any of these, per the register's own rule: a claim is
only evidence when observed from OUTSIDE the system that makes it, and the
method must not be shaped by the result.

Recommended order and why: #10 first (physical, unattended, highest chance
of a genuine new finding while there is still time to fix it), then the two
cheap desk checks (MCP proof, DB compaction), then #11 and #12 (measurement,
low risk), boss-endpoint last (already proven once, this is a re-confirm),
screenshots throughout as each test produces its own evidence.

---

## #10 - Recovery ladder, live, unattended

**Claim under test.** A unit that loses its network connection escalates
through its own recovery rungs and rejoins on its own, with no operator
touching it.

**Rungs (already in the shipped code, not new):**

| Device | Rung 1 | Rung 2 | Rung 3 |
|---|---|---|---|
| NMU | radio reset, floor 600s (10 min) | - | reboot, floor 1800s (30 min) |
| AMU | radio reset, floor 600s (10 min), or 1.5x heartbeat | process restart, 2.5x heartbeat | reboot, 3.5x heartbeat |

Actual trigger time = max(floor, multiple x that unit's current heartbeat).
Check each unit's live heartbeat on the dashboard before the test so you can
predict roughly when each rung should fire.

**Setup.** Pick ONE NMU and ONE AMU not used in a prior physical test this
week (avoid NMU_18 and AMU_13/14 - already handled). Confirm both reporting
normally first.

**Steps.**
1. Start a tcpdump/capture on the server scoped to that unit's IP (optional
   but cheap - keeps the "outside view" if anything surprising happens).
2. Cut ONLY that unit's network path without touching the unit itself:
   easiest is to block it at the WiFi access point (MAC filter / disable the
   port), not a server-side iptables DROP - that already tests differently
   in experiment #13. If the AP can't selectively block, physically move the
   unit out of range instead.
3. Do nothing. Do not SSH in, do not power-cycle, do not touch it.
4. Watch from the server only: `journalctl -u <service> -f` for the AMU, or
   the dashboard's last-seen column for the NMU (no serial console needed -
   serial access would make this an "assisted" test, not unattended).
5. Restore network access once you've seen at least the radio-reset rung
   fire (or after 35+ min, whichever is longer, to also see reboot).

**Pass criteria.** Log timestamps show escalation in the right order at
roughly the right offsets, the unit reappears reporting normally with no
manual intervention, and buffered readings from the outage arrive with their
original timestamps (same check as the 2026-08-26 revocation tests).

**Stated failure in advance.** No escalation within 2x the expected rung
time; escalation happens but the unit never reconnects unattended; buffered
readings lost or arrive out of order.

**Evidence file:** `evidence/2026-08-2X_recovery_ladder.md` + journalctl/
dashboard excerpts, one per device type.

---

## MCP stored-data tools - read-only proof

**Claim under test.** `list_devices`, `latest_reading`, `device_stats`,
`activity_report` never write to the database - same guarantee already
proven for the live-query path (`2026-08-25_mcp_live_isolation.md`).

**Steps.**
1. Record row counts before: on the server,
   `sqlite3 <db> "SELECT (SELECT COUNT(*) FROM noise_data)+(SELECT COUNT(*) FROM air_data);"`
2. From the operator machine, run each of the four stored-data tools once
   through Claude Code (`claude` on the server, or your local MCP client).
3. Record row counts after, same query.
4. Also worth stating for the memo (already true by code inspection): the
   only write path into either table is `storage.ingest_telemetry` on the
   telemetry path (`session.py`), which none of these four tools call -
   they all go through `device_api.py` read functions.

**Pass criteria.** Row counts identical before/after, for both tables.

**Evidence file:** `evidence/2026-08-2X_mcp_stored_readonly.md` - paste both
counts and the four tool outputs.

---

## DB compaction (retention/VACUUM), forced for real

**Background.** `db_retention.py` has only ever been tested on a COPY of the
live DB (2026-08-21 audit). This test runs it for real.

**Steps.**
1. On the server, back the DB up first:
   `cp /path/to/omega.db /path/to/omega.db.bak-$(date +%Y%m%d)`
2. Record row counts and file size (include the WAL file) before.
3. Force a run: `python3 server/db_retention.py <db_path>` (reads
   `DEFAULT_MAX_SIZE_MB`/`DEFAULT_PRUNE_FRACTION` from the module - lower
   `max_size_mb` as an argv override if the live DB is still under threshold
   and you want to actually see it prune today; state clearly in the
   evidence file if you did this to force the trigger rather than it firing
   naturally).
4. Record row counts and size after.
5. Reload the dashboard, confirm charts still render and recent data (last
   hour) is intact - compaction should only ever remove the OLDEST 10-20%.

**Pass criteria.** Oldest rows gone, newest rows and dashboard untouched,
`PRAGMA integrity_check` still `ok`, file size (incl. WAL) actually smaller.

**Evidence file:** `evidence/2026-08-2X_db_compaction.md`.

---

## #11 - OFF-versus-ON OFF vs ON

**Claim under test.** Side by side, the same payload is fully readable
unprotected and fully opaque protected - that same OFF/ON framing,
applied to this system.

**Setup.** Need a genuine cleartext capture to sit beside the existing
encrypted one (`2026-08-26_live_wire_capture.md`). Two options, pick
whichever is less disruptive:
  (a) Run one AMU sender temporarily against a throwaway plaintext-JSON UDP
      listener on a spare port, side by side with its real DTLS session (no
      change to the production listener or any real device's traffic).
  (b) Replay a captured plaintext payload from brick1 (the frozen fallback,
      genuinely unprotected) if a brick1 capture already exists.
Prefer (a) - it's the same live unit, same real sensor values, so the
comparison is apples-to-apples rather than an old baseline.

**Steps.**
1. Capture 60-100s of the plaintext side. Confirm in Wireshark: device
   names, field names (co2, temp, cause...), numeric values all readable.
2. Point at the existing `2026-08-26_live_wire_capture.md` pcap for the
   protected side (already DONE - no need to recapture).
3. Put both captures' entropy/readability side by side in one table.

**Pass criteria.** OFF capture readable in plain Wireshark (no decryption
needed); ON capture already proven opaque (7.61 bits/byte, zero terms
found).

**Evidence file:** `evidence/2026-08-2X_psk_off_on.md`, referencing the
existing ON evidence rather than duplicating it.

---

## #12 - Scale and timing

**Claim under test.** The already-deployed 16-unit fleet (8 AMU + 8 NMU)
handshakes and runs concurrently at a fleet scale, with real numbers.

**Steps.**
1. Handshake duration: trigger a reconnect on a handful of units (revoke/
   restore, or a heartbeat change, both already-proven ways to force one)
   and read the handshake time from each unit's own log line (NMU logs it
   directly; AMU via journalctl). Reuse the two existing hardware timings
   already measured (4190ms cold, 4415ms warm on NMU_18) rather than
   re-deriving them, but add 2-3 more units for a real spread, not n=1.
2. Concurrent sessions: `journalctl` grep across all 16 units' handshake
   completion timestamps during the boot-jitter window (0-30s spread,
   already implemented per FINDINGS #45) after a simultaneous power event,
   OR simply query the server's live known_peers/session table for a
   concurrent snapshot count during normal operation - simplest single
   command, already partially done in the live wire capture (4 units same
   millisecond).
3. Present as a small table: n units, handshake min/avg/max, peak concurrent
   sessions observed.

**Pass criteria.** No caveats needed here beyond what's already measured -
this is descriptive, not pass/fail. State sample size honestly (this is a
16-unit fleet, not the 40-unit simulation - keep those two numbers labeled
separately, real vs simulated).

**Evidence file:** `evidence/2026-08-2X_scale_timing.md`.

---

## Boss-endpoint send, re-confirmed

**Background.** Already proven live once (2026-08-20, FINDINGS #28) against
the sim endpoint. This is a fresh, dated re-confirmation for the evidence
register, plus an explicit retry-then-print check.

**Steps.**
1. Confirm the sim boss endpoint is running.
2. Force today's report: `python3 server/daily_stats.py <db_path>` with
   `OMEGA_STATS_URL`/`OMEGA_STATS_TOKEN` pointed at the sim (same env-var
   swap noted in FINDINGS #28 - no code change).
3. Confirm RECEIVED via the sim's own `/stats/received` log, not just "sent"
   from this end.
4. To prove retry-then-print: stop the sim endpoint, force a second run, and
   confirm it prints the report after 3 attempts rather than losing it, then
   restart the sim and confirm it does NOT resend that already-checkpointed
   report (idempotency).

**Pass criteria.** RECEIVED confirmed on the sim side; report is printed,
never lost, when the endpoint is down; already-sent reports aren't
duplicated on a later run.

**Evidence file:** `evidence/2026-08-2X_boss_endpoint_reconfirm.md`.

---

## Screenshots checklist

Take these as each test above produces them, not as a separate session:

- [ ] Dashboard, normal state (all 16 units green, recent data)
- [ ] Dashboard, one unit in revoked state (red/flagged, from #10 or a
      repeat of the existing revocation click)
- [ ] MCP terminal session (a real `claude` query + answer, from the
      stored-data proof above)
- [ ] Boss-endpoint output (the sim's `/stats/received` log entry)
- [ ] Wireshark, side by side: OFF (readable) vs ON (opaque) from #11
