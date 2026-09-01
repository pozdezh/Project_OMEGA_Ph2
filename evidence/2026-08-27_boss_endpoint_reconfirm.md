# Boss-endpoint push: already built, was already working, found silently broken

Run 2026-08-27 07:15-07:26 CEST on the deployed server, against the REAL
production database (`~/omega_brick4/sensor_data.db`). No copy was needed:
`daily_stats.py` is read-only over the DB by design (see its own module
docstring) and a failed send never advances its checkpoint, so nothing here
could have disturbed live data.

## What this corrects

`NEXT_SESSION.md` (2026-08-27 05:10) states the boss-endpoint push "works
but only PRINTS ... until OMEGA_STATS_URL/OMEGA_STATS_TOKEN hold the real
research-unit endpoint." That was already stale when written. The real
state, discovered before touching anything:

- A systemd service, `omega-boss-sim.service`, has been running the
  project's own `simlab/boss_endpoint_sim.py` continuously since at least
  2026-08-16.
- `/etc/cron.d/omega-maintenance` has pointed `daily_stats.py` at it every
  night at 03:30 since the same date, and it delivered successfully for a
  week: reports `OMEGA_20260816` through `OMEGA_20260823` all appear in the
  sim's own independent received-log (`~/omega_brick4/boss_endpoint_sim_received.jsonl`),
  2 devices each, full min/max/avg/median/sd per variable.

## What was actually broken, and since when

`daily_stats.py` was updated on the server **2026-08-23 23:37** to add
`require_secure_url()` - a real security fix: refuse any endpoint that
is not `https://`, because the report and the bearer token would otherwise
cross the network in clear text. Correct, defensible code.

But `/etc/cron.d/omega-maintenance` still pointed the sim at plain
`http://127.0.0.1:9443/stats/ingest`, and nobody added the one override the
new check's own error message names (`OMEGA_STATS_ALLOW_INSECURE=1` -
documented in the code as "local simulator only", which this is exactly).

**Result, confirmed from `~/omega_brick4/logs/stats.log`:** every cron run
from 2026-08-24 through 2026-08-27 - four consecutive nights - printed

```
REFUSED: OMEGA_STATS_URL is http, not https. The report carries per-device
statistics and the Authorization header carries the bearer token; both
would travel in clear text. Use an https:// endpoint, or set
OMEGA_STATS_ALLOW_INSECURE=1 to override deliberately (local simulator only).
```

and delivered nothing. This is a code/config drift bug, not a design flaw:
a security improvement shipped without updating the one deployment file
that needed a matching change.

## Was any data lost?

No. `require_secure_url()` is checked BEFORE `build_report()` runs, so a
refused push never touches the checkpoint (`sensor_data.db.stats_state.json`).
The design is a rolling cutoff specifically so a missed run costs nothing:
the next successful run just covers a longer window. Confirmed below.

## The fix

One line, added to `/etc/cron.d/omega-maintenance` by the user over their
own sudo session (the password never enters a command Claude runs):

```
sudo sed -i '/OMEGA_STATS_TOKEN/a OMEGA_STATS_ALLOW_INSECURE=1' /etc/cron.d/omega-maintenance
```

## Proof 1: the catch-up run delivers, corroborated independently

Checkpoint before: `{"last_report_end_ts": 1787448602}` (2026-08-23, the
last successful night).

```
OMEGA_STATS_URL=http://127.0.0.1:9443/stats/ingest \
OMEGA_STATS_TOKEN=sim-token OMEGA_STATS_ALLOW_INSECURE=1 \
python3 daily_stats.py ~/omega_brick4/sensor_data.db
```

```
stats report OMEGA_20260827: 16 devices, period 2026-08-23 01:30:02 -> 2026-08-27 05:24:19 UTC
stats push: delivered
```

Checked against the SIM's own log, not the sender's claim:

```
report_id: OMEGA_20260827
devices: 16
received_at: 1787808260
```

The 4-day gap is covered in one report exactly as the rolling-checkpoint
design intends - no window was skipped, none double-counted.

## Proof 2: retry-then-print, against a genuinely unreachable endpoint

A second manual run, pointed at port 9599 (nothing listens there), tests
the DOWN-endpoint path specifically - a different failure mode from the
REFUSED-insecure-URL path above, and the one `REMAINING_TESTS_PROCEDURE.md`
originally asked for. No production file was touched: the real sim and the
real cron config were left exactly as fixed.

```
stats push attempt 1 failed: <urlopen error [Errno 111] Connection refused>
stats push attempt 2 failed: <urlopen error [Errno 111] Connection refused>
stats push attempt 3 failed: <urlopen error [Errno 111] Connection refused>
stats push: FAILED (report printed above)
```

The full report (16... 13 devices for this narrower window, every
device/variable, all summary statistics) printed to stdout in full -
nothing silently dropped. Checkpoint immediately after:
`{"last_report_end_ts": 1787808260}` - byte-identical to before the failed
attempt. Confirmed unchanged.

## Proof 3: recovery covers the failed window exactly, no duplication

A third run, back against the real sim, with no code or config change:

```
stats report OMEGA_20260827: 14 devices, period 2026-08-27 05:24:20 -> 2026-08-27 05:26:09 UTC
stats push: delivered
```

`period_start` (05:24:20) is the exact instant the checkpoint was frozen at
by proof 2's failure - not a second earlier, not a second later. The window
that failed to send is the same window this run delivered: nothing skipped,
nothing sent twice. Checkpoint advanced to `1787808370`.

## Pass criteria, all met

- [x] RECEIVED confirmed independently on the sim's own log, not just "sent"
  from the sender's side
- [x] Report is printed in full and never lost when the endpoint is down
- [x] The checkpoint does not advance on a failed send
- [x] A subsequent successful run covers exactly the previously-failed
  window - no data skipped, no duplication
- [x] Found and fixed a real, dated regression along the way (Aug 23-27,
  four nights silently failing) rather than only re-confirming a scripted
  procedure

## Honest limitations

- This is still a SIMULATED endpoint (`boss_endpoint_sim.py`), not the real
  research-unit HTTPS endpoint - `OMEGA_STATS_URL`/`OMEGA_STATS_TOKEN` still
  need the real values once that endpoint exists (unchanged from prior
  status).
- The plaintext-http path is deliberately allowed here via
  `OMEGA_STATS_ALLOW_INSECURE=1`, which the code itself frames as
  simulator-only. The real endpoint MUST be https, and the safety check that
  refuses plaintext is exactly the mechanism that (correctly) blocked
  delivery for four days - it did its job; the deployment config just
  hadn't caught up to it yet.
- The four-night gap itself was not directly observed in progress; it was
  reconstructed from `stats.log` and the `daily_stats.py` file mtime. Both
  are server-side artifacts independent of this session's own commands.
