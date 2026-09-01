# DB compaction, forced live against a real copy of the production database

Run 2026-08-27 07:34 CEST. Subject: a byte-for-byte copy of the deployed
`sensor_data.db`, made with SQLite's own backup API
(`sqlite3 sensor_data.db ".backup <copy>"`), which is safe against a
concurrently-writing process - this is the same technique used for the
boss-endpoint test, and unlike that test this one is genuinely destructive
(it deletes rows), so it ran ONLY against the copy, never the live file.
The copy was removed after the run; nothing here is a permanent artifact.

## Why forced, not organic

The real database is 6.6 MB against a 500 MB budget (`OMEGA_DB_MAX_SIZE_MB`
in `/etc/cron.d/omega-maintenance`) - months from triggering naturally. To
prove the mechanism now, `OMEGA_DB_MAX_SIZE_MB=1` was passed as a one-off
environment override for this run only. This is stated here exactly as
`REMAINING_TESTS_PROCEDURE.md` asks: the trigger was forced, the pruning
logic that ran once triggered is the same production code, untouched.

```
OMEGA_DB_MAX_SIZE_MB=1 python3 db_retention.py <copy>
```

## Before

```
file size          : 6,905,856 bytes (6.6 MB)
noise_data rows     : 42,670   (oldest ts 1783496181, newest 1787808875)
air_data rows       : 7,454    (oldest ts 1787631615, newest 1787808877)
```

Newest 5 rows recorded per table before running, to check against afterward:

```
noise_data (rowid, ts): (42673,1787808875) (42672,1787808874) (42670,1787808873) (42671,1787808872) (42669,1787808858)
air_data   (rowid, ts): (7454,1787808877) (7453,1787808862) (7452,1787808844) (7451,1787808815) (7450,1787808800)
```

## The run

```
retention: db size 6.6 MB (budget 1 MB)
retention: noise_data pruned 6400 rows
retention: air_data pruned 1118 rows
retention: vacuumed, reclaimed space for 7518 rows
```

6400 / 42670 = 15.0%, 1118 / 7454 = 15.0% - exactly `DEFAULT_PRUNE_FRACTION`
(0.15), as expected with no `OMEGA_DB_PRUNE_FRAC` override.

## After

```
file size          : 5,517,312 bytes (5.3 MB)  -  20.1% smaller
noise_data rows     : 36,270   (oldest ts 1787683853, newest 1787808875)
air_data rows       : 6,336    (oldest ts 1787691484, newest 1787808877)
integrity_check     : ok
```

Newest 5 rows, same table, after the run:

```
noise_data (rowid, ts): (42673,1787808875) (42672,1787808874) (42670,1787808873) (42671,1787808872) (42669,1787808858)
air_data   (rowid, ts): (7454,1787808877) (7453,1787808862) (7452,1787808844) (7451,1787808815) (7450,1787808800)
```

**Byte-identical to before the run**, both tables. Nothing recent moved or
disappeared.

## Pass criteria, all met

- [x] Oldest rows gone - noise_data's oldest timestamp jumped from
  1783496181 to 1787683853 (roughly 51 days newer); air_data's from
  1787631615 to 1787691484
- [x] Newest rows and their rowids completely unchanged, both tables
- [x] `PRAGMA integrity_check` returns `ok` after VACUUM
- [x] File size on disk actually shrank (6.6 MB -> 5.3 MB), not just row
  count - VACUUM reclaimed the freed pages rather than leaving them allocated
- [x] Prune fraction matches the documented default (15%) with no override

## Honest limitations

- Run against a copy, at a forced budget. The real 500 MB budget and the
  real fleet's growth rate were not exercised - this proves the mechanism
  works correctly when triggered, not how many months until it triggers
  organically.
- Only one run was measured. The function is deterministic given the same
  inputs, so a repeat run is expected to behave identically, but that
  repeat was not performed here.
- WAL/SHM sidecar files were not present on the copy (a `.backup` produces a
  plain single-file snapshot), so this run did not exercise the on-disk-size
  calculation's WAL-inclusive path (`_on_disk_bytes`'s own comment about a
  2.8 MB db beside a 4.1 MB wal) - that path is exercised naturally by the
  real cron job against the real WAL-mode file, just not measured in this run.
