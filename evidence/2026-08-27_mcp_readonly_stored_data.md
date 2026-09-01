# MCP stored-data tools: proven read-only, static and dynamic

Run 2026-08-27 07:38 CEST, against the deployed server code and a snapshot
of the real production database.

This is the mirror of `2026-08-25_mcp_live_isolation.md` (which proved the
LIVE-query tools never write) for the four STORED-DATA tools:
`list_devices`, `latest_reading`, `device_stats`, `activity_report`.
`set_heartbeat` is deliberately excluded - it is documented and tested
elsewhere as the one tool that DOES write (to device config, never to the
sensor database).

## 1. Static check

```
grep -n "INSERT\|UPDATE\|DELETE\|conn.commit" server/mcp_server.py   -> no matches
grep -n "INSERT\|UPDATE\|DELETE\|conn.commit" server/device_api.py   -> no matches
```

Neither file contains a single write statement or a `commit()` call. This is
a code-discipline fact, not a database-enforced one: `device_api._connect()`
opens the database with a plain `sqlite3.connect(db_path)`, not a read-only
URI (`file:...?mode=ro`). The guarantee here rests on the absence of write
code, confirmed by direct inspection, not on a permission the OS or SQLite
enforces for these tools. Worth noting as a genuine (if minor) hardening
opportunity: `mode=ro` would make this an enforced guarantee rather than an
audited one.

## 2. Dynamic check, first pass - against the LIVE production database

Row-level SHA-256 checksum of both tables, before and after calling
`list_devices` / `latest_reading` / `device_stats` / `activity_report`
repeatedly against the real, currently-running database:

```
before: noise_data rows=42737  sha256[:16]=34b65a5201f055aa
        air_data   rows=7480   sha256[:16]=96fecfcb7f6acd54
after:  noise_data rows=42738  sha256[:16]=1582a0e0541ba138   <- CHANGED
        air_data   rows=7480   sha256[:16]=96fecfcb7f6acd54
```

`noise_data` gained exactly one row and its checksum changed. **This is the
real, live fleet reporting concurrently during the test** - an NMU sent a
genuine event in the few seconds this took, exactly as it would with no
test running at all. It is not evidence of a write from these tools: the
new row is an appended one at a new rowid, not a modification of any row
that existed before, and it is the ordinary background activity this same
evidence set has documented all along (16 real units, live, unattended).

Still, "probably fine" is not the standard this project holds itself to
elsewhere, so a second pass removes the ambiguity entirely.

## 3. Dynamic check, second pass - against a frozen snapshot

Made with SQLite's own backup API, same technique as the retention and
boss-endpoint tests, so no concurrent writer can touch it during the run:

```
before: noise_data rows=42741  sha256[:16]=2496f79ffcaaf46d
        air_data   rows=7482   sha256[:16]=6d0f5576173ac095
```

320 real calls against every one of the 16 real deployed devices:

```
5 rounds x 16 devices x 4 tools = 320 calls, no errors
```

```
after:  noise_data rows=42741  sha256[:16]=2496f79ffcaaf46d
        air_data   rows=7482   sha256[:16]=6d0f5576173ac095
```

**Byte-identical**, both tables, both row count and full-table hash.

## Pass criteria, all met

- [x] No write statement exists in either file (static)
- [x] 320 real tool calls against the frozen snapshot leave it byte-identical
- [x] The one row-count change seen against the LIVE database is
  independently explained (real concurrent fleet traffic, a new appended
  row, not a modified one) rather than dismissed
- [x] `set_heartbeat` correctly excluded - it is the one tool with a write
  path, and it writes to config, never to this database

## Honest limitations

- The read-only property is enforced by there being no write code, not by a
  database-level read-only connection. A future regression that adds a
  write to `device_api.py` would not be caught by a permissions error - it
  would need a test like this one, or the static grep, to notice.
- This proves the four tools as they exist today. It is not a guarantee
  against a future tool added to `mcp_server.py` without the same audit.
