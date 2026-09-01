# Project Omega - Brick 4 - system diagram

One schematic, for the memo, the defence slides, the GitHub repo and
LinkedIn. `system_overview.py` places every box and every connector by hand,
so nothing crosses and the proportions are fixed.

## Build

```
py -3.12 system_overview.py
```

Only needs matplotlib (already present in `py -3.12`). Outputs to `out/`:

- `system_overview.png` - transparent background (white memo page or a dark
  slide alike)
- `system_overview_on_white.png` - flattened onto white, for quick viewing
- `system_overview.svg` - vector
- `system_overview.pdf` - vector, and the script **also copies this straight
  to `memo/figures/fig_system_overview.pdf`**, which is what
  `memo/chapters/03_design.tex` includes as `fig:architecture`. Re-run
  `latexmk` in `memo/` after any edit here.

## What is in a box, and what is not

Each box carries **only its bold name**. Everything that used to sit inside
the boxes - hardware, filenames, ports, the handshake steps, the cron
schedule - now lives in the figure caption in `03_design.tex` and in the
chapter prose. The picture is a structural map: tiers, parts, and how data
moves between them. If you find yourself wanting to add a line of body text
to a box, add it to the caption instead.

## Sizing

The figure is drawn at `figsize=(6.1, 6.75)` inches. 6.1 in is exactly the
memo's `\textwidth` (15.5 cm), so at `width=\textwidth` the labels render at
their designed point size - roughly 8.2 pt bold names, 7.1 pt key. Do not
change the figure width without re-checking that the longest name still fits
its box: matplotlib's DejaVu Sans **bold** runs about 0.63 em per character,
so a box `w` units wide holds about `w * 35.7 / (0.63 * fontsize)`
characters. The whole diagram plus its caption occupies one page with room
to spare underneath.

## Editing

Everything is a table at the top of the script:

- `BOXES` - `name: (x0, y0, x1, y1, style, title)`. The title is the box's
  bold name, drawn centred in its edge colour. No body text - see above.
  Move or resize a box by changing its four numbers.
- `LINKS` - `(from_xy, to_xy, style, label, label_xy, label_ha)`. `style`
  picks colour, width, dash and arrowheads from `L`; `<|-|>` styles are
  drawn two-headed.
- `KEY_ROWS` and `KEY_BOX` - the embedded legend.
- `C` and `L` - the palette and the five line styles.

**Every link is axis-aligned.** A connector shares either its x or its y
with both endpoints - no diagonals, ever. One diagonal among a dozen
verticals is what makes a diagram look sloppy, and it is the only rule here
that is not negotiable. All arrowheads are drawn at a single `HEAD` size so
a 0.65-unit internal link caps the same as a 2.35-unit telemetry link.

The canvas is 0..12.15 wide, 0..15.25 tall (portrait). Links are constrained
to a small number of vertical columns so they cannot cross:

| x | carries |
|---|---------|
| 2.00 | AMU telemetry, field units down to the server frame |
| 10.00 | NMU telemetry, field units down to the server frame |
| 6.00 | provisioning, pen-drive down to the server frame |
| 3.00 | the left internal column - telemetry to DB, DB to web plane, web plane to operator |
| 8.50 | the right internal column - DB to cron, cron out to the research endpoint |

The three arrows from above stop at the server frame's top edge, which keeps
the band between the frame and the telemetry box free for the
`SYSTEM SERVER` label. After an edit, re-run and look at
`out/system_overview_on_white.png`.

The rendered figure says "SYSTEM SERVER", not the `omega-*` unit names used
in `deploy/`: "Omega" is a personal machine nickname the author keeps out of
the memo. The units are really `omega-listener.service` and
`omega-web.service`; the reference list below uses the real names.

The "reads, prunes" label on the DB -> cron link is deliberate: the nightly
`daily_stats.py` reads, and `db_retention.py` deletes the oldest rows when
the database file on disk crosses `OMEGA_DB_MAX_SIZE_MB` (500 MB by
default) - a size threshold, not a row count, and not a fixed periodic cut.
cron at 03:15 / 03:30 is only the polling cadence.

Arrowheads: `_arrow()` draws every head as its own short SOLID `-|>`
segment, separate from the shaft. A dashed or dotted shaft therefore still
ends in a clean filled triangle identical to the solid links - matplotlib's
own dashed arrowheads come out ragged and must not be used here.

## What it shows, and why it is this simple

Top to bottom: field units, the offline trust anchor, the server with its
software parts stacked in data-flow order, the two consumers, and the key.
Five channel types, each a distinct line:

| line | channel |
|------|---------|
| thick solid dark-blue, two-headed | DTLS 1.3 over UDP, mutual certificate - device telemetry + ACK (5000), discovery (5001) |
| solid purple, two-headed | mutual-TLS HTTPS over TCP - dashboard, REST API, MCP (443) |
| solid grey, one-way | HTTPS + bearer token, server-authenticated TLS only - nightly statistics report |
| dashed red, one-way | one-time offline provisioning by hand (CA certificate to every unit, signed identity to the server) |
| dotted gold, one-way | internal to the server - process and database access |

Detail that does not belong in a one-page overview is in the memo prose
(`memo/chapters/03_design.tex`) and summarised below.

---

## Reference: which server part does what

- **`omega-listener`** (one process) is the only thing that speaks DTLS.
  `listener.py` binds UDP 5000 and runs the accept loop; **`session.py`**
  performs the **DTLS 1.3 handshake** (`ctx.wrap_socket()`, wolfSSL), takes
  the identity from the verified certificate, checks the allow-list and
  revocation, and runs the read/ACK loop. `discovery.py` (UDP 5001 + mDNS)
  and the `db_worker` thread (the only DB writer) run in the same process.
- **`omega-web`** (`app.py`, Flask, `127.0.0.1:8081`) serves the dashboard
  and REST API, reads `sensor_data.db` read-only, and writes
  `device_config.json` for revoke / heartbeat. Its one DTLS action is
  `device_live.py`, a single outbound live call to an AMU as `omega-server`
  (not drawn - a secondary feature).
- **`nginx`** (TCP 443) terminates the **operator's** mutual-TLS
  (`ssl_verify_client on`) and proxies to Flask. Devices never go through it.
  The dashboard HTML is rendered by Flask and passed through by nginx.
- **cron**: `db_retention.py` trims + VACUUMs; `daily_stats.py` POSTs the
  nightly report to the research endpoint with `urllib` directly - no nginx,
  no client certificate, server-authenticated TLS plus a bearer token. The
  only path that leaves the LAN and the only outbound one that is not DTLS.
- **`mcp_server.py`** runs on the operator's machine, not the server, and
  calls the REST API through the same nginx mutual-TLS gate.
