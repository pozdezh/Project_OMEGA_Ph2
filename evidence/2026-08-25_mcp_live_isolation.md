# A live MCP query reaches real hardware and leaves no trace in the record

Captured 2026-08-25 on the deployed fleet, 16 units live.

## What was asked

Through Claude Code, in plain language, against the fleet's MCP server:

    take a live reading from AMU_14 right now

AMU_14 is a unit built that day by cloning a working card. It had never been
named in any configuration file: its address was learned by the server from
its own first handshake (see server/device_addresses.py).

## What came back

    direct call, sampled 0.9 s ago, 2026-08-25 21:40:13 CEST

    DHT22     30.29 C   64.17 %
    Enviro+   29.11 C   64.55 %   1000.7 hPa   0 lux
    SCD30     25.55 C   82.34 %   525.79 ppm CO2
    PMS5003   PM1.0 5   PM2.5 6   PM10 6 ug/m3

`via="direct"` means the server opened a fresh mutual-authentication DTLS 1.3
session to the physical unit and it answered - not a database lookup.

## What the database held at that moment

    AMU_14 rows in the previous five minutes : NONE
    rows whose cause mentions query/operator/MCP (whole table) : 0

The unit was measuring - it produced a sample 0.9 s old - and had decided
none of it warranted transmission. The live query saw that sample; the record
does not contain it.

## Why that is the correct behaviour, not a gap

The database answers one question: **what did the fleet decide to report?**
That is what makes statistics over it meaningful. Storing live reads would
change the question to "what the fleet reported, plus whatever an operator
happened to look at", and every average over that window would quietly
include the looking.

For an NMU the objection is stronger still, because the quantities differ. A
live read returns `audioAmbientDb()` - the rolling ambient level, no event
attached. `noise_data.db` holds the MEAN CHUNK-WISE dB SPL of a detected
event. Same column name, same unit, different measurement. Nothing downstream
would flag the mixture.

## How it is enforced

Structurally, not by convention. Exactly one path writes to the database:

    session.py -> storage.ingest_telemetry()

reached only by a record a device chose to send. The live-query path returns
its answer to the caller and returns. `session.py` states the rule in its own
header: *"Must never touch the database directly."*

## Sensor spread in the reading above

Three temperature sensors disagree by up to 4.7 C. That is expected - they
sit in different positions on the board with different self-heating - and is
precisely why alarm decisions use the MEDIAN of the redundant sensors rather
than any one of them, so a single drifting sensor can neither raise nor mask
an alarm. The SCD30 reads coolest and therefore reports the highest relative
humidity for the same air, which is consistent physics rather than a fault.
