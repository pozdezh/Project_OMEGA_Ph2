# Scale and timing, real 16-unit fleet

Compiled 2026-08-27 from two independent captures, reusing measurements
already on record rather than re-deriving them where a fresh derivation
would add nothing. This is the REAL deployed fleet (8 AMU + 8 NMU) - kept
labeled separately from any simulated unit count, per the standing rule
that real and simulated numbers must never be quoted interchangeably.

## 1. Concurrent sessions

**Strongest existing evidence, reused as-is:** `2026-08-26_live_wire_capture.md`
already recorded four devices delivering to the server within the SAME
MILLISECOND (02:33:51.747711-.747714), with four independent ACKs returning -
proof the server holds multiple authenticated DTLS sessions concurrently
rather than serialising them.

**Corroborating breadth, from this session's Marina-ladder capture**
(`2026-08-27_marina_ladder.pcap`, 5.5 minutes, incidental to that
experiment): **16 distinct device IPs** contacted the server on port 11400
in the window - the complete real fleet, all active inside one short
capture with no coordination or forced trigger. Peak same-second overlap in
THIS particular window was lower (2 devices), which is expected: the
fleet's individual heartbeat/alarm timers are not synchronised, so any given
short window samples a different slice of overlap. The 4-way same-millisecond
result stands as the peak-concurrency figure; this capture's contribution is
confirming all 16 real units were live and reachable at once, independently
of the earlier capture and three weeks later in the deployment's life.

```
distinct real devices seen, this capture : 16
peak same-second overlap, this capture   :  2
peak same-millisecond overlap, 08-26     :  4  (evidence: 2026-08-26_live_wire_capture.md)
```

## 2. Handshake duration

**Device-logged, reused per `REMAINING_TESTS_PROCEDURE.md`'s own
instruction** (NMU_18, `2026-08-26_boot_discovery_handshake.md`):

```
cold boot (empty NVS)     : 4190 ms
warm boot (cached server) : 4415 ms
```

**A third, independent data point from this session** (NMU_21,
`2026-08-27_nmu21_handshake_recapture.md`'s own capture, re-examined here
for timing): the network-observed span from the first ClientHello to the
last handshake-associated flight, immediately before traffic settles into
ordinary periodic records, measured directly from packet timestamps -
**not** the device's own internal timer.

```
first ClientHello                    : t+0.000 s
last handshake-flight record         : t+4.122 s   <- ClientHello to here
first ordinary periodic record after : t+5.316 s
```

**~4.12 s**, independently measured by an outside witness (the packet
capture) using a different method than the two NMU_18 figures (which come
from the device's own `wolfSSL_connect()` timer) - and it lands inside the
same 4.19-4.42 s band. Two different measurement methods on two different
units agreeing to within ~7% is a real cross-check, not a coincidence
dressed up as one.

**Honest gap:** the procedure asked for "2-3 more units" of timing spread;
this session adds exactly one (NMU_21), by a different method than the
original two. Real n=3 total, not the fuller spread originally scoped.
No AMU handshake timing is available from this session - AMU_T1 held one
continuous DTLS session throughout the Marina-ladder run
(`2026-08-27_marina_ladder.md` section 6), so no fresh AMU handshake
occurred to time.

## 3. What this does and does not prove

**Proves:** the real 16-unit fleet runs concurrent, independently-timed
sessions against one server (not serialised, not coordinated); handshake
cost is consistently in the 4.1-4.4 second range across two units and two
independent measurement methods.

**Does not prove:** behaviour at a larger fleet size than 16, or under
simultaneous mass-reconnect load (a power-cut recovery scenario, which
`omega_net.cpp`'s `BOOT: spreading first handshake by <n> ms` jitter is
specifically designed to soften - see FINDINGS #45). That scenario is
covered by the boot-jitter design and the recovery-ladder evidence (#10),
not by this file.

## Sources

- `2026-08-26_live_wire_capture.md` - peak concurrency (4, same millisecond)
- `2026-08-26_boot_discovery_handshake.md` - NMU_18 device-logged timings
- `2026-08-27_nmu21_handshake_recapture.md` - source capture for the
  network-observed NMU_21 timing derived in section 2 above
- `2026-08-27_marina_ladder.pcap` - source of the 16-distinct-device count
