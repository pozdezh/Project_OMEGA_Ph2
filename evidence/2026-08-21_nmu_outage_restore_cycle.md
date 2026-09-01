# Full outage-and-restore cycle on the noise unit, 2026-08-21

The complete round trip: buffer during an outage, lose volatile state, restore
from flash on the next boot, and deliver every record under its original
identity. Run on real hardware after the buffer and queue changes of the same
day were in place.

This is a **different run** from the recovery-ladder observation of 2026-08-27
(`2026-08-27_nmu22_recovery_ladder_serial.md`), which watched the escalation
rungs fire but whose serial capture ended while the buffer was still draining.
This one covers the part that run did not: the drain completing.

## Claim under test

A unit that buffers readings while the server is unreachable, and then loses
its volatile state, restores every buffered record from flash and delivers it
with its original identity rather than as new data.

## Conditions

- Outage induced on a deployed noise unit; records accumulate in the RAM ring
  buffer and are mirrored to flash.
- **Restart performed with RTC memory cleared**, which is the harsher case: it
  is what a loss of power looks like, not a self-healing reboot. The unit
  therefore comes back with no retained session identifier and no retained
  event counter.

## Result

During the outage phase, **67 records** were buffered with `mirrfail=0`, no
lock timeouts, flat heap and no watchdog resets.

On the restart, the boot banner reported the flash mirror being mounted and
read back:

```
BUFFER: mount=1 total=1739681 used=2259 mirror=1 tmp=0 bytes=1927 restored=91
```

**1927 bytes is exactly a 16-byte header plus 91 records of 21 bytes**, so the
restored count is arithmetically consistent with the bytes actually read, not
merely reported by the same code that wrote them.

**All 91 records were delivered in 6 seconds**, under their **original**
session identifier (`858912470`), while the unit's new post-restart session
(`2684943063`) ran alongside them. The record's identity travels with the
record in the flash mirror; it is not reconstructed from the unit's current
state.

`journalctl -u omega-listener` showed **zero `DUPLICATE` lines** across the
whole test.

## What this shows, and what it does not

It shows that the durable path is genuinely durable: identity survives in the
stored record itself, so a unit that has forgotten who it was still delivers
its backlog correctly attributed, and the server's uniqueness index is not
being relied on to hide double delivery.

It does not measure the escalation ladder; that is the 2026-08-27 run. The two
are complementary and neither replaces the other.

## Distinguish the two persistence mechanisms

These are separate and are easy to conflate:

- **RTC retained memory** (`RTC_NOINIT_ATTR` in `nmu/omega_tasks.cpp`) holds
  `bootSession` and `eventCounter`. It survives a software reset, which is why
  a self-healing reboot keeps one continuous session, and it is cleared by a
  loss of power, which is the case tested here.
- **The flash mirror** (`nmu/omega_buffer.cpp`) holds the buffered records,
  each carrying its own session identifier and event number. This is what
  survives the power loss above.
