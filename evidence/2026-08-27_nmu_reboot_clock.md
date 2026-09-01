# NMU keeps its clock and identity through a self-heal reboot, and honestly
# loses both through a power cut (2026-08-27)

Unit: NMU_22. Firmware: commit e931bd8. Gate: PASS, 27 groups.

## Why this experiment exists

The NMU reboots itself as the top rung of its recovery ladder. It has no
battery-backed clock, so the time it knows arrived from the server inside an
authenticated ACK. If that is lost on the reboot, every reading taken before
the link returns is undatable - and the reboot happens precisely when the
link is already broken. The claim under test is that a HEALING reboot keeps
the time while a POWER CUT does not, and that the unit can tell the two
apart.

## Method

A `reboot` command was added to the NMU mailbox, so the restart is triggered
on demand instead of by waiting out a 17.5-minute outage. The unit answers
the command and restarts only after that answer is acknowledged, so an
answered reboot is a confirmed reboot. Serial is captured continuously across
the restart; the board's USB port renumbers on every reset, so the capture
reattaches rather than holding one handle. The server database is queried
independently - the device's own log is not accepted as proof of what was
stored.

## Result A - software restart (the self-heal case)

    04:32:57 Net: operator requested restart - flushing and rebooting
    04:33:00 BOOT: reset_reason=3
    04:33:00 Clock: 1787797979 survived the reset - still trusted
    04:33:00 BOOT: clock_before_floor=1787797979
    04:33:00 BOOT: clock=1787797979 (build floor applied)
    04:33:00 BOOT: session=751636677 resuming at event=11

Server side, one unbroken session across the restart:

    04:32:57  751636677_11     last before
    04:33:34  751636677_12     first after
    ... through _15, no gap, no duplicate, no future-dated row

## Result B - power cut (board unplugged and replugged)

    04:43:53 BOOT: reset_reason=1
    04:43:53 Clock: RTC state uninitialised (power-on) - time unknown
             until the server answers
    04:43:53 BOOT: clock_before_floor=2
    04:43:53 BOOT: clock=1787718338 (build floor applied)
    04:43:54 BOOT: session=4254210621 resuming at event=0

`clock_before_floor=2` - two seconds past 1970 - is the load-bearing line.
The memory really was gone, and the unit reported that rather than trusting a
flag. It re-established the session 31 seconds later and its first five
records carry correct times:

    04:44:24  4254210621_1 .. _3
    04:44:25  4254210621_4
    04:44:26  4254210621_5
    future-dated rows in this session: 0

The floor applied on the cold boot (1787718338) is roughly 22 hours BEFORE
real time, which is what a floor is supposed to be. Before this fix it was
two hours AFTER real time and overwrote a correct clock.

## Negative control

The pre-fix firmware's damage is still visible in the same table and is
absent from every post-fix session:

    session 4207522588 (pre-fix)   06:14:16, 06:14:25, 06:14:36 - 3 future rows
    session 751636677  (post-fix)  0 future rows
    session 4254210621 (post-fix)  0 future rows

One bad row per reboot, dated by a floor that sat in the future.

## What this does not prove

The gate cannot see radio or power faults, and this experiment used a single
unit. Fleet-scale behaviour under simultaneous restarts is experiment #12.

Root cause and the four defects behind it: FINDINGS #59.

## Cleanup of the control rows (2026-08-27 05:08)

The three future-dated rows above were deleted from the live database once
they had served as the negative control. They were not merely cosmetic: every
`MAX(timestamp)` query - including the dashboard's "last seen" - reported
NMU_22 as having reported an hour and a half in the future.

Deleted, after printing them for the record:

    06:14:16  4207522588_21
    06:14:25  4207522588_32
    06:14:36  4207522588_43

Fleet-wide future-dated rows afterwards: 0. NMU_22 last seen 04:55:48, which
is true. The values are preserved here rather than in the table, because a
demonstration of a fixed bug does not belong in the measurement data the
memo draws on.

## Fleet rollout (2026-08-27 04:55 - 05:06)

All eight units flashed with this firmware, each with its own certificate
pulled fresh from the CA, each confirmed by the SERVER to have reported in
rather than by the flasher's own "4/4 regions verified":

    NMU_T1  reported after 14s      NMU_19  after  7s
    NMU_16  reported after  7s      NMU_20  after 21s
    NMU_17  reported after 14s      NMU_21  after  7s
    NMU_18  reported after  7s      NMU_22  proof unit, see above

Future-dated rows on every one of them: 0.

One near-miss worth recording. The first attempt asked for "NMU_15", which
has no CA identity. The staging script correctly refused - but the driver
script piped its output, and a pipe hides the exit status of the command on
its left, so `set -e` never saw the failure and the board was flashed with
the certificate still sitting in the sketch folder: NMU_22's. The
server-side check caught it, because a flash that verifies 4/4 regions
proves nothing about whether the unit can join the fleet. The board was
NMU_T1 and was reflashed correctly. This is the same class as FINDINGS #56,
reached by a different route.
