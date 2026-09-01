// Proof for the NMU record-dating rule, run on this PC by the gate.
//
// The NMU is C++ on hardware, so its logic normally cannot be tested without
// a board on the desk. omega_time.h exists to make this one rule an exception:
// it is the part that got the timestamps wrong, so it is the part that has to
// be provable without hardware.
//
// Built and run by simlab/run_gate.py. To run it by hand:
//     g++ -std=c++11 -I ../nmu -o /tmp/t test_nmu_time.cpp && /tmp/t

#include "omega_time.h"

#include <cstdio>
#include <cstdlib>

static int failures = 0;

static void check(bool ok, const char* what) {
  if (ok) {
    printf("  ok    %s\n", what);
  } else {
    printf("  FAIL  %s\n", what);
    failures++;
  }
}

static void check_eq(unsigned long got, unsigned long want, const char* what) {
  if (got == want) {
    printf("  ok    %s\n", what);
  } else {
    printf("  FAIL  %s (got %lu, want %lu)\n", what, got, want);
    failures++;
  }
}

// A plausible wall clock: 27 Aug 2026, near midnight.
static const unsigned long NOW_S = 1787790000UL;

int main() {
  printf("NMU record dating\n");

  // A record that already knows its own time is never second-guessed. This
  // is the overwhelmingly common case: the clock was synced at capture.
  check_eq(omegaResolveTimestamp(1787780000UL, 5000, 600000UL, NOW_S, true, true),
           1787780000UL,
           "a record dated at capture keeps its own timestamp");

  // The rule that makes the whole thing honest: when the time is still
  // unknown, say so. Do not invent a number that will be believed.
  check_eq(omegaResolveTimestamp(0UL, 5000, 600000UL, NOW_S, false, true), 0UL,
           "an unknown time stays unknown, never a guess");

  // The repair itself. Captured 20 minutes ago while the clock was unknown,
  // delivered now that it is known: it must be dated 20 minutes ago.
  const unsigned long captured_at_ms = 100000UL;
  const unsigned long delivered_at_ms = captured_at_ms + 1200000UL;  // +20 min
  check_eq(omegaResolveTimestamp(0UL, (uint32_t)captured_at_ms,
                                 delivered_at_ms, NOW_S, true, true),
           NOW_S - 1200UL,
           "a record buffered 20 min is dated 20 min ago, not now");

  // THE NMU_19 FAILURE, REPLAYED.
  //
  // 84 records captured across ~20 minutes while the server rejected the
  // unit, then drained in 6 seconds once it was fixed. Under the old rule
  // every one was stamped with its arrival time and the whole 20 minutes
  // collapsed into 6 seconds. Here they must stay spread out.
  const int kRecords = 84;
  const unsigned long span_ms = 1200000UL;      // captured over 20 minutes
  const unsigned long drain_start_ms = 2000000UL;
  unsigned long first = 0, last = 0;
  bool monotonic = true;
  unsigned long previous = 0;
  for (int i = 0; i < kRecords; i++) {
    const unsigned long capture_ms = (span_ms * (unsigned long)i) / (kRecords - 1);
    // Drained fast: the whole backlog goes out inside six seconds.
    const unsigned long now_ms = drain_start_ms + (6000UL * (unsigned long)i) / (kRecords - 1);
    const unsigned long dated =
        omegaResolveTimestamp(0UL, (uint32_t)capture_ms, now_ms, NOW_S, true, true);
    if (i == 0) {
      first = dated;
    } else if (dated < previous) {
      monotonic = false;
    }
    previous = dated;
    last = dated;
  }
  const unsigned long spread = last - first;
  printf("  ..    84 records spread over %lu s of history (drained in 6 s)\n",
         spread);
  check(spread >= 1190UL && spread <= 1210UL,
        "a 20 min backlog stays 20 min wide after a 6 s drain");
  check(monotonic, "restored order is never inverted");

  // Without the fix the same backlog would have collapsed. Stated as a test
  // so the regression is visible, not just described in a comment.
  check(spread > 60UL,
        "the pre-fix behaviour (all 84 inside one minute) is impossible now");

  // millis() wraps to zero about every 49.7 days. Unsigned subtraction has to
  // carry the unit through it: a wrap must not date a record in the future or
  // fling it into the past.
  const unsigned long before_wrap_ms = 0xFFFFF000UL;
  const unsigned long after_wrap_ms = 0x00001000UL;  // 8192 ms later
  check_eq(omegaResolveTimestamp(0UL, (uint32_t)before_wrap_ms, after_wrap_ms,
                                 NOW_S, true, true),
           NOW_S - 8UL,
           "a millis() wrap during the wait costs nothing");

  // A RECORD THAT OUTLIVED THE BOOT THAT MEASURED IT.
  // The ring buffer is mirrored to flash, so a self-heal reboot hands the next
  // boot records whose capture_ms was measured against a millis() that no
  // longer exists. Reusing it underflows the subtraction: with capture_ms
  // 900000 (15 min into the old boot) and now_ms 60000, the "wait" comes out
  // as 49.7 days and the record lands weeks in the past - stated confidently.
  // Found on NMU_22, 2026-08-27, while proving the RTC clock fix.
  check_eq(omegaResolveTimestamp(0UL, 900000U, 60000UL, NOW_S, true, false),
           0UL,
           "a record from a previous boot is undatable, not misdated");

  // The same record IS datable when its own boot is still running.
  check_eq(omegaResolveTimestamp(0UL, 900000U, 960000UL, NOW_S, true, true),
           NOW_S - 60UL,
           "same boot, so the stopwatch still means something");

  // A dated record is unaffected either way: a real capture time outranks
  // every stopwatch, including a stale one.
  check_eq(omegaResolveTimestamp(1787780000UL, 900000U, 60000UL, NOW_S, true, false),
           1787780000UL,
           "a record dated at capture survives the reboot with its date");

  // A clock that would put the record at or before the epoch is nonsense,
  // and nonsense must be refused rather than stored.
  check_eq(omegaResolveTimestamp(0UL, 0, 5000000UL, 4000UL, true, true), 0UL,
           "a timestamp that would predate the epoch is refused");

  // Delivered in the same instant it was captured: the live path. Must be
  // now, not now minus something.
  check_eq(omegaResolveTimestamp(0UL, 50000, 50000UL, NOW_S, true, true), NOW_S,
           "a record sent the moment it is captured is dated now");

  if (failures) {
    printf("\n%d check(s) FAILED\n", failures);
    return 1;
  }
  printf("\nall checks passed\n");
  return 0;
}
