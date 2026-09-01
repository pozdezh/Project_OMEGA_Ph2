#ifndef OMEGA_TIME_H
#define OMEGA_TIME_H

#include <stdint.h>

// Dating a record that was captured before this unit knew what time it was.
//
// An ESP32 has no battery-backed clock: every boot starts believing it is
// 1970, and the real time arrives only inside an authenticated ACK. A record
// captured before that therefore carries ts = 0 - a confident wrong time
// being worse than an admitted unknown - and the server dates such a record
// by ARRIVAL. That is correct for a live record, where arrival is capture,
// and WRONG for a buffered one, which arrives whenever the link came back.
//
// On 2026-08-27 NMU_19 delivered 84 records captured across roughly twenty
// minutes and every one of them landed inside the same six seconds. The noise
// history for that period was false. The user caught it from the data alone:
// the room had been steady, so the burst could not be real.
//
// The way out is that millis() needs no clock and never jumps. However long a
// record has waited is always knowable, so once the real time is learned the
// capture time is simply now minus that wait.
//
// This lives in its own header, free of Arduino types, so the rule can be
// tested on a PC instead of only on hardware. simlab/test_nmu_time.cpp is
// that test and the gate runs it.
//
// Returns 0 when the time is still genuinely unknown. That is the whole
// point: it never invents a number, it reports that it cannot tell.
// same_boot says whether capture_ms was taken by the boot that is running
// now. It never is for a record restored from flash after a reboot, and the
// elapsed-time arithmetic is meaningless in that case - see OfflinePayload.
static inline unsigned long omegaResolveTimestamp(unsigned long stored_ts,
                                                  uint32_t capture_ms,
                                                  unsigned long now_ms,
                                                  unsigned long now_s,
                                                  bool clock_synced,
                                                  bool same_boot) {
  if (stored_ts != 0UL) {
    return stored_ts;  // Already dated at capture. Never second-guess it.
  }
  if (!clock_synced) {
    return 0UL;  // Still unknown. Say so.
  }
  if (!same_boot) {
    return 0UL;  // The stopwatch belongs to a boot that no longer exists.
  }
  // Done in explicit 32-bit, because that is what millis() is. Relying on
  // `unsigned long` would silently change meaning between the device (32-bit,
  // wraps every ~49.7 days) and any host that tests this (64-bit, does not
  // wrap at all) - the wrap would then be handled correctly on hardware by
  // accident and be untestable off it. The PC test caught exactly this.
  const uint32_t waited_ms = (uint32_t)now_ms - capture_ms;
  const unsigned long waited_s = (unsigned long)(waited_ms / 1000U);
  if (now_s <= waited_s) {
    return 0UL;  // Would land at or before the epoch: nonsense, so refuse.
  }
  return now_s - waited_s;
}

#endif  // OMEGA_TIME_H
