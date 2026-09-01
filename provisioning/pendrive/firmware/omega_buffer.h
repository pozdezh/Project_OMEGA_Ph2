#ifndef OMEGA_BUFFER_H
#define OMEGA_BUFFER_H

#include <Arduino.h>

struct __attribute__((packed)) OfflinePayload {
  unsigned long ts;
  uint32_t session;
  uint32_t event_cnt;
  float db;
  float duration;
  bool isHeartbeat;
  // millis() at the moment of capture. This exists so a record captured
  // BEFORE this unit learned the time can still be dated correctly later.
  //
  // ts is 0 while the clock is unsynced (an ESP32 boots believing it is
  // 1970, and a confident wrong time is worse than none). The server then
  // stamps such a record with its ARRIVAL time - which is right for a live
  // record and wrong for a buffered one. On 2026-08-27 NMU_19 delivered a
  // backlog of 84 records, captured over ~20 minutes, all stamped into the
  // same 6 seconds. The noise history for that period was simply false.
  //
  // millis() needs no clock and never jumps, so the ELAPSED time since
  // capture is always knowable. Once the clock syncs, real capture time is
  // just now minus that elapsed time. See netResolveTimestamp().
  uint32_t capture_ms;
  // Which boot that millis() reading belongs to. millis() restarts at 0 on
  // every reboot, and the mirror on flash outlives reboots, so a restored
  // record's capture_ms is meaningless unless this matches the running boot.
  // Without it the elapsed-time arithmetic underflows and dates a record
  // weeks into the past - a confident wrong answer, which is the one outcome
  // this whole mechanism exists to avoid.
  uint32_t boot_id;
};

static const int BUFFER_NO_SLOT = -1;


void bufferInit();
void bufferSave(const OfflinePayload& payload);
int bufferCount();
int bufferPeekOldest(OfflinePayload& out);
void bufferPopOldest(int slot_token);
void bufferSyncMirror();
uint32_t bufferMirrorFailures();

#endif  // OMEGA_BUFFER_H
