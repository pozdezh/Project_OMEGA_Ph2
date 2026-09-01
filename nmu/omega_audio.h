#ifndef OMEGA_AUDIO_H
#define OMEGA_AUDIO_H

#include <Arduino.h>

struct AudioCaptureResult {
  float db;
  float duration_s;
  bool hit_max_chunks;
  // How long the trailing quiet period that STOPPED capture lasted. The
  // capture loop keeps sampling until MAX_SILENCE consecutive quiet chunks
  // confirm the sound really ended, so the loop itself always runs longer
  // than the sound did - this is what lets the timestamp be corrected back
  // to when the sound actually stopped, not when the loop gave up waiting.
  float silence_tail_s;
};

bool audioInit();
float audioAmbientPoll();
float audioAmbientDb();
float audioTriggerThresholdV(float ambient_db);
void audioUpdateBaseline(float current_p2p_v);
void audioResetBaseline(float current_p2p_v);
AudioCaptureResult audioCaptureSyncMeasurement();
AudioCaptureResult audioCaptureTriggeredEvent(float ambient_db);

#endif  // OMEGA_AUDIO_H
