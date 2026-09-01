#ifndef OMEGA_NET_H
#define OMEGA_NET_H

#include <Arduino.h>

extern bool serverFound;
extern String deviceID;
extern unsigned long heartbeatInterval;
// Restores the heartbeat the server last taught this unit. Call before
// the network task starts: the recovery rungs are derived from it.
void netLoadPersistedHeartbeat();

// Confirms that a clock which survived a software reset is actually still
// there. Call in setup() BEFORE any build-time floor is applied, or the floor
// hides the very thing this checks for.
void netValidateRestoredClock();

// The real capture time of a record that was stamped 0 because this unit
// did not know the time yet. Returns stored_ts unchanged when it is
// already valid, and 0 when the clock is STILL unknown - never a guess.
unsigned long netResolveTimestamp(unsigned long stored_ts, uint32_t capture_ms,
                                  bool same_boot);

int netInit();
bool ensureSession();
void killWiFi();
void netCloseSession();
bool deliverRecord(uint32_t session, uint32_t counter, unsigned long ts, bool hb,
                   float db, float duration);
uint32_t netLastHandshakeMs();
uint32_t netHandshakeCount();
uint32_t netStaleAcks();
bool netSessionConnected();
void netSetRadioSleep(bool enable);
int netRadioSleepMode();
bool netClockSynced();

// True once an operator has asked this unit to restart. The caller performs
// the restart only after a record has been acknowledged, so the answer to the
// request reaches the server before the radio goes down.
bool netRebootRequested();

#endif  // OMEGA_NET_H
