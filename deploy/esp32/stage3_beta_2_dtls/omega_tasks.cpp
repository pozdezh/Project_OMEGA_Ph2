#include "omega_tasks.h"
#include "omega_config.h"
#include "omega_audio.h"
#include "omega_buffer.h"
#include "omega_net.h"

#include <Preferences.h>
#include <esp_task_wdt.h>

TaskHandle_t netTaskHandle = NULL;

static QueueHandle_t networkQueue = NULL;
static uint32_t queueDisplacedCount = 0;

// RTC_NOINIT_ATTR for the same reason as the clock flag in omega_net.cpp:
// RTC_DATA_ATTR is reloaded from flash on a software reset, so a unit that
// rebooted to heal itself came back as a brand new session and restarted its
// event numbering. Continuity across a self-heal is the whole point.
static RTC_NOINIT_ATTR uint32_t sessionStateMagic;
// Distinct from bootSession, which deliberately SURVIVES a reboot so the
// server sees one continuous session. This changes on every boot, and is
// what tells a restored record that its millis() stopwatch is stale.
static RTC_NOINIT_ATTR uint32_t bootId;
static RTC_NOINIT_ATTR uint32_t eventCounter;
static RTC_NOINIT_ATTR uint32_t bootSession;
static RTC_NOINIT_ATTR int missedAckStreak;

static unsigned long lastDiscoveryAttempt = 0;
static unsigned long lastHeartbeat = 0;

// See FINDINGS #54 and ARCHITECTURE.md 25.
static unsigned long lastServerContactMs = 0;
static unsigned long lastClockProbe = 0;
static uint32_t radioResets = 0;
// Lifetime count, never cleared. radioResets is now cleared on contact
// so the rung re-arms, which would otherwise have made the diagnostic
// line report 0 forever and hidden how often a unit is struggling.
static uint32_t radioResetsTotal = 0;
static unsigned long lastBacklogAttempt = 0;
static uint32_t backlogRetryMs = BACKLOG_RETRY_MIN_MS;

static volatile bool netNeedsSpeed = false;

static void sentrySetCpu(int mhz) {
#if CPU_FREQ_SCALING_ENABLED
  if (netNeedsSpeed) return;
  setCpuFrequencyMhz(mhz);
#else
  (void)mhz;
#endif
}

static void netClaimCpu(bool claim) {
#if CPU_FREQ_SCALING_ENABLED
  if (claim) {
    netNeedsSpeed = true;
    setCpuFrequencyMhz(CPU_MHZ_CRYPTO);
  } else {
    netNeedsSpeed = false;
    setCpuFrequencyMhz(CPU_MHZ_IDLE);
  }
#else
  (void)claim;
#endif
  netSetRadioSleep(!claim);
}

static bool enqueueEventNewestWins(const OfflinePayload& event) {
  if (xQueueSend(networkQueue, &event, 0) == pdPASS) {
    return true;
  }
  OfflinePayload displaced;
  if (xQueueReceive(networkQueue, &displaced, 0) == pdPASS) {
    bufferSave(displaced);
    queueDisplacedCount++;
  }
  if (xQueueSend(networkQueue, &event, 0) == pdPASS) {
    return true;
  }
  // Nothing may be dropped on the floor. If even the freed slot could not be
  // claimed, the event goes to flash instead of being lost - the AMU has
  // always done this and the two device types must not disagree about what
  // happens to a reading. Only the ring being full may overwrite anything,
  // and it overwrites the OLDEST.
  bufferSave(event);
  return false;
}

static void noteServerContact() {
  lastServerContactMs = millis();
  // Re-arm the cheap rung. radioResets only ever incremented, and the rung is
  // guarded by `radioResets == 0`, so before 2026-08-27 a unit got exactly ONE
  // radio reset per boot for the rest of its life: every later outage in that
  // boot had to wait out the full reboot rung instead. A rung that stops
  // working after it is used once is not a recovery ladder.
  radioResets = 0;
}

static unsigned long outageRung(uint32_t floor_ms, uint32_t beats_x10) {
  const unsigned long derived = (heartbeatInterval / 10UL) * beats_x10;
  return derived > floor_ms ? derived : (unsigned long)floor_ms;
}

static void enforceOutageRecovery() {
  if (lastServerContactMs == 0) {
    lastServerContactMs = millis();
    return;
  }
  const unsigned long silent = millis() - lastServerContactMs;
  const unsigned long radioAt =
      outageRung(OUTAGE_RADIO_RESET_MS, OUTAGE_RADIO_RESET_BEATS_X10);
  const unsigned long rebootAt =
      outageRung(OUTAGE_REBOOT_MS, OUTAGE_REBOOT_BEATS_X10);

  if (silent > rebootAt) {
    Serial.printf("Net: no server contact for %lus after %u radio reset(s) - "
                  "rebooting (buffer is mirrored to flash)\n",
                  silent / 1000UL, (unsigned)radioResets);
    Serial.flush();
    bufferSyncMirror();
    ESP.restart();
  }

  if (silent > radioAt && radioResets == 0) {
    Serial.printf("Net: no server contact for %lus - full radio reset\n",
                  silent / 1000UL);
    radioResets++;
    radioResetsTotal++;
    killWiFi();
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_task_wdt_reset();
    serverFound = false;
    lastDiscoveryAttempt = 0;
  }
}

static void runAggressiveReconnect() {
  unsigned long start = millis();
  while (!serverFound && (millis() - start < AGGRESSIVE_RECONNECT_WINDOW_MS)) {
    // The ladder outranks this loop. Without this line the loop owned the
    // unit for a full five minutes while the outage clock kept running, so a
    // rung due at 450 s actually fired at 740 s - measured on NMU_22,
    // 2026-08-27. A recovery ladder that a retry loop can postpone is not a
    // ladder, it is a suggestion.
    enforceOutageRecovery();
    ensureSession();
    if (serverFound) {
      return;
    }
    killWiFi();
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_task_wdt_reset();
  }
}

static void delayJitter(uint32_t min_ms, uint32_t max_ms) {
  const unsigned long span = (unsigned long)random(min_ms, max_ms);
  const unsigned long start = millis();
  while (millis() - start < span) {
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
  }
}

// Empties the offline buffer oldest-first. A record is popped only once ITS
// OWN ACK arrived, so an interrupted drain loses nothing and resumes later.
// Returns true when the buffer is empty on exit.
static bool drainBacklog() {
  OfflinePayload bPL;
  int slot = bufferPeekOldest(bPL);
  bool drained = false;
  while (slot != BUFFER_NO_SLOT) {
    esp_task_wdt_reset();
    if (deliverRecord(bPL.session, bPL.event_cnt,
                      netResolveTimestamp(bPL.ts, bPL.capture_ms, bPL.boot_id == bootId),
                      false, bPL.db, bPL.duration)) {
      missedAckStreak = 0;
      noteServerContact();
      bufferPopOldest(slot);
      drained = true;
      vTaskDelay(pdMS_TO_TICKS(50));
      slot = bufferPeekOldest(bPL);
    } else {
      serverFound = false;
      missedAckStreak++;
      netCloseSession();
      break;
    }
  }
  if (drained) {
    bufferSyncMirror();
  }
  return bufferCount() == 0;
}

static void processAndSend(OfflinePayload pl) {
  netClaimCpu(true);

  if (!serverFound) {
    delayJitter(RECONNECT_JITTER_MIN_MS, RECONNECT_JITTER_MAX_MS);
  }

  if (!ensureSession()) {
    Serial.printf("Net: no session, buffering event=%u\n", pl.event_cnt);
    bufferSave(pl);
    killWiFi();
    netClaimCpu(false);
    return;
  }

  // THE LIVE EVENT GOES FIRST, ahead of any history.
  //
  // It used to go last, behind a full drain of up to 100 buffered records.
  // That put a fresh event behind all of them, and worse: a drain that failed
  // part-way cleared serverFound, and the live event was then buffered
  // WITHOUT EVER BEING SENT - the one event that might matter was the one
  // guaranteed not to go out. History can wait; the present cannot.
  const float liveDb = pl.isHeartbeat ? 0.0f : pl.db;
  const float liveDuration = pl.isHeartbeat ? 0.0f : pl.duration;
  const bool ackReceived = deliverRecord(pl.session, pl.event_cnt,
                                         netResolveTimestamp(pl.ts, pl.capture_ms, pl.boot_id == bootId),
                                         pl.isHeartbeat, liveDb, liveDuration);
  Serial.printf("Net: event=%u_%u ACK %s\n", pl.session, pl.event_cnt,
                ackReceived ? "OK" : "FAILED (buffering)");

  if (ackReceived) {
    missedAckStreak = 0;
    noteServerContact();
    if (!pl.isHeartbeat) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
    drainBacklog();
    if (netRebootRequested()) {
      Serial.println("Net: operator requested restart - flushing and rebooting");
      Serial.flush();
      bufferSyncMirror();
      ESP.restart();
    }
  } else {
    serverFound = false;
    missedAckStreak++;
    bufferSave(pl);
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
  }

  if (!netSessionConnected()) {
    killWiFi();
  }

  netClaimCpu(false);
}

void tasksInit() {
  networkQueue = xQueueCreate(NETWORK_QUEUE_DEPTH, sizeof(OfflinePayload));
  if (sessionStateMagic != RTC_STATE_MAGIC) {
    sessionStateMagic = RTC_STATE_MAGIC;
    bootSession = 0;
    eventCounter = 0;
    missedAckStreak = 0;
    bootId = 0;
  }
  bootId++;
  if (bootSession == 0) {
    bootSession = esp_random();
  }
  Serial.printf("BOOT: session=%u resuming at event=%u\n",
                (unsigned)bootSession, (unsigned)eventCounter);
}

void tasksMarkBootComplete() {
  lastDiscoveryAttempt = millis();
  lastHeartbeat = millis();
  lastClockProbe = 0;
}

void NetworkTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  OfflinePayload incomingItem;

  for (;;) {
    esp_task_wdt_reset();
    unsigned long now = millis();

    enforceOutageRecovery();

    if (missedAckStreak >= MAX_MISSED_ACKS) {
      Serial.println("Net: 5 missed ACKs - aggressive reconnect.");
      serverFound = false;
      missedAckStreak = 0;
      killWiFi();
      lastDiscoveryAttempt = 0;
      runAggressiveReconnect();
    }

    if (!serverFound && (now - lastDiscoveryAttempt > SYNC_RETRY_INTERVAL_MS)) {
      ensureSession();
      if (!serverFound) {
        killWiFi();
      }
      lastDiscoveryAttempt = millis();
    }

    // A buffered event must not wait for the next noise event to carry
    // it out. A quiet room can stay quiet for a whole heartbeat period,
    // and an alarm parked on flash for that long is a lost alarm.
    if (bufferCount() > 0 && (now - lastBacklogAttempt) > backlogRetryMs) {
      lastBacklogAttempt = now;
      netClaimCpu(true);
      const bool emptied = ensureSession() && drainBacklog();
      if (emptied) {
        backlogRetryMs = BACKLOG_RETRY_MIN_MS;
      } else if (backlogRetryMs * 2 > BACKLOG_RETRY_MAX_MS) {
        backlogRetryMs = BACKLOG_RETRY_MAX_MS;
      } else {
        backlogRetryMs = backlogRetryMs * 2;
      }
      if (!netSessionConnected()) {
        killWiFi();
      }
      netClaimCpu(false);
    }

    // Ask the time first. A heartbeat is the cheapest record that earns an
    // authenticated ACK, and the ACK is what carries the server's clock.
    if (serverFound && !netClockSynced() &&
        (now - lastClockProbe) > CLOCK_PROBE_INTERVAL_MS) {
      lastClockProbe = now;
      eventCounter++;
      OfflinePayload probe = {0UL, bootSession, eventCounter, 0.0f, 0.0f, true,
                              (uint32_t)millis(), bootId};
      processAndSend(probe);
      lastHeartbeat = millis();
    }

    if (now - lastHeartbeat > heartbeatInterval) {
      time_t tNow;
      time(&tNow);
      eventCounter++;
      unsigned long ts = netClockSynced() ? (unsigned long)tNow : 0;
      OfflinePayload hb = {ts, bootSession, eventCounter, 0.0f, 0.0f, true,
                           (uint32_t)millis(), bootId};
      processAndSend(hb);
      lastHeartbeat = millis();
    }

    if (xQueueReceive(networkQueue, &incomingItem, pdMS_TO_TICKS(1000)) == pdPASS) {
      processAndSend(incomingItem);
      lastHeartbeat = millis();
    }
  }
}

void SentryTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  int diagTickCounter = 0;
  bool syncMeasurementPending = true;
  unsigned long syncArmedMs = 0;

  for (;;) {
    esp_task_wdt_reset();

    if (!serverFound) {
      syncMeasurementPending = true;
      syncArmedMs = 0;
    } else if (syncMeasurementPending && syncArmedMs == 0) {
      syncArmedMs = millis();
    } else if (syncMeasurementPending &&
               (netClockSynced() ||
                (millis() - syncArmedMs) > SYNC_CLOCK_WAIT_MS)) {
      sentrySetCpu(CPU_MHZ_CAPTURE);
      AudioCaptureResult sync = audioCaptureSyncMeasurement();
      sentrySetCpu(CPU_MHZ_IDLE);

      time_t nowT;
      time(&nowT);
      eventCounter++;
      unsigned long ts = netClockSynced() ? (unsigned long)nowT : 0;
      OfflinePayload syncEvent = {ts, bootSession, eventCounter, sync.db,
                                  sync.duration_s, false, (uint32_t)millis(), bootId};
      enqueueEventNewestWins(syncEvent);
      Serial.printf("Sentry: initial sync measurement queued, db=%.2f, event=%u\n",
                    syncEvent.db, eventCounter);
      syncMeasurementPending = false;
    }

    sentrySetCpu(CPU_MHZ_IDLE);
    float currentP2pV = audioAmbientPoll();
    float ambientDb = audioAmbientDb();
    float triggerThresholdV = audioTriggerThresholdV(ambientDb);

    if (currentP2pV > triggerThresholdV) {
      sentrySetCpu(CPU_MHZ_CAPTURE);
      AudioCaptureResult captured = audioCaptureTriggeredEvent(ambientDb);

      time_t nowT;
      time(&nowT);
      eventCounter++;
      unsigned long stoppedAt = (unsigned long)nowT -
          (unsigned long)(captured.silence_tail_s + 0.5f);
      unsigned long ts = netClockSynced() ? stoppedAt : 0;
      OfflinePayload newEvent = {ts, bootSession, eventCounter, captured.db,
                                 captured.duration_s, false, (uint32_t)millis(), bootId};
      enqueueEventNewestWins(newEvent);
      Serial.printf("Sentry: TRIGGERED db=%.2f duration=%.2fs event=%u\n",
                    captured.db, captured.duration_s, eventCounter);

      if (captured.hit_max_chunks) {
        audioResetBaseline(currentP2pV);
        vTaskDelay(pdMS_TO_TICKS(10000));
      }
    } else {
      audioUpdateBaseline(currentP2pV);
    }

    diagTickCounter++;
    if (diagTickCounter > 26) {
      diagTickCounter = 0;

      Serial.printf("Sentry: alive, ambient=%.1fdB serverFound=%d heap=%u "
                    "stack_free[sentry]=%u stack_free[net]=%u "
                    "buffered=%d qdisplaced=%u mirrfail=%u lastreset=%d uptime=%us "
                    "ps=%d handshakes=%u silent=%lus rreset=%u stale=%u\n",
                    ambientDb, serverFound, (unsigned)ESP.getFreeHeap(),
                    (unsigned)uxTaskGetStackHighWaterMark(NULL),
                    (unsigned)(netTaskHandle ? uxTaskGetStackHighWaterMark(netTaskHandle) : 0),
                    bufferCount(), (unsigned)queueDisplacedCount,
                    (unsigned)bufferMirrorFailures(),
                    (int)esp_reset_reason(), (unsigned)(millis() / 1000),
                    netRadioSleepMode(), (unsigned)netHandshakeCount(),
                    lastServerContactMs ? (millis() - lastServerContactMs) / 1000UL : 0UL,
                    (unsigned)radioResetsTotal, (unsigned)netStaleAcks());

      Preferences bc;
      if (bc.begin("omega_pm", false)) {
        bc.putUInt("last_uptime", (uint32_t)(millis() / 1000));
        bc.end();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SENTRY_LOOP_DELAY_MS));
  }
}
