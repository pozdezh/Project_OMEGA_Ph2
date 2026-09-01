#include "omega_net.h"
#include "omega_config.h"
#include "config.h"
#include "omega_certs.h"
#include "omega_dtls.h"
#include "omega_discovery.h"
#include "omega_audio.h"
#include "omega_buffer.h"
#include "omega_time.h"

#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <string.h>

bool serverFound = false;
String deviceID = OMEGA_DEVICE_ID;
unsigned long heartbeatInterval = DEFAULT_HEARTBEAT_INTERVAL_MS;

// The learned heartbeat has to survive a reboot, because the recovery rungs
// are derived from it and a reboot is exactly when they matter. Without this
// a unit came back up using the 30 min compiled default instead of the 20 min
// the server had told it, which pushed its self-reboot rung out to 105
// minutes - it became MORE patient by rebooting, not less. Same defect the
// AMU had (heartbeat_state.json, fixed 2026-08-26); worse here because the
// default was higher than the real value rather than lower.
static const char OMEGA_PREFS_NS[] = "omega_pm";
static const char HEARTBEAT_KEY[] = "hb_ms";

static void persistHeartbeat(unsigned long value_ms) {
  Preferences prefs;
  if (!prefs.begin(OMEGA_PREFS_NS, false)) {
    return;
  }
  if (prefs.getULong(HEARTBEAT_KEY, 0UL) != value_ms) {
    prefs.putULong(HEARTBEAT_KEY, value_ms);
  }
  prefs.end();
}

void netLoadPersistedHeartbeat() {
  Preferences prefs;
  if (!prefs.begin(OMEGA_PREFS_NS, true)) {
    return;
  }
  const unsigned long stored = prefs.getULong(HEARTBEAT_KEY, 0UL);
  prefs.end();
  // Zero means nothing was ever stored. Anything absurd is refused rather
  // than acted on: a corrupt value here silently retunes the whole ladder.
  if (stored >= 60000UL && stored <= 7200000UL) {
    heartbeatInterval = stored;
    Serial.printf("Net: heartbeat %lus restored from flash\n", stored / 1000UL);
  }
}

static OmegaDtls dtls;
static OmegaServer omegaServer = {"", OMEGA_SERVER_PORT, false};
static volatile bool wifiInUse = false;
static RTC_DATA_ATTR int lastCfgVer = -1;
static unsigned long lastExchangeMs = 0;
static uint32_t rejectedServers = 0;

// RTC_NOINIT_ATTR, not RTC_DATA_ATTR, and the distinction is the whole point.
//
// The intent is to keep the clock across a HEALING REBOOT and lose it only
// when power is actually removed - the difference between the two ways an NMU
// can restart. As a plain static the flag was cleared by every reset, so a
// unit that rebooted itself to recover threw away a perfectly good clock and
// stamped its readings 0 until the server answered again.
//
// RTC_DATA_ATTR does NOT achieve that, which cost a full test cycle to learn.
// It is an INITIALISED section: the bootloader reloads it from the flash image
// on every reset that is not a deep-sleep wake, so the flag came back false
// anyway. Measured on NMU_22, 2026-08-27: a self-heal restart at 03:58:47
// changed the session id from 2990672037 to 516844122, and bootSession is
// regenerated only when it reads back zero. Only RTC_NOINIT_ATTR is left
// untouched by the bootloader.
//
// The cost of a section that is never initialised is that a genuine power-on
// starts with whatever happened to be in that RAM, so it is guarded by a magic
// word. Wrong magic means the contents are garbage, not history.
static RTC_NOINIT_ATTR uint32_t clockStateMagic;
static RTC_NOINIT_ATTR bool clockSyncedFromServer;
static unsigned long staleSessionMs = 120000UL;
static char pendingReplyJson[96] = "";
static volatile bool rebootRequested = false;

static void answerPendingQuery(const char* cmd) {
  unsigned long ts = netClockSynced() ? (unsigned long)time(nullptr) : 0;
  if (strcmp(cmd, "read_now") == 0) {
    snprintf(pendingReplyJson, sizeof(pendingReplyJson),
             "{\"db\":%.2f,\"buffered\":%d,\"ts\":%lu}",
             audioAmbientDb(), bufferCount(), ts);
  } else if (strcmp(cmd, "status") == 0) {
    snprintf(pendingReplyJson, sizeof(pendingReplyJson),
             "{\"heap\":%u,\"buffered\":%d,\"uptime_ms\":%lu,\"ts\":%lu}",
             (unsigned)ESP.getFreeHeap(), bufferCount(), millis(), ts);
  } else if (strcmp(cmd, "reboot") == 0) {
    // Answered first, acted on after the answer has actually been delivered
    // (see netRebootRequested). Restarting inside this call would drop the
    // reply and leave the operator unable to tell a reboot from a crash.
    snprintf(pendingReplyJson, sizeof(pendingReplyJson),
             "{\"rebooting\":true,\"buffered\":%d}", bufferCount());
    rebootRequested = true;
  } else {
    snprintf(pendingReplyJson, sizeof(pendingReplyJson),
             "{\"error\":\"unknown command\"}");
  }
}

static void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  wifiInUse = true;
  WiFi.mode(WIFI_STA);
  vTaskDelay(pdMS_TO_TICKS(10));
  WiFi.begin(OMEGA_WIFI_SSID, OMEGA_WIFI_PASS);
  unsigned long start = millis();
  // WL_CONNECTED alone is not enough: it can be reported before DHCP has
  // produced an address, and a UDP probe sent then leaves on no interface and
  // is silently lost. See FINDINGS #57.
  while (millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress((uint32_t)0)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_task_wdt_reset();
  }
  Serial.printf("WiFi: status=%d ip=%s rssi=%d bcast=%s\n", (int)WiFi.status(),
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
                WiFi.broadcastIP().toString().c_str());
}

static void applyAckConfig(const char* json) {
  StaticJsonDocument<384> cfg;
  if (deserializeJson(cfg, json)) {
    return;
  }
  if (cfg.containsKey("hb")) {
    const unsigned long learned =
        (unsigned long)cfg["hb"].as<unsigned long>() * 60000UL;
    if (learned != heartbeatInterval) {
      heartbeatInterval = learned;
      persistHeartbeat(learned);
    }
  }
  if (cfg.containsKey("cfg_ver")) {
    lastCfgVer = cfg["cfg_ver"].as<int>();
  }
  if (cfg.containsKey("idle")) {
    staleSessionMs = (unsigned long)(cfg["idle"].as<unsigned long>() * 800UL);
  }
  if (cfg.containsKey("t")) {
    const unsigned long server_time = cfg["t"].as<unsigned long>();
    if (server_time > TIME_VALID_EPOCH) {
      struct timeval now;
      now.tv_sec = (time_t)server_time;
      now.tv_usec = 0;
      settimeofday(&now, nullptr);
      clockSyncedFromServer = true;
    }
  }
  if (cfg.containsKey("q")) {
    const char* cmd = cfg["q"]["cmd"] | "";
    answerPendingQuery(cmd);
    Serial.printf("Net: operator question '%s' - answering on next record\n", cmd);
  }
}

static bool buildNoiseRecord(uint32_t session, uint32_t counter, unsigned long ts,
                             bool hb, float db, float duration, char* out, size_t cap) {
  char event[24];
  snprintf(event, sizeof(event), "%u_%u", session, counter);
  const bool hasReply = pendingReplyJson[0] != '\0';
  snprintf(out, cap,
          "{\"id\":\"%s\",\"type\":\"noise\",\"ts\":%lu,\"hb\":%s,\"event\":\"%s\","
          "\"db\":%.2f,\"duration\":%.2f%s%s}",
          deviceID.c_str(), (unsigned long)ts, hb ? "true" : "false", event, db, duration,
          hasReply ? ",\"qr\":" : "", hasReply ? pendingReplyJson : "");
  return hasReply;
}

int netInit() {
  return dtls.begin(OMEGA_CA_CERT_PEM, OMEGA_DEVICE_CERT_PEM, OMEGA_DEVICE_KEY_PEM);
}

void netCloseSession() {
  dtls.close();
}

void killWiFi() {
  netCloseSession();
  WiFi.disconnect(true);
  vTaskDelay(pdMS_TO_TICKS(10));
  WiFi.mode(WIFI_OFF);
  wifiInUse = false;
}

static bool sessionIsStale() {
  if (lastExchangeMs == 0) {
    return false;
  }
  return (millis() - lastExchangeMs) > staleSessionMs;
}

bool ensureSession() {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Net: WiFi connect failed.");
    return false;
  }
  if (dtls.connected()) {
    if (!sessionIsStale()) {
      return true;
    }
    Serial.println("Net: session idle too long - re-handshaking before send");
    dtls.close();
  }

  if (omegaServer.valid) {
    esp_task_wdt_reset();
    const bool ok = dtls.connect(omegaServer.ip, omegaServer.port,
                                 OMEGA_HANDSHAKE_TIMEOUT_MS);
    esp_task_wdt_reset();
    if (ok) {
      serverFound = true;
      omegaCacheServer(omegaServer);
      lastExchangeMs = millis();
      Serial.printf("Net: DTLS session to %s:%u UP (handshakes=%u)\n",
                    omegaServer.ip, (unsigned)omegaServer.port,
                    (unsigned)dtls.handshakes());
      return true;
    }
    omegaServer.valid = false;
  }

  esp_task_wdt_reset();
  OmegaCandidates candidates = omegaDiscoverCandidates(
      OMEGA_SERVER_IP, OMEGA_SERVER_PORT, DISCOVERY_TIMEOUT_MS);
  esp_task_wdt_reset();
  if (candidates.count == 0) {
    Serial.println("Net: no server address known yet - buffering.");
    serverFound = false;
    return false;
  }

  for (int i = 0; i < candidates.count; i++) {
    const OmegaServer candidate = candidates.items[i];
    esp_task_wdt_reset();
    const bool ok = dtls.connect(candidate.ip, candidate.port,
                                 OMEGA_HANDSHAKE_TIMEOUT_MS);
    esp_task_wdt_reset();
    if (ok) {
      omegaServer = candidate;
      serverFound = true;
      omegaCacheServer(omegaServer);
      lastExchangeMs = millis();
      Serial.printf("Net: DTLS session to %s:%u UP (handshakes=%u)\n",
                    omegaServer.ip, (unsigned)omegaServer.port,
                    (unsigned)dtls.handshakes());
      return true;
    }
    rejectedServers++;
    Serial.printf("Net: %s:%u answered but failed mutual auth - rejected "
                  "(total rejected=%u)\n", candidate.ip,
                  (unsigned)candidate.port, (unsigned)rejectedServers);
  }

  // The cached address is NOT forgotten here. See FINDINGS #57: erasing it
  // on failure is what turned a temporary outage into a permanent one, since
  // the cached unicast address is the only discovery method that reliably
  // works on this hardware. A stale entry costs one failed probe before
  // discovery runs anyway, and is overwritten the moment any server
  // authenticates.
  serverFound = false;
  return false;
}

bool deliverRecord(uint32_t session, uint32_t counter, unsigned long ts, bool hb,
                   float db, float duration) {
  char record[384];
  const bool carriedReply =
      buildNoiseRecord(session, counter, ts, hb, db, duration, record, sizeof(record));
  char ack[384];

  char expect[24];
  snprintf(expect, sizeof(expect), "%u_%u", session, counter);
  const OmegaSendResult result =
      dtls.sendRecord(record, ack, sizeof(ack), OMEGA_ACK_TIMEOUT_MS, expect);
  if (result == OMEGA_SEND_LOST || result == OMEGA_SEND_BROKEN) {
    serverFound = false;
    return false;
  }

  StaticJsonDocument<384> ackDoc;
  if (deserializeJson(ackDoc, ack)) {
    return false;
  }
  char event[24];
  snprintf(event, sizeof(event), "%u_%u", session, counter);
  if (!ackDoc.containsKey("ack") || ackDoc["ack"] != event) {
    return false;
  }
  if (carriedReply) {
    pendingReplyJson[0] = '\0';
  }
  applyAckConfig(ack);
  lastExchangeMs = millis();

  if (result == OMEGA_SEND_REAUTH) {
    Serial.println("Net: server requested re-auth (revocation checkpoint)");
    serverFound = false;
  }
  return true;
}

uint32_t netLastHandshakeMs() {
  return dtls.lastHandshakeMs();
}

uint32_t netHandshakeCount() {
  return dtls.handshakes();
}

uint32_t netStaleAcks() {
  return dtls.staleAcks();
}

bool netSessionConnected() {
  return dtls.connected();
}

void netSetRadioSleep(bool enable) {
#if RADIO_SLEEP_ENABLED
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(enable);
  }
#else
  (void)enable;
#endif
}

int netRadioSleepMode() {
  wifi_ps_type_t ps = WIFI_PS_NONE;
  if (esp_wifi_get_ps(&ps) != ESP_OK) {
    return -1;
  }
  return (int)ps;
}

unsigned long netResolveTimestamp(unsigned long stored_ts, uint32_t capture_ms,
                                  bool same_boot) {
  // The rule itself lives in omega_time.h, free of Arduino types, so it can
  // be tested on a PC rather than only on hardware. This wrapper supplies the
  // three things only the device knows: its uptime, its idea of the wall
  // clock, and whether that idea is trustworthy yet.
  return omegaResolveTimestamp(stored_ts, capture_ms, millis(),
                               (unsigned long)time(nullptr), netClockSynced(),
                               same_boot);
}

void netValidateRestoredClock() {
  if (clockStateMagic != RTC_STATE_MAGIC) {
    clockStateMagic = RTC_STATE_MAGIC;
    clockSyncedFromServer = false;
    Serial.println("Clock: RTC state uninitialised (power-on) - time unknown "
                   "until the server answers");
    return;
  }
  if (!clockSyncedFromServer) {
    return;
  }
  const unsigned long now = (unsigned long)time(nullptr);
  if (now < TIME_VALID_EPOCH) {
    // The flag survived but the clock did not, so power was lost after all.
    // Trusting the flag alone here would stamp every reading with a confident
    // wrong time, which is worse than admitting the time is unknown.
    clockSyncedFromServer = false;
    Serial.println("Clock: lost across power-off - time unknown until the "
                   "server answers");
    return;
  }
  Serial.printf("Clock: %lu survived the reset - still trusted\n", now);
}

bool netClockSynced() {
  return clockSyncedFromServer;
}

bool netRebootRequested() {
  return rebootRequested;
}
