// Server discovery for the NMU. See omega_discovery.h for the rationale.

#include "omega_discovery.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

static const char* NVS_NAMESPACE = "omega";
static const char* NVS_KEY_IP = "srv_ip";
static const char* NVS_KEY_PORT = "srv_port";

static const uint16_t DISCOVERY_PORT = 5001;
static const char* DISCOVERY_MAGIC = "OMEGA_DISCOVER_V1";
static const char* DISCOVERY_REPLY_MAGIC = "OMEGA_SERVER_V1";
static const char* MDNS_SERVICE = "omega";
static const char* MDNS_PROTO = "udp";
static const uint32_t PROBE_TIMEOUT_MS = 1500;
static const int MDNS_DEFAULT_PRIORITY = 50;

static void setServer(OmegaServer* out, const char* ip, uint16_t port) {
  strncpy(out->ip, ip, sizeof(out->ip) - 1);
  out->ip[sizeof(out->ip) - 1] = '\0';
  out->port = port;
  out->valid = true;
}

OmegaServer omegaCachedServer(uint16_t default_port) {
  OmegaServer server;
  memset(&server, 0, sizeof(server));
  server.valid = false;

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) {
    return server;
  }
  String ip = prefs.getString(NVS_KEY_IP, "");
  uint16_t port = prefs.getUShort(NVS_KEY_PORT, default_port);
  prefs.end();

  if (ip.length() >= 7) {
    setServer(&server, ip.c_str(), port ? port : default_port);
  }
  return server;
}

void omegaCacheServer(const OmegaServer& server) {
  if (!server.valid) {
    return;
  }
  OmegaServer existing = omegaCachedServer(server.port);
  // Only write when it actually changed - NVS is flash, and flash wears out.
  if (existing.valid && strcmp(existing.ip, server.ip) == 0 &&
      existing.port == server.port) {
    return;
  }
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return;
  }
  prefs.putString(NVS_KEY_IP, server.ip);
  prefs.putUShort(NVS_KEY_PORT, server.port);
  prefs.end();
  Serial.printf("DISCOVERY: cached server %s:%u\n", server.ip,
                (unsigned)server.port);
}

// Ask the address directly whether an Omega server is answering there. Cheap
// liveness check before paying ~5 s for a full handshake.
static bool probeReachable(const char* ip, uint16_t port, uint32_t timeout_ms) {
  WiFiUDP udp;
  if (!udp.begin(0)) {
    return false;
  }
  IPAddress target;
  if (!target.fromString(ip)) {
    udp.stop();
    return false;
  }
  udp.beginPacket(target, DISCOVERY_PORT);
  udp.write((const uint8_t*)DISCOVERY_MAGIC, strlen(DISCOVERY_MAGIC));
  udp.endPacket();

  const uint32_t deadline = millis() + timeout_ms;
  char buffer[256];
  while ((int32_t)(deadline - millis()) > 0) {
    const int size = udp.parsePacket();
    if (size > 0) {
      const int read = udp.read(buffer, sizeof(buffer) - 1);
      if (read > 0) {
        buffer[read] = '\0';
        if (strstr(buffer, DISCOVERY_REPLY_MAGIC) != nullptr) {
          udp.stop();
          return true;
        }
      }
    }
    esp_task_wdt_reset();
    delay(10);
  }
  udp.stop();
  (void)port;
  return false;
}

static const char* mdnsHostname() {
  static char host[24] = "";
  if (host[0] == '\0') {
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    snprintf(host, sizeof(host), "omega-nmu-%s", mac.substring(6).c_str());
  }
  return host;
}

// Fills out[] with every Omega server mDNS can see, lowest "prio" TXT value
// first (the DNS SRV convention). Returns how many were written.
static int discoverByMdns(OmegaServer* out, int capacity, uint16_t default_port) {
  // mDNS must be started after WiFi is up; restarting it is harmless.
  // The hostname carries the MAC suffix because MDNS.begin() also CLAIMS the
  // name: a fleet of units all announcing "omega-nmu.local" would spend every
  // boot resolving a 40-way name conflict on multicast.
  if (!MDNS.begin(mdnsHostname())) {
    return 0;
  }
  const int found = MDNS.queryService(MDNS_SERVICE, MDNS_PROTO);
  if (found <= 0) {
    return 0;
  }

  int prios[OMEGA_MAX_CANDIDATES];
  int written = 0;
  for (int i = 0; i < found && written < capacity; i++) {
    IPAddress address = MDNS.address(i);
    if (address == IPAddress((uint32_t)0)) {
      continue;
    }
    int prio = MDNS_DEFAULT_PRIORITY;
    String txt = MDNS.txt(i, "prio");
    if (txt.length() > 0) {
      prio = txt.toInt();
    }
    const uint16_t port = MDNS.port(i);
    setServer(&out[written], address.toString().c_str(),
              port ? port : default_port);
    prios[written] = prio;
    written++;
  }

  for (int i = 1; i < written; i++) {
    const OmegaServer server = out[i];
    const int prio = prios[i];
    int j = i - 1;
    while (j >= 0 && prios[j] > prio) {
      out[j + 1] = out[j];
      prios[j + 1] = prios[j];
      j--;
    }
    out[j + 1] = server;
    prios[j + 1] = prio;
  }

  for (int i = 0; i < written; i++) {
    Serial.printf("DISCOVERY: mDNS candidate %s:%u prio=%d\n", out[i].ip,
                  (unsigned)out[i].port, prios[i]);
  }
  return written;
}

static bool discoverByBroadcast(OmegaServer* out, uint16_t default_port,
                                uint32_t timeout_ms) {
  WiFiUDP udp;
  if (!udp.begin(0)) {
    return false;
  }
  // From the interface's real netmask, not localIP with .255 forced: that
  // assumed a /24 and sends to the wrong address on any other subnet.
  IPAddress broadcast = WiFi.broadcastIP();
  if (broadcast == IPAddress((uint32_t)0)) {
    broadcast = WiFi.localIP();
    broadcast[3] = 255;
  }

  udp.beginPacket(broadcast, DISCOVERY_PORT);
  udp.write((const uint8_t*)DISCOVERY_MAGIC, strlen(DISCOVERY_MAGIC));
  udp.endPacket();

  const uint32_t deadline = millis() + timeout_ms;
  char buffer[256];
  while ((int32_t)(deadline - millis()) > 0) {
    const int size = udp.parsePacket();
    if (size > 0) {
      const int read = udp.read(buffer, sizeof(buffer) - 1);
      if (read > 0) {
        buffer[read] = '\0';
        if (strstr(buffer, DISCOVERY_REPLY_MAGIC) != nullptr) {
          // Minimal parse: the responder's own source address is the answer.
          IPAddress from = udp.remoteIP();
          uint16_t port = default_port;
          const char* port_field = strstr(buffer, "\"port\":");
          if (port_field != nullptr) {
            const int parsed = atoi(port_field + 7);
            if (parsed > 0 && parsed < 65536) {
              port = (uint16_t)parsed;
            }
          }
          setServer(out, from.toString().c_str(), port);
          udp.stop();
          Serial.printf("DISCOVERY: broadcast found %s:%u\n", out->ip,
                        (unsigned)out->port);
          return true;
        }
      }
    }
    esp_task_wdt_reset();
    delay(10);
  }
  udp.stop();
  return false;
}

void omegaForgetServer() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return;
  }
  prefs.remove(NVS_KEY_IP);
  prefs.remove(NVS_KEY_PORT);
  prefs.end();
  Serial.println("DISCOVERY: cached server failed to authenticate - forgotten");
}

static void addCandidate(OmegaCandidates* list, const OmegaServer& server,
                         const char* why) {
  if (!server.valid || list->count >= OMEGA_MAX_CANDIDATES) {
    return;
  }
  for (int i = 0; i < list->count; i++) {
    if (strcmp(list->items[i].ip, server.ip) == 0 &&
        list->items[i].port == server.port) {
      return;
    }
  }
  list->items[list->count] = server;
  list->count++;
  Serial.printf("DISCOVERY: candidate %s:%u (%s)\n", server.ip,
                (unsigned)server.port, why);
}

OmegaCandidates omegaDiscoverCandidates(const char* static_fallback_ip,
                                        uint16_t default_port,
                                        uint32_t timeout_ms) {
  OmegaCandidates list;
  memset(&list, 0, sizeof(list));
  list.count = 0;

  OmegaServer cached = omegaCachedServer(default_port);
  if (cached.valid && probeReachable(cached.ip, cached.port, PROBE_TIMEOUT_MS)) {
    addCandidate(&list, cached, "last confirmed, still answering");
  }

  OmegaServer via_mdns[OMEGA_MAX_CANDIDATES];
  const int mdns_count = discoverByMdns(via_mdns, OMEGA_MAX_CANDIDATES,
                                        default_port);
  for (int i = 0; i < mdns_count; i++) {
    addCandidate(&list, via_mdns[i], "mDNS");
  }

  OmegaServer via_broadcast;
  memset(&via_broadcast, 0, sizeof(via_broadcast));
  if (discoverByBroadcast(&via_broadcast, default_port, timeout_ms)) {
    addCandidate(&list, via_broadcast, "UDP broadcast");
  }

  if (static_fallback_ip != nullptr && strlen(static_fallback_ip) >= 7) {
    OmegaServer configured;
    memset(&configured, 0, sizeof(configured));
    setServer(&configured, static_fallback_ip, default_port);
    addCandidate(&list, configured, "configured fallback");
  }

  if (cached.valid) {
    addCandidate(&list, cached, "last confirmed, unverified");
  }

  if (list.count == 0) {
    Serial.println("DISCOVERY: no server found by any method");
  }
  return list;
}
