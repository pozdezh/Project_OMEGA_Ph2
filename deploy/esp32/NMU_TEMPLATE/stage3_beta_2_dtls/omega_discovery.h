// Find the Omega server from the NMU without a hardcoded address.
//
// The router (DHCP) can reassign the server's IP at any time. With the
// address compiled into the firmware, that change bricks the whole fleet and
// every unit needs a reflash. This module removes that failure mode.
//
// Three sources, cheapest first:
//   1. CACHE  - the address that worked last time, kept in NVS (the ESP32's
//               small storage area that survives reboot and reflash). Almost
//               always still right, costs one datagram to confirm.
//   2. mDNS   - ask the local network by name (_omega._udp.local), the same
//               mechanism printers and Chromecasts use. NOTE: ESPmDNS cannot
//               read SRV priority, which is why the server also publishes
//               "prio" as a TXT record.
//   3. UDP BROADCAST PROBE - shout on the LAN and let the server answer.
//               Works when mDNS is unavailable on either end.
//
// A compiled-in address remains as a final fallback for sites with no
// multicast, but it is now the last resort rather than the only option.
//
// SAFETY: discovery only ever yields a candidate ADDRESS. Nothing secret is
// exchanged and nothing is trusted. A forged reply merely points the device
// at a host that cannot produce a CA-signed certificate, so the handshake
// fails and the device moves on. The DTLS handshake stays the trust boundary.

#ifndef OMEGA_DISCOVERY_H
#define OMEGA_DISCOVERY_H

#include <Arduino.h>

struct OmegaServer {
  char ip[16];
  uint16_t port;
  bool valid;
};

static const int OMEGA_MAX_CANDIDATES = 6;

struct OmegaCandidates {
  OmegaServer items[OMEGA_MAX_CANDIDATES];
  int count;
};

// Every address worth trying, best first, deduplicated. These are CANDIDATES,
// not servers: nothing here is trusted and nothing is written to NVS. The
// caller tries each until a handshake succeeds - see ARCHITECTURE.md 13.
OmegaCandidates omegaDiscoverCandidates(const char* static_fallback_ip,
                                        uint16_t default_port,
                                        uint32_t timeout_ms);

// Drop the remembered address after it fails to authenticate.
void omegaForgetServer();

// Remember a server that actually completed a handshake, so the next boot
// finds it in one datagram instead of re-querying the network.
void omegaCacheServer(const OmegaServer& server);

// The cached address alone, without probing. Used on boot before WiFi is
// fully settled.
OmegaServer omegaCachedServer(uint16_t default_port);

#endif  // OMEGA_DISCOVERY_H
