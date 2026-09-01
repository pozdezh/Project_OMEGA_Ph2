# Procedure: capture the AMU<->server DTLS 1.3 cipher suite (minimal rig)

Goal: turn the deduced answer (`TLS_AES_256_GCM_SHA384`, see
`crypto_versions_and_cipher_choice.md` section 2.1) into an **observed**
one, and re-confirm the NMU's `TLS_AES_128_GCM_SHA256` in the same capture.

**No code change.** The DTLS 1.3 ServerHello carries the negotiated cipher
suite **in cleartext** - record-layer encryption only starts after it - so a
packet capture on the server reads it directly.

---

## Is the minimal rig enough?

**Yes.** Server + internet box / access point + **1 AMU + 1 NMU**, all
headless over SSH, no screen or keyboard. One AMU handshake and one NMU
handshake need to be seen; nothing else.

## Rig

- Server running the normal `omega-listener` service.
- Access point / router (the "internet box") - the units and the server on
  the same LAN. Internet itself is not needed (discovery and DTLS are
  local-only).
- 1 AMU (Raspberry Pi), powered, `omega-amu` service running.
- 1 NMU (ESP32-S3), powered. It will handshake on its own at boot.

## Steps (all via SSH)

**1. Start the capture on the server.** Quiet rig, so capture all UDP to the
server except the noise:

```
sudo tcpdump -i any -s 0 -w ~/amu_nmu_cipher_$(date +%Y%m%d_%H%M).pcap \
  'udp and not port 22 and not port 53 and not port 5353'
```

Leave it running.

**2. Force a fresh AMU handshake.** On the AMU:

```
sudo systemctl restart omega-amu
```

This tears down the AMU's DTLS session and it re-handshakes within a few
seconds.

**3. Force a fresh NMU handshake** (optional - its cipher is already proven
6x, but one capture with both device types is a cleaner artefact). Power-cycle
the NMU, or pull and re-seat its USB power. It handshakes at boot.

**4. Wait ~30 s**, then stop `tcpdump` (Ctrl-C on the server).

**5. Copy the pcap off the server:**

```
scp omega-server:~/amu_nmu_cipher_*.pcap .
```

**6. Read the cipher suites.** Open the pcap in Wireshark:

- Display filter: `dtls.handshake.type == 2`  (ServerHello)
- For each ServerHello, expand
  `Datagram Transport Layer Security -> DTLSv1.3 Record -> Handshake
  Protocol: Server Hello -> Cipher Suite`.
- The AMU's session -> expect `TLS_AES_256_GCM_SHA384 (0x1302)`.
- The NMU's session -> expect `TLS_AES_128_GCM_SHA256 (0x1301)`.

Tell them apart by the client IP in the preceding ClientHello (the AMU and
the NMU have different addresses; cross-check against the server log's
`[AMU_*]` / `[NMU_*]` session-up lines at the same timestamps).

### If Wireshark will not decode it as DTLS

Some captures need the UDP port decoded as DTLS manually: right-click a
packet -> Decode As -> set the server's telemetry port to `DTLS`. The
server's listener port is `OMEGA_PORT` (check `systemctl cat omega-listener`
for the value in this deployment; the code default is 5000).

### Command-line alternative (no Wireshark)

```
tshark -r amu_nmu_cipher_*.pcap -Y 'dtls.handshake.type == 2' \
  -T fields -e ip.src -e ip.dst -e dtls.handshake.ciphersuite
```

Maps ciphersuite `0x1301` = `TLS_AES_128_GCM_SHA256`,
`0x1302` = `TLS_AES_256_GCM_SHA384`, `0x1303` =
`TLS_CHACHA20_POLY1305_SHA256`.

---

## What to record afterwards

A short evidence file (`evidence/2026-08-29_amu_cipher_observed.md`) with:
the pcap name, the two ServerHello cipher-suite values and their client
IPs, the matching server-log session lines, and a one-line update to
`crypto_versions_and_cipher_choice.md` section 2.1 changing "deduced" to
"observed" for the AMU suite (or recording the real value if it differs).
