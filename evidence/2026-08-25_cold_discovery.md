# Cold-start discovery: a unit that knows nothing finds its server

Captured 2026-08-25 from NMU (Adafruit Feather ESP32-S3), serial at 115200.

## Method

The unit's NVS partition (0x9000, 0x5000 bytes) - the only place it stores a
remembered server address - was erased with esptool, then the unit was reset.
It therefore booted in exactly the state of a brand-new unit out of the box:
firmware, WiFi credentials and its own certificate, but NO idea where the
server is. The firmware carries no server IP at all (`OMEGA_SERVER_IP ""`).

## Result - cold boot, empty memory

    WiFi: status=3 ip=192.168.0.100 rssi=-73 bcast=192.168.0.255
    DISCOVERY: mDNS candidate 192.168.0.112:11400 prio=10
    DISCOVERY: candidate 192.168.0.112:11400 (mDNS)
    DISCOVERY: broadcast found 192.168.0.112:11400
    DTLS: session #1 up in 4163 ms (DTLSv1.3 / TLS_AES_128_GCM_SHA256)
    DISCOVERY: cached server 192.168.0.112:11400
    Net: DTLS session to 192.168.0.112:11400 UP (handshakes=1)
    Net: event=3353849509_1 ACK OK

Time from power-on to first acknowledged reading: under 45 s, unattended.

## What this proves

1. **No hardcoded address.** There is no cache line in this boot, because
   there was no cache. The address came from the network itself.

2. **The priority record is read.** `prio=10` is the value the server
   publishes in its mDNS TXT record. ESPmDNS cannot read the standard DNS SRV
   priority field, which is why the server publishes it as TXT as well.

3. **Two independent methods agreed.** mDNS answered first; the plain-UDP
   broadcast probe independently found the same address. Either alone is
   sufficient, so one failing does not strand the unit.

4. **Discovery is untrusted; the handshake is the trust boundary.** The line
   `DISCOVERY: cached server` appears AFTER `DTLS: session #1 up`, never
   before. Answering a discovery probe earns a candidate nothing. Only a
   completed mutual-auth DTLS 1.3 handshake writes the address to flash.

## Contrast - the SAME unit's previous boot, with a warm cache

    DISCOVERY: candidate 192.168.0.112:11400 (last confirmed, still answering)
    DISCOVERY: mDNS candidate 192.168.0.112:11400 prio=10
    DISCOVERY: broadcast found 192.168.0.112:11400

The remembered address is tried first because it costs one datagram and is
almost always still right. It does not suppress the other methods; it is
ranked ahead of them, and is discarded if it fails to authenticate.
