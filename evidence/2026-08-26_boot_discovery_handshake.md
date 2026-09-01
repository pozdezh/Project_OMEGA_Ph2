# Cold boot, warm boot, and exactly where the handshake is readable

Run 2026-08-26 02:53-03:00 on the deployed system. NMU_18 (Adafruit Feather
ESP32-S3) on the development PC by USB; the live server capturing on ports
11400 (telemetry), 5001 (discovery) and 5353 (mDNS) throughout.

Every claim below has two independent witnesses: the device's own serial
output, and a packet capture taken on the server. Neither is our test code
agreeing with itself.

Files:
- `captures/2026-08-26_nmu18_coldboot.pcap` / `nmu18_coldboot_serial.txt`
- `captures/2026-08-26_nmu18_warmboot.pcap` / `nmu18_warmboot_serial.txt`
- `captures/2026-08-26_amu_t1_restart.pcap`

---

## 1. Cold boot - a unit that knows nothing

**Method.** The NVS partition (0x9000, 0x5000) was erased with esptool 5.3.1.
That is the only place the unit stores a remembered server address, and the
firmware carries no server IP (`Config OK: id=NMU_18 server=:11400` - the
address field is empty). The board was then released from the bootloader with
serial already recording.

**What the unit said:**

```
BOOT: reset_reason=1 (1=power-on 4=panic 6=task-wdt 9=brownout 11=usb)
POSTMORTEM: previous boot ended reason=-1 after 0s
Config OK: id=NMU_18 server=:11400
BUFFER: mount=1 total=1739681 used=0 mirror=0 tmp=0 bytes=0 restored=0
BOOT: spreading first handshake by 24492 ms
WiFi: status=3 ip=192.168.0.111 rssi=-67 bcast=192.168.0.255
DISCOVERY: mDNS candidate 192.168.0.112:11400 prio=10
DISCOVERY: candidate 192.168.0.112:11400 (mDNS)
DISCOVERY: broadcast found 192.168.0.112:11400
DTLS: session #1 up in 4190 ms (DTLSv1.3 / TLS_AES_128_GCM_SHA256)
DISCOVERY: cached server 192.168.0.112:11400
Net: event=1362299004_1 ACK OK
```

**What the network saw, same seconds:**

```
02:53:10.824  192.168.0.111.5353  > 224.0.0.251.5353: PTR (QM)? _omega._udp.local.
02:53:13.896  192.168.0.111.57999 > 192.168.0.255.5001: UDP, length 17
02:53:13.897  192.168.0.112.5001  > 192.168.0.111.57999: UDP, length 76
02:53:14.718  192.168.0.111.58000 > 192.168.0.112.11400: UDP, length 206
```

The two agree. **No cached entry was consulted** - the first discovery line is
the mDNS answer, not `last confirmed`. The unit went from empty memory to an
acknowledged reading with nobody touching it.

**`BOOT: spreading first handshake by 24492 ms`** is deliberate: a random
delay so that a fleet restored after a site-wide power cut does not hit the
server as one thundering herd.

---

## 2. Warm boot - the same unit, memory intact

Same board, reset again, NVS untouched. **One line differs, and it is the
line that matters:**

```
DISCOVERY: candidate 192.168.0.112:11400 (last confirmed, still answering)
DISCOVERY: mDNS candidate 192.168.0.112:11400 prio=10
DISCOVERY: broadcast found 192.168.0.112:11400
DTLS: session #1 up in 4415 ms (DTLSv1.3 / TLS_AES_128_GCM_SHA256)
```

The remembered address is tried first, and only after it answers a probe
(`still answering`). Note that mDNS and broadcast still run: the cache is a
head start, not a substitute. If the remembered address had moved, the other
two would have supplied the new one in the same second.

**Handshake cost is unchanged** - 4190 ms cold, 4415 ms warm. The cache saves
discovery time, not handshake time, and on this network discovery is cheap
either way.

---

## 3. Exactly where the handshake is readable

Every DTLS record in the cold-boot session, classified by its header. A
record beginning `content-type, 0xfefd` is a legacy plaintext record; a
record using the DTLS 1.3 unified header is encrypted, and even its type
cannot be read.

```
TIME      DIRECTION LEN   DTLS RECORD
------------------------------------------------------------------------------
02:53:14  dev->srv 206   PLAINTEXT  Handshake     epoch=0  ClientHello
02:53:15  dev->srv 206   PLAINTEXT  Handshake     epoch=0  ClientHello
02:53:15  srv->dev 144   PLAINTEXT  Handshake     epoch=0  ServerHello
02:53:15  dev->srv 279   PLAINTEXT  Handshake     epoch=0  ClientHello
02:53:15  srv->dev 700   PLAINTEXT  Handshake     epoch=0  ServerHello
02:53:15  srv->dev 176   ENCRYPTED  unified header 0x2e - type and contents hidden
02:53:17  srv->dev 876   PLAINTEXT  Handshake     epoch=0  ServerHello
02:53:17  dev->srv 394   ENCRYPTED  unified header 0x2e - type and contents hidden
02:53:18  srv->dev  40   ENCRYPTED  unified header 0x2e - type and contents hidden
...
------------------------------------------------------------------------------
6 records readable, 34 records encrypted
```

**The readable part is exactly six records, all `epoch=0`: the ClientHello and
ServerHello exchanges.** There is no other plaintext anywhere in the session.

That is not an accident of configuration, it is the protocol. Those messages
carry the key exchange itself - the cipher suites each side supports and the
public halves of the ephemeral keys - and they must be readable because until
they complete there is no shared secret to encrypt anything with. **From the
first record after ServerHello, the shared secret exists and everything from
then on is encrypted**: the certificates, the identities, the readings and
the ACKs.

The repeated ClientHello at 206 bytes is DTLS's cookie exchange: the server
answers the first one with a stateless challenge and only commits resources
when the client returns it. That is the built-in defence against a flood of
forged handshake openings from spoofed addresses.

### What an observer therefore learns, and what they do not

**Learns:** a device at 192.168.0.111 exists, it asked for `_omega._udp` on
the LAN, it spoke to the server at 02:53, and the messages were of certain
lengths.

**Does not learn:** which device it is, what it measured, or whether the
server accepted it.

---

## 4. The 1.3-over-1.2 claim, tested rather than asserted

The project's stated reason for choosing DTLS 1.3 is that it derives keys
before certificates are sent, closing an identity-exposure gap that 1.2 has -
in 1.2 the certificates travel in the clear, so anyone watching learns exactly
which device is talking.

**This was an assertion from the specification. It is now measured.** The
whole cold-boot capture was searched for the strings that appear inside the
certificates it carried:

| searched for | occurrences in the capture |
|---|---|
| `AMU_T1` | 0 |
| `omega-spike-ca` | 0 |
| `omega-server` | 0 |
| `operator` | 0 |
| `CN=` | 0 |

For comparison, the certificate actually presented in that session:

```
subject = CN = AMU_T1
issuer  = CN = omega-spike-ca
```

**The names are in the certificate and nowhere in the capture.** Under DTLS
1.2 the same search would have returned them, because the Certificate message
is sent before encryption begins. This is the single clearest practical
argument for 1.3 in this project, and it is now evidence rather than a claim.

---

## 5. Finding: the AMU does not discover in normal operation

Recorded because it corrects an overstatement, and because it would be a fair
question at a defense.

**The claim as previously stated** - "the fleet finds its server by itself" -
is true of the NMU and **not currently true of the AMU**.

`amu/config/global.ini` on AMU_T1 contains:

```
server_host = 192.168.0.112
```

The connect path is:

```python
if client.connected():  return True
if client.connect():    return True      # the configured address
return _try_candidates()                 # discovery, only if that failed
```

So the AMU reaches its **configured** address first and runs discovery only
when that stops working. The NMU has no configured address at all
(`OMEGA_SERVER_IP ""`), so it discovers on every boot - which is why its logs
show the full sequence and the AMU's do not.

**Consequence, and it is small:** the AMU never writes a discovery cache,
because the cache is written only by `confirm_server()` inside
`_try_candidates()`. Its `~/.omega_server_cache.json` has never existed. This
is consistent, not broken: with a configured address there is nothing to
remember.

**Is it a fault?** No - the fallback works, and a DHCP move would push the AMU
into discovery on its next failure. But ARCHITECTURE.md describes the static
address as the *final* fallback, and here it is the *first* choice. The two
device types are therefore configured differently for the same design.

**Decision to make:** blanking `server_host` on the AMUs would make both types
behave identically and exercise discovery in production, at the cost of a few
seconds per boot. Leaving it means the AMUs are faster to reconnect and the
discovery path is only proven on the NMU. Not changed tonight - a
configuration change across eight units is not a thing to do at 03:00 while
gathering evidence.
