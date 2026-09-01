# Fix: the AMUs now discover their server instead of being told where it is

Found and fixed 2026-08-26 03:00-03:15, on the deployed fleet, over SSH.
No card was removed and no unit was reflashed.

Capture: `captures/2026-08-26_amu_fleet_mdns_discovery.pcap`

## The fault

The project's claim is that a unit finds its server by itself, so a DHCP
change cannot strand the fleet. That was true of the NMU and **not** of the
AMU.

An AMU carries a configured address in `config/global.ini` as its FINAL
fallback:

```
server_host = 192.168.0.112
```

But `ensure_session()` tried that address BEFORE consulting discovery:

```python
if client.connected():  return True
if client.connect():    return True      # the configured address
return _try_candidates()                 # discovery - only if that failed
```

On a network where the configured address happens to be correct, the first
attempt always succeeded. **Discovery never ran.** The feature the fleet's
DHCP-independence rests on was dead code in production.

Two consequences followed silently:
- no AMU had ever written a discovery cache, because the cache is written
  only by `confirm_server()` inside `_try_candidates()`
- a DHCP move would have cost a full round of failures before the fallback
  engaged, rather than being absorbed on the next connection

**`find_servers()` was never the problem.** Its ranking was already correct -
cache, mDNS, broadcast, then the configured address last, exactly as
ARCHITECTURE.md section 13 describes. The fault was reaching a configured
address without ever consulting the ranking.

## How it surfaced

Not from a test, and not from anything failing. Two packet captures side by
side: an NMU restart showed an mDNS query before its handshake, and an AMU
restart showed a handshake with nothing in front of it. Both units were
working perfectly.

This is the argument for capturing from outside the system rather than
trusting its own logs: **the AMU's log said it connected, and it was true.**

## The fix

One gate. The current address is retried only once a handshake has already
succeeded against it:

```python
if client.connected():
    return True
time.sleep(jitter)
if _server_established and client.connect():
    return True
return _try_candidates()
```

- **First connection ever** -> discovery runs, ranked properly.
- **Reconnect to a known-good server** -> no discovery cost, straight back.
- **Configured address** -> still present, now genuinely last.

Pinned by two tests in `amu/test_rogue_server.py` so it cannot regress
quietly: one asserts `ensure_session` does not try its address before
discovery, the other asserts the configured address is ranked after mDNS and
broadcast. Both read the shipped file rather than importing it, because
`network.py` needs wolfSSL and the gate runs where that is unavailable.

## Evidence it works, all 8 units

Deployed over SSH to every AMU and restarted, with a capture running.

**From the units:**

```
03:11:57 amu16  discovery: candidate 192.168.0.112:11400 (mDNS)
03:11:59 amu17  discovery: candidate 192.168.0.112:11400 (mDNS)
03:12:00 amu11  discovery: candidate 192.168.0.112:11400 (mDNS)
03:12:00 amu14  discovery: candidate 192.168.0.112:11400 (mDNS)
03:12:05 amu12  discovery: candidate 192.168.0.112:11400 (mDNS)
03:12:06 amu13  discovery: candidate 192.168.0.112:11400 (mDNS)
03:12:08 amu10  discovery: candidate 192.168.0.112:11400 (mDNS)
03:12:14 amu15  discovery: candidate 192.168.0.112:11400 (mDNS)
```

**From the wire, the same seconds:**

```
03:11:54.368  192.168.0.118.5353 > 224.0.0.251.5353: PTR (QU)? _omega._udp.local.
03:11:54.369  192.168.0.112.5353 > 192.168.0.118.5353: PTR smartageing._omega._udp.local.
03:11:56.926  192.168.0.116.5353 > 224.0.0.251.5353: PTR (QU)? _omega._udp.local.
03:11:57.537  192.168.0.109.5353 > 224.0.0.251.5353: PTR (QU)? _omega._udp.local.
03:11:57.744  192.168.0.113.5353 > 224.0.0.251.5353: PTR (QU)? _omega._udp.local.
```

Each unit asks the network "who offers `_omega._udp`?" and the server answers
by name. The address is no longer something a device is told in advance.

**And the cache now exists**, on every unit, written only after a completed
handshake:

```
{"ip": "192.168.0.112", "port": 11400, "saved": 1787706721}
```

## Both device types now behave identically

| | NMU | AMU |
|---|---|---|
| Address in firmware/config | none (`OMEGA_SERVER_IP ""`) | present, as final fallback |
| First boot | discovery | **discovery** |
| Reconnect | cached address first | **cached address first** |
| Cache written | only after a handshake | **only after a handshake** |

## Scope

The card tool (`patch-amu-card`) is unchanged. It installs whatever is in
`deploy/amu/payload/`, which now carries the fixed `network.py` - so a card
patched from here on receives this without the tool knowing anything about it.
The eight deployed units were updated over the network; none was opened.
