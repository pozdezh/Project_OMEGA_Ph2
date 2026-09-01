# Knowing the WiFi password gets you nothing

Run 2026-08-26 against the LIVE listener on its real port (11400), while the
real fleet was connected and reporting normally.

Script: `simlab/wifi_attacker.py` (also saved under `evidence/captures/`).

## The threat model, stated exactly

The attacker has:
- **the WiFi password** - so full local-network access; they can reach any
  host and any port, sniff traffic, and send whatever they like
- **complete knowledge of the protocol** - the entire source tree is public,
  so nothing about how the system works is secret
- **the ability to forge any identity** - they can make a device-id string
  say anything, and they can run their own certificate authority and sign a
  cryptographically valid certificate claiming to be `NMU_16`

The attacker does **not** have: the project CA's private key.

This is the realistic outsider. A WiFi password is the single thing most
likely to leak - written on a whiteboard, shared with a contractor, guessed.
The question this answers is: **what does that leak actually buy an
attacker?**

## Positive control

Throughout the run, the real fleet kept connecting and delivering data,
visible in the listener log. Same server, same port, same protocol version.
The ONLY thing the attacker lacks that a real unit has is a CA-signed
certificate. So if that one missing thing is what stops every attempt, the
certificate is demonstrably carrying the entire trust decision.

## The three attempts, and what happened

**Attempt 1 - raw UDP injection (the pre-security era).**
The attacker sends the exact JSON a real device would, in the clear, posing
as `NMU_16` with a forged "Sustained Alarm: High Temp".

> RESULT: silence. The listener speaks only DTLS; a plain UDP datagram is not
> a handshake, so it is ignored. The forged alarm never reaches the database.

**Attempt 2 - a proper DTLS handshake with no certificate.**
The attacker speaks the correct protocol but presents no identity.

> RESULT: handshake REFUSED - do_handshake failed with error -308

**Attempt 3 - DTLS with a valid-looking certificate the attacker made
themselves.**
The attacker runs their own certificate authority and signs a certificate
that says `CN = NMU_16`. It is cryptographically perfect - correct structure,
valid signature, not expired. It is simply signed by the wrong authority.

> RESULT: handshake REFUSED - do_handshake failed with error -308

## Corroboration from the server's own log

The attacker's account could be doubted; the server's cannot. During the run
the listener logged, from the attacker's address:

```
18:45:37  handshake from ('192.168.0.111', 38876) never completed in 30s -
          closing its socket and releasing the slot
18:45:38  handshake/session failed: SSLError: do_handshake failed with
          error -308: error state on socket. alert (-1):
```

And the forged records never landed:

```
forged events in database: 0
```

## Summary

```
1 raw UDP injection              DEFEATED
2 DTLS, no certificate           DEFEATED
3 DTLS, forged certificate       DEFEATED
```

**WiFi access yielded nothing.** Being on the network is not being in the
system. The trust boundary is not the WiFi password and not the network - it
is the CA-signed certificate, which the attacker cannot produce without the
CA's private key. This is the practical, measured meaning of "the handshake
is the trust boundary, discovery is not" that the discovery design rests on.

## Honest limitations of THIS run

- The attacker script was executed from a host with network reach to the
  listener, which is exactly what the WiFi password grants - so the vantage
  point is faithful, but no separate physical attacker machine was used.
- It exercises the SERVER's refusal of a rogue client. The mirror case - a
  rogue SERVER trying to fool a real device - is covered separately by the
  AMU's `test_rogue_server.py` and the discovery evidence.
- Error -308 is wolfSSL's generic "handshake failed after an alert." The
  cause here is certificate verification (`CERT_REQUIRED` against the project
  CA, listener.py); the code path is the same one the attack suite exercises
  for a cross-CA certificate.

## Where the real risk actually is (see THREAT_MODEL.md)

This run proves the network attacker gets nothing. It says nothing about an
attacker who physically STEALS a unit - that is a different threat, with
different (and currently weaker) defenses, documented honestly in
`THREAT_MODEL.md`.
