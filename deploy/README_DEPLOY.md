# Project Omega - Brick 3 (true DTLS) Deployment

This bundle deploys the true-DTLS track: mutual-authenticated, forward-secret
DTLS 1.2 between every device and the server, with certificate identities.

## Read these in order

1. **SETUP_MANUAL.md** - step-by-step, copy-paste setup for server, AMU, NMU.
2. **TROUBLESHOOT.md** - health checks and fixes for issues seen on hardware.

## Folder layout

```
deploy/
  server/   install_server.sh   payload/ (DTLS listener, app core, dashboard,
                                          maintenance crons, MCP server)
            pki/  <- put ca-cert.pem + omega-server-cert.pem + -key.pem here
  amu/      install_amu.sh      payload/ (sensor_manager_dtls, dtls_client, config)
            pki/  <- put ca-cert.pem + this unit's AMU_0X-cert.pem + -key.pem here
  esp32/    stage3_beta_2_dtls/ (sketch + omega_dtls.h/.cpp + config example)
```

## Order of operations

1. Make certificates once: `python3 ../provisioning/omega_pki.py init ./pki`
   (keep `ca-key.pem` offline).
2. Install the **server** (Step 1 of SETUP_MANUAL).
3. Install each **AMU** (Step 2) and flash each **NMU** (Step 3).

## Before you deploy

Everything in this bundle is proven on the laptop by the verification gate:

```
py -3.12 ../simlab/run_gate.py
```

The gate must print `GATE: PASS`. It checks the DTLS mutual-auth handshake, the
server logic (dedup, ACK heartbeat piggyback, revocation), the maintenance and
MCP tools, a full-stack in-memory client<->server round-trip with the attack
suite (eavesdrop, spoofing, cross-CA forgery), and that these shipped payload
files are byte-identical to the tested source.

Note: live UDP DTLS is validated on the Linux server/Pi and on the ESP32, not on
the Windows dev laptop (a Windows UDP-loopback limitation, not a code issue).
