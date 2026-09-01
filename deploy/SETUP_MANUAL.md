# Project Omega - Brick 4 (DTLS 1.3) Setup Manual

**CORRECTED 2026-08-27: this manual predates `deploy/server/install_hub.sh`
("2026-08-26a-replication-pack"), which is now the real starting point for
a bare Ubuntu server - it runs CA generation, the server install, the unit
factory and the operator console in one idempotent pass
(`./install_hub.sh`, or `--only pki` / `--check` for one phase at a time).
Steps 0-1 below (manual `omega_pki.py`, manual `install_server.sh`) still
work but are the LONGER path; use `install_hub.sh` unless you have a
specific reason not to. Step 2's `omega_pki.py device`/manual per-unit AMU
copy is superseded by `provisioning/clone-amu-card` (from a golden image)
or `provisioning/patch-amu-card` (updating an existing card in place).
Step 3's NMU section describes manual Arduino IDE flashing - superseded by
`provisioning/nmu_factory.sh`, which produces a self-contained, no-toolchain
ZIP per batch (see ARCHITECTURE.md section 19 for why). This file is kept
as narrative background on WHY each piece exists; treat the scripts' own
header comments as the authoritative how-to.**

This manual takes you from bare devices to a running, encrypted fleet. No prior
knowledge of the code is needed - you copy folders and run one script per
device. Commands are copy-paste ready. Where you must edit a value, it says so
in **bold**.

The three device types:

- **Server** - the Ubuntu mini PC. Receives everything, runs the dashboard.
- **AMU** - Raspberry Pi air-quality unit (AMU_01, AMU_02, ...).
- **NMU** - ESP32-S3 noise unit (NMU_01, NMU_02, ...).

Everything is authenticated by **certificates**. There is one Certificate
Authority (CA); it signs the server and every device. A device with no
CA-signed certificate cannot join, even with the WLAN password.

---

## Prerequisites and what installs them

`install_hub.sh` bootstraps everything on a bare Ubuntu server: the apt
packages (`python3`, `python3-venv`, `python3-pip`, `nginx`, `avahi`,
`sqlite3`, `curl`, `git`, `openssl`), `arduino-cli` with the ESP32 core and
firmware libraries, and the Python packages (`cryptography`, `wolfssl`,
`flask`, `mcp`, `zeroconf`). `./install_hub.sh --check` reports what is
missing without changing anything.

The **manual path** (Steps 0-1 below) only bootstraps the server's Python
packages, plus nginx and avahi; it assumes `python3` and `python3-venv` are
present and prints the `apt install` line if not. If you take the manual path
specifically to keep the CA private key on a pendrive (see Step 0), run
`./install_hub.sh --only system` and `--only arduino` first for the base
tooling - those phases never touch the CA - or install `arduino-cli` and
`python3-cryptography` yourself.

**Versions.** The version numbers this project ran on are recorded for
provenance (`wolfssl` 5.9.2 on the server, `wolfssl@5.8.4` in the Arduino
build, `esptool` 5.3.1, `arduino-cli` current at the time). When recreating
the system from scratch, install the **current release** of each of these
rather than hunting for the exact build - none of the security properties
depend on a specific patch version, and newer releases carry fixes. The one
firm requirement is that the server-side stack provides **DTLS 1.3**
(`wolfssl` does; the older `python3-dtls`/PyDTLS does not), and that the
Arduino wolfSSL build has `WOLFSSL_SHA384` enabled if the AES-256 suite is
wanted on the noise unit (Section on cipher suites). Re-run
`simlab/run_gate.py` after any version bump.

---

## Step 0 - Make the certificates (once, on the server)

On the server, in the `brick4_dtls13` folder (or use `install_hub.sh` per
the note above, which does this step for you):

```
python3 provisioning/omega_pki.py init ./pki
```

This creates `./pki/` containing:

- `ca-cert.pem` - the trust anchor (goes on every device).
- `ca-key.pem` - **keep this offline and secret.** It never goes on a device.
- `omega-server-cert.pem` / `omega-server-key.pem` - the server identity.
- `AMU_01-cert.pem` / `AMU_01-key.pem`, ... - one pair per device.

Need another device later? `python3 provisioning/omega_pki.py device ./pki AMU_04`.

### Where the CA private key ends up, and the two supported paths

For a supervisor recreating the system with a **fresh** CA, use one of these.
The first is the one that has been tested phase by phase; the second keeps the
CA private key off the server but is only partly supported by the tooling
today.

**Path A - the tested one-command path (`START_HERE.md`).** Run
`./setup-server` with the `ca/` folder left empty. Phase 2 creates a new CA,
then the server, operator, retention, dashboard and unit factory are set up in
one pass (`--check` for a dry run first). Consequence: `ca-key.pem` lives on
the server at `~/omega_brick4/pki_provisioning/`. This is the deliberate
prototype trade recorded in `FUTURE_WORK.md` section 7 - anyone with
administrator access to the server can then mint a unit. `START_HERE.md` and
`REPLICATION_AUDIT.md` are the full account; the pack is verified phase by
phase and dry-run green end to end, with a single uninterrupted bare-metal run
being the one step not yet evidenced.

**Path B - CA private key stays on removable media.** Create the new CA on the
drive: `./provisioning/new-fleet-ca /media/<drive>/ca` (it prompts for
`NEW FLEET`). Copy only `ca-cert.pem`, `omega-server-cert.pem` and
`omega-server-key.pem` to `deploy/server/pki/` and run
`deploy/server/install_server.sh` directly - **not** `./setup-server`, whose
PKI phase refuses a directory that has `ca-cert.pem` but no `ca-key.pem`.
NMU units are then built with the `provisioning/pendrive/` kit, whose
`make-units` reads the CA key straight from the drive (`--ca-key-dir`) and
never copies it to the server.
**Known gap:** the AMU card tools (`clone-amu-card`, `prepare-amu-card`,
`make-amu-bundle`) currently require `ca-key.pem` to be present in the
server's PKI directory - they do not accept `--ca-key-dir`. A fully
CA-off-server build therefore needs the key placed on the server only while
AMU cards are written and removed afterwards, or the AMU device certificates
issued by hand with `omega_pki.py device ... --ca-key-dir`. Path B has not
been run end to end.

In both paths, back up `ca-key.pem` plus `ca-cert.pem` to a second offline
location. A lost CA key cannot be recovered and every unit would have to be
rebuilt.

---

## Step 1 - Server

1. Copy the whole `deploy/server` folder to the server (e.g. to `~/omega-install`).
2. Copy these three files from `./pki` into `deploy/server/pki/`:
   `ca-cert.pem`, `omega-server-cert.pem`, `omega-server-key.pem`.
3. Run the installer (it asks for sudo when needed):

```
cd ~/omega-install/server
./install_server.sh
```

4. When it finishes, open the dashboard: `http://<server-ip>:8080`.
5. Health check any time: `./install_server.sh --check`.

The installer sets up the DTLS listener and dashboard as services (auto-start on
boot) and schedules nightly database cleanup and the daily statistics report.

---

## Step 2 - Each AMU (Raspberry Pi)

Do this on each Pi, logged in as the normal `pi`-type user. **Do not use sudo to
run the installer** - it will refuse (that mistake caused a sensor conflict).

1. Copy the `deploy/amu` folder to the Pi (e.g. `~/omega-install/amu`).
2. Copy this unit's three files from the server's `./pki` into `deploy/amu/pki/`:
   `ca-cert.pem`, `AMU_0X-cert.pem`, `AMU_0X-key.pem` (use the right number).
3. Create and edit the config:

```
cd ~/omega-install/amu
cp payload/config/global.ini.example payload/config/global.ini
nano payload/config/global.ini
```

Set **device_id** (e.g. AMU_01), **wifi_ssid**, **wifi_pass**, and
**server_host** (the server's IP). Save.

4. Install (no sudo):

```
./install_amu.sh
```

5. Watch it run: `journalctl -u omega-amu -f`. Health check: `./install_amu.sh --check`.

6. **Optional but recommended - the live-query service.** Without it the AMU
   still sends everything normally; what you lose is the ability to ASK it for
   a reading on demand (questions then wait for the unit's next transmission,
   up to 5 minutes). It needs a second, separate service because the
   encryption library cannot host two sessions in one process:

```
sudo ./install_live_service.sh
```

   Check both are up: `systemctl is-active omega-amu omega-amu-live`.

---

## Step 3 - Each NMU (ESP32-S3)

Done from the laptop with the Arduino IDE.

1. On the server, make this unit's certificate header:

```
python3 provisioning/omega_pki.py arduino ./pki NMU_01 omega_certs.h
```

2. Copy the sketch folder `deploy/esp32/stage3_beta_2_dtls` to the laptop and put
   `omega_certs.h` inside it (next to the `.ino`). **Never commit this file - it
   holds the device key.**
3. In that folder, copy `config.h.example` to `config.h` and set **OMEGA_WIFI_SSID**,
   **OMEGA_WIFI_PASS**, and **OMEGA_SERVER_IP**.
4. Open the sketch in Arduino IDE, select the ESP32-S3 board, and Upload.
   If the upload fails with a checksum/timeout error, use a USB 2.0 port, a
   different cable, and set the upload speed to 115200 (see TROUBLESHOOT.md).
5. Open Serial Monitor at 115200 to watch it connect.

---

## What the system does (functionality)

- **Devices** sense continuously. They send a reading when something changes
  (an alarm, a spike/drift) and a routine **heartbeat** at a set interval.
- Each reading travels inside a **mutual-authenticated DTLS session** - both
  sides prove identity with certificates, the traffic is encrypted, and nothing
  readable appears on the wire.
- The **server** stores each reading (duplicates are ignored) and replies with
  an **ACK** that carries the current heartbeat setting. Changing the heartbeat
  in the dashboard reaches each device on its next ACK - no restart, no push.
- If a device loses the server, it **buffers** readings and flushes them when
  the session is restored (no data lost for normal events).
- The dashboard shows live noise and air-quality data, lets you **set the
  heartbeat** per device type, and **revoke** a device (blocks it even if its
  certificate is still valid).
- Nightly, the server **trims** the oldest data if the database grows too large
  and **emails a statistics report** (min/max/avg/median/sd per device) to the
  configured website endpoint.

See TROUBLESHOOT.md if anything does not come up.
