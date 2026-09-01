# Replication status: what can be rebuilt from this repository, and what is not yet proven

This document states, against the tooling as written rather than from
recollection, exactly what a person with a stock Ubuntu machine can and cannot
rebuild from this repository unaided. It is deliberately explicit about the
one step that has not been observed end to end.

**Scenario it is written against:** a stock Ubuntu PC, this repository on a
pendrive or a clone, an air-unit golden image, blank SD cards, and bare
ESP32-S3 boards. Nothing else.

---

## 1. Summary

The replication path is implemented, coherent and documented, with three
prerequisites that cannot travel in a repository and are named in section 5.
One verification gap remains and is stated in section 6.

The operating procedure for a reader who simply wants to build the system is
`START_HERE.md`. This document is the account of how far that procedure has
been tested.

---

## 2. Two different pendrives, which must not be conflated

| | **Replication pack** | **Unit-making drive** |
|---|---|---|
| What is on it | the whole project folder | `provisioning/pendrive/` only |
| Purpose | rebuild the entire system from nothing | add units to an existing fleet |
| Entry point | `./setup-server` | `./make-units` |
| Carries the CA key? | only if one is placed in `ca/` | yes, always; that is its reason to exist |
| Instructions | `START_HERE.md` | `pendrive/README.txt` |
| Security posture | the CA ends up on the server, a prototype trade-off | the CA never touches the server: unplug the drive and no one can add a unit, not even from the server |

Both are built. The second is the production posture; the first is what someone
standing the system up from scratch uses.

---

## 3. What `./setup-server` does

`setup-server` is a wrapper that runs `deploy/server/install_hub.sh`. Eight
phases, each idempotent, `--check` for a dry run, `--only <phase>` to repeat
one. The phases run in this order, verified in the installer's own loop:

| Phase | What it does |
|---|---|
| **1 system** | `apt-get install` of python3, venv, pip, nginx, avahi-daemon, avahi-utils, sqlite3, curl, git, openssl |
| **2 pki** | Creates a certificate authority, **or adopts one found in `ca/`**. This is what allows a dead server to be replaced without touching a single deployed unit |
| **3 server** | Runs `install_server.sh`: virtual environment, `pip install` (which compiles wolfSSL from source), systemd units, nginx with the mutual-TLS gate, cron entries for retention and daily statistics. A running server is left alone unless `OMEGA_FORCE_SERVER=1` |
| **4 toolchain** | `arduino-cli`, the ESP32 core, and the firmware libraries pinned to `wolfssl@5.8.4` and `ArduinoJson@7.4.3` |
| **5 tools** | Installs `omega-make-units` and `omega-{make-amu-bundle, clone-amu-card, prepare-amu-card, patch-amu-card, read-amu-card}` into `~/bin`, linked into `/usr/local/bin` so they work under `sudo` |
| **6 operator** | Issues the operator certificate and `operator.p12` for the dashboard |
| **7 console** | Installs the operator console, registers the MCP tools, sets the dashboard to open on login. Non-fatal if the machine is offline: an offline hub is still a correct fleet server |
| **8 doctor** | Verifies services are active, the CA is present, the unit factory is ready, and mDNS is advertising |

**The one irreversible decision is phase 2.** The script states which way it is
about to go and waits for a typed `yes`: adopt the fleet's existing CA, so
deployed units keep working, or create a new one, in which case every unit must
be rebuilt.

---

## 4. The full operator sequence

Taken from `START_HERE.md` and verified against the scripts themselves.

**Preparation**

1. Copy the project folder onto the machine.
2. To rebuild an existing fleet, place `ca-key.pem` and `ca-cert.pem` into
   `ca/`. For a new fleet, leave `ca/` empty.
3. Keep the air-unit golden image at `~/amu-golden.img` on the server or an
   external drive. It does not fit on a pendrive (section 5).

**Server**

```
# copy the folder off the pendrive onto the machine first: the installer
# writes as it goes and a pendrive is slow
./setup-server            # or --check to see what it would do
```

**Operator access**

```
omega-install-operator-cert     # installs the browser credential, opens the dashboard
```

**Noise units (NMU).** Plug a board in over USB, then:

```
sudo omega-make-units
```

It asks for the Wi-Fi network name and password, how many units to build, and
each unit's name, suggesting the next free one. Then, per unit, it builds that
unit's own firmware with its own certificate compiled in, flashes it with
`esptool`, verifies `Hash of data verified`, and says when to swap boards.

**Air units (AMU).** There are two distinct paths, and they take different
cards. Choosing the wrong one is the single easiest mistake to make here.

*Way 1, from the golden image, the normal path.* Requires `~/amu-golden.img`
and a **blank** SD card:

```
sudo omega-clone-amu-card AMU_11
```

This writes the image, then removes the donor's private key, certificate, SSH
host keys, random seed, logs and machine identifier, issues the new unit its
own identity from the CA, and asks for the Wi-Fi network name and password,
defaulting to the donor's. `ca-cert.pem` is deliberately kept, because it is
the shared trust anchor.

*Way 2, without a golden image.* Write a card with Raspberry Pi Imager first,
then:

```
sudo omega-prepare-amu-card AMU_11
```

This writes a unit's software and identity onto an **already imaged** card so
the Pi installs itself on first boot. It does not write an image, and it will
refuse a blank card.

**Making the golden image in the first place**, from a card taken out of a
working unit:

```
sudo omega-read-amu-card          # writes ~/amu-golden.img
```

**Repairing a deployed air unit without erasing it:**

```
sudo omega-patch-amu-card         # keeps name, identity and unsent readings
```

---

## 5. The three things that deliberately do not travel

`START_HERE.md` section 10 states all three. They are consequences of the
design, not oversights.

1. **The air-unit golden image (about 29 GB) is not in the repository.** It does
   not fit on a pendrive and does not belong in version control. It is made once
   with `omega-read-amu-card` from a working unit and kept with the server.
   Without it, the server and every noise unit still build normally; only air-unit
   cloning needs it, and Way 2 above exists for that case.
   *Why a golden image at all:* an air unit needs compiled sensor libraries and
   enabled I2C and UART buses that take a long time to install from scratch and
   fail obscurely. Cloning a unit that already works removes that entire class
   of failure.
2. **The statistics endpoint is not filled in.** It cannot be guessed. Until
   `OMEGA_STATS_URL` and `OMEGA_STATS_TOKEN` are set, the daily report is
   printed to a log rather than sent. Nothing fails and no report is lost.
3. **The CA private key does not travel in the repository.** That is precisely
   the point of it being a secret.

---

## 6. What has been verified, and the one gap that has not

**Verified by running it, not by assuming it.** The replication pack was
exercised on the server against a clean copy of itself, with
`./setup-server --check` green on all eight phases. Both certificate-authority
refusal paths were made to fire: an empty `ca/` warns that this will start a
new fleet, and a half-present CA refuses outright. A carried CA was adopted
with a byte-identical SHA-256 fingerprint at mode 600, which is the exact
property that lets a dead server be replaced without touching a deployed unit.

Three further claims were audited by execution:

- **Operator certificate: pass.** `certutil` shows the CA and the operator key
  in both browser profiles, and the mutual-TLS gate answers 400 without the
  credential and 200 with it.
- **Unit factory: pass.** The firmware compiles clean on the server through the
  factory's own template and board identifier, at 1,313,377 bytes, 62% of flash
  and 25% of RAM.
- **Operator console: pass.** Installed and running. Exercising it exposed two
  real defects, both fixed: the phase resolved the console binary before
  installing it, so MCP registration was skipped silently, and the registration
  itself was guarded on a path that did not exist.

**Three defects that only a fresh install could expose, all found this way and
fixed.** Each was invisible on the development machine, which already had every
tool present:

1. `install_hub.sh` looked for the card tools only in `provisioning/`, but
   `read-amu-card` lives in `provisioning/pendrive/`, so a fresh install would
   have shipped without the one tool that makes the golden image. The lookup now
   tries both locations.
2. The same tool loop omitted `clone-amu-card`, so a fresh install shipped
   without the only tool that writes the golden image to a card. Added to the
   loop; the installer version was bumped to `2026-08-28a-clone-amu-card`.
3. `START_HERE.md` told the operator to use `prepare-amu-card` on a blank card
   and claimed it writes the image. It does neither: it contains no `dd` and no
   `losetup`, and it refuses a card that Raspberry Pi Imager has not written.
   Following the manual on a fresh install therefore failed at the first air
   unit. Both paths are now described correctly, and the installer's closing
   summary was updated to match.

**The remaining gap, stated plainly.** `--check` is a dry run. **A single
uninterrupted destructive install on a genuinely bare Ubuntu machine has not
been observed.** Individual phases have been run for real, including the
console, the operator certificate and a toolchain compile, and the dry run
passes end to end, but the whole sequence has not been watched through on a
clean machine in one go.

Accordingly, the honest description of this pack is: **verified phase by phase
and dry-run green end to end, with the fresh-install-only defects it exposed
found by that testing and fixed; one uninterrupted bare-metal run is the
remaining unevidenced step.**

---

## 7. The highest-value next verification

For anyone continuing this work, the single most useful remaining check is one
real `./setup-server` run on a clean Ubuntu virtual machine.

- **Cost:** roughly an hour, most of it unattended.
- **Risk:** none to a deployed fleet. It runs in a virtual machine, with `ca/`
  left empty so it builds a throwaway certificate authority, and no hardware
  attached.
- **Payoff:** the central deployment claim moves from *designed and dry-run
  verified* to *demonstrated*, and whoever inherits the pack has one that is
  known to work rather than believed to.

Nothing is changed by doing this; the sequence is only run. If it passes, one
sentence in the documentation changes. If it fails, that is exactly the thing
worth discovering deliberately rather than by accident.
