# Can the whole system be rebuilt from a pendrive? - code audit

Written 2026-08-28 by reading the tooling, not by recollection. Purpose: to
state in the memo, and defend at the viva, exactly what a person with a stock
Ubuntu PC can and cannot rebuild unaided - and to give the supervisors a
procedure that works.

**Scenario tested against the code:** a stock Ubuntu PC, a pendrive, an AMU
golden image, blank SD cards, and bare ESP32-S3 boards. Nothing else.

---

## 1. Verdict

**Yes - the replication path is implemented, coherent, and documented**, with
three prerequisites that are stated openly rather than hidden. One honesty gap
is identified in section 6 and should be declared rather than discovered.

---

## 2. The distinction that must not be blurred

There are **two different pendrives** in this project. Conflating them in the
memo would be a real error.

| | **Replication pack** | **NMU-making drive** |
|---|---|---|
| What is on it | the whole `brick4_dtls13/` folder | `provisioning/pendrive/` only |
| Purpose | rebuild the **entire system** from nothing | add units to an **existing** fleet |
| Entry point | `./setup-server` | `./make-units` |
| Carries the CA key? | only if you put it in `ca/` | **yes, always** - that is its reason to exist |
| Instructions | `START_HERE.md` | `pendrive/README.txt` |
| Security posture | CA ends up **on the server** (prototype trade) | CA **never touches the server**; unplug the drive and nobody can add a unit, not even from the server itself |

Both are real and both are built. The second is the production posture; the
first is what a sponsor uses to stand a system up.

---

## 3. What `./setup-server` actually does

`setup-server` is a wrapper that runs `deploy/server/install_hub.sh`
(`setup-server:96`). Eight phases, each idempotent, `--check` for a dry run,
`--only <phase>` to repeat one.

| Phase | What it does |
|---|---|
| **1 system** | `apt-get install` python3, venv, pip, nginx, avahi-daemon, avahi-utils, sqlite3, curl, git, openssl |
| **2 pki** | Creates a CA, **or adopts one found in `ca/`** - this is what lets a dead server be replaced without touching a single deployed unit |
| **3 server** | Runs `install_server.sh`: venv, `pip install` (which compiles wolfSSL from source), systemd units, nginx + mTLS gate, cron for retention and daily stats. **A running server is left alone** unless `OMEGA_FORCE_SERVER=1` |
| **4 toolchain** | `arduino-cli`, the ESP32 core, and the firmware libraries **pinned**: `wolfssl@5.8.4 ArduinoJson@7.4.3` |
| **5 tools** | Installs `omega-make-units` plus `omega-{make-amu-bundle, prepare-amu-card, patch-amu-card, read-amu-card}` into `~/bin` and links them into `/usr/local/bin` so they work under `sudo` |
| **6 operator** | Issues the operator certificate and `operator.p12` for the dashboard |
| **7 console** | Installs Claude Code, registers the MCP tools, sets the dashboard to open on login. Non-fatal if offline - an offline hub is still a correct fleet server |
| **8 doctor** | Verifies services active, CA present, factory ready, mDNS advertising |

**The one irreversible decision** is phase 2, and the script says which way it
is going and waits for `yes`: adopt the fleet's existing CA (units keep
working) or create a new one (every unit must be rebuilt). `setup-server`'s
own header calls this out before anything runs.

---

## 4. The full operator sequence

From `START_HERE.md`, verified against the scripts.

**Preparation**
1. Copy the `brick4_dtls13/` folder onto the pendrive.
2. If rebuilding an existing fleet, put `ca-key.pem` and `ca-cert.pem` into
   `ca/`. For a new fleet leave `ca/` empty.
3. Keep the AMU golden image at `~/amu-golden.img` on the server or an
   external drive - it does not fit on a pendrive (section 5).

**Server**
```
# copy the folder off the pendrive onto the machine first - the installer
# writes as it goes and a pendrive is slow
./setup-server            # or --check to see what it would do
```

**Operator access**
```
omega-install-operator-cert     # installs the browser pass, opens the dashboard
```

**Noise units (NMU)** - plug a board in by USB, then
```
sudo omega-make-units
```
Asks WiFi SSID, WiFi password, how many units, and each unit's name (with the
next free name suggested). Then per unit: builds that unit's own firmware with
its own certificate compiled in, flashes it with `esptool`, verifies
`Hash of data verified`, and tells you when to swap boards.

**Air units (AMU)** - put a blank SD card in the reader, then
```
sudo omega-prepare-amu-card       # golden image -> a new, named unit
```
Writes the image, **removes the donor's private key, certificate, SSH host
keys, random seed, logs and machine-id**, issues the new unit its own identity
from the CA, and asks for the WiFi SSID and password (defaulting to the
donor's). `ca-cert.pem` is deliberately kept - it is the shared trust anchor.

**Making the golden image in the first place** - card from a working AMU:
```
sudo omega-read-amu-card          # writes ~/amu-golden.img
```

**Repairing a deployed AMU without erasing it**:
```
sudo omega-patch-amu-card         # keeps name, identity and unsent readings
```

---

## 5. The three things that are deliberately not automatic

`START_HERE.md` §10 states all three. They are design consequences, not
oversights, and saying so first is stronger than being asked.

1. **The AMU golden image (~29 GB) is not in the folder.** It does not fit on
   a pendrive. Made once with `omega-read-amu-card` from a working unit and
   kept with the server. Without it, the server and all NMUs still build
   normally - only AMU cloning needs it.
   *Why a golden image at all:* an air unit needs compiled sensor libraries
   and enabled I2C/UART buses that take a long time to install from scratch
   and fail obscurely. Cloning a unit that already works removes that whole
   class of failure.
2. **The head-office endpoint is not filled in.** Nobody can guess it. Until
   it is set, the daily report is **printed to a log rather than sent** -
   nothing fails and nothing is lost.
3. **The CA private key does not travel in the folder.** That is precisely the
   point of it being a secret.

---

## 6. Honesty gap - declare this, do not let it be found

**What was tested (`DEV_CHRONOLOGY.md`, 2026-08-26):** the replication pack
was exercised on the server against a clean copy of the pack, with
`./setup-server --check` **green on all 8 phases**; both CA refusal paths were
made to fire (an empty `ca/` warns that this starts a NEW fleet; half a CA
refuses outright); and a carried CA was adopted with a **byte-identical
SHA-256 fingerprint at mode 600** - the exact property that lets a dead server
be replaced without touching a deployed unit.

Three claims were then audited by running them rather than assuming:
- **Operator certificate: PASS** - `certutil` shows the CA and the operator
  key in both browser profiles, and the mTLS gate answers 400 without the pass
  and 200 with it.
- **NMU factory: PASS** - the firmware compiles clean on the server through
  the factory's own template and FQBN, 1,313,377 bytes (62% flash, 25% RAM).
- **Claude Code: installed and running (2.1.246)** - and running it exposed
  **two real defects** (the phase resolved `claude` before installing it, so
  MCP registration was skipped silently; and the registration was guarded on a
  path that did not exist). Both fixed.

The pack test also caught a gap invisible on the development machine:
`install_hub.sh` looked for the card tools only in `provisioning/`, but
`read-amu-card` lives in `provisioning/pendrive/` - so a fresh install would
have shipped **without the one tool that makes the golden image**. Lookup now
tries both locations.

**Two defects found by this audit and fixed, 2026-08-28.** Both were invisible
on the development machine, which already had every tool:

1. `install_hub.sh`'s AMU tool loop omitted **`clone-amu-card`**, so a fresh
   install shipped without the only tool that writes the golden image to a
   card. Fixed: added to the loop, `HUB_VERSION` bumped to
   `2026-08-28a-clone-amu-card`.
2. `START_HERE.md` §6 told the operator to use `prepare-amu-card` on a **blank
   card** and claimed it "writes the image". It does neither - it contains no
   `dd` and no `losetup`, and it refuses a card with no Raspberry Pi Imager
   settings on it. Following the manual on a fresh install therefore failed at
   the first AMU. Rewritten to describe both paths correctly, and
   `setup-server`'s closing summary updated to match.

**The remaining gap:** `--check` is a dry run. **A single uninterrupted
destructive install on a genuinely bare Ubuntu machine is not evidenced.** Individual
phases have been run for real (console, operator, toolchain compile), and the
dry run passes end to end, but the whole sequence has not been observed on a
clean machine in one go.

**How to state it:** *"The replication pack is verified phase-by-phase and
dry-run green end to end, and the fresh-install-only defects it exposed were
found by that testing and fixed. A single uninterrupted bare-metal run is the
one step not yet evidenced."* That is accurate and still strong - it credits
the testing that genuinely happened.

---

## 7. Recommendation

Given the supervisors intend to **use this pack to continue the project**, the
highest-value remaining verification in the whole project is **one real
`./setup-server` run on a clean Ubuntu VM**.

- **Cost:** roughly an hour, most of it unattended.
- **Risk:** none to the deployed fleet - a VM, `ca/` left empty so it builds a
  throwaway CA, no hardware attached.
- **Payoff, twofold:** the deployment chapter's central claim moves from
  *"designed and dry-run verified"* to *"demonstrated"*; and the supervisors
  inherit a pack that is known to work rather than believed to.

It does not reopen the code freeze - nothing is being changed, only run. If it
passes, one line in the memo changes. If it fails, that is exactly the thing
worth knowing before someone else discovers it.

**This is the user's call, not an automatic action.**
