# Omega user manual

**What this is.** Everything you need to build the system, run it, and change
it. No programming required. Commands are meant to be copied and pasted
exactly as printed.

If you only want to know *which file to edit to change something*, skip to
[CONFIGURATION.md](CONFIGURATION.md).

---

## 1. What the system is

Three kinds of machine, and one rule that ties them together.

| | What it is | What it does |
|---|---|---|
| **Server** | An ordinary PC running Ubuntu | Receives readings, stores them, shows the web page |
| **NMU** | A small box with a microphone | Measures noise, sends when it hears something |
| **AMU** | A small box with a Raspberry Pi inside | Measures air quality, sends on a timer and on alarms |

The rule: **every box and the server carry an identity certificate signed by
one certificate authority.** Before a single reading is exchanged, each side
proves its identity to the other and they agree on a fresh encryption key.
Nothing on the network can read the readings, and nothing without a valid
certificate can send one.

That is what DTLS 1.3 does, and both box types use it — the noise units and
the air units alike.

The authority has two halves:

- **`ca-cert.pem`** — the public half. It is on every box and on the server,
  and it is what the running system actually uses: each side checks the other's
  certificate against it at every handshake.
- **`ca-key.pem`** — the private half. It is used **only** when a new box is
  built or the server is rebuilt. The running system never touches it.

**The private half does not have to be on the server.** Keeping it there is a
convenience — it lets you make a new box on the spot without fetching a drive.
It can instead live only on a USB stick in a drawer, off the internet, and be
plugged in just for the few minutes it takes to build a box. That stricter
arrangement is [deploy/SETUP_MANUAL.md](deploy/SETUP_MANUAL.md), "Path B".
Either way:

> **Back up `ca-key.pem` together with `ca-cert.pem` to a second offline
> place.** If you lose the private half you cannot make any new boxes, and
> every existing box has to be rebuilt from scratch. Anyone who *copies* it can
> make a box your whole fleet will trust — so it goes in exactly two places and
> nowhere else.

---

## 2. Before you start

You need:

- **A PC for the server**, with Ubuntu already installed and connected to the
  network. Installing Ubuntu itself is not covered here.
- **An internet connection on that PC.** The setup downloads about 600 MB of
  tools. Without internet, setup stops partway.
- **A USB stick** with this project folder copied onto it.
- **An SD card reader** on the server, if you will build air units.
- **A USB cable** to plug noise units into the server.

**Ports.** If a firewall is in the way, the server needs these open:

| Port | Why |
|---|---|
| UDP 11400 | Readings arrive here |
| UDP 5001 | Boxes ask "where is the server?" here |
| UDP 5353 | Name lookup (`smartageing.local`) |
| TCP 443 | The dashboard |
| TCP 80 | Redirects to 443 |

---

## 3. Build the server

**Copy the project folder off the USB stick onto the PC first.** The installer
writes as it goes and a USB stick is slow.

Then, in a terminal, inside that folder:

```
./setup-server
```

Do **not** put `sudo` in front of it. It asks for your password when it needs
to install software; everything else it makes belongs in your own home folder.

It will tell you one thing and wait:

- **"EXISTING fleet"** — it found a certificate authority in the `ca/` folder
  and will keep using it. Boxes you already built keep working.
- **"NEW fleet"** — `ca/` is empty, so it makes a brand-new authority. Any box
  built for a different authority will never talk to this server again.

Type `yes` to go ahead.

> **Rebuilding a server for boxes that already exist?** Stop, and put that
> fleet's `ca-key.pem` and `ca-cert.pem` into the `ca/` folder *before* you
> run this. There is no way to fix it afterwards.

It takes a while. When it finishes it prints what to do next.

### Open the dashboard

The dashboard deliberately refuses everyone who cannot prove who they are, so
you need a pass installed in the browser first. On the server:

```
omega-install-operator-cert
```

Then open **https://smartageing.local** in Firefox. It will ask which
certificate to present — choose **operator**.

If the page is refused, that is the gate working, not a fault.

### Check it worked

```
./setup-server --check
```

It reports each part as present or missing and changes nothing.

---

## 4. Build a noise unit (NMU)

Plug the ESP32-S3 board into the **server** by USB, then:

```
sudo omega-make-units
```

It asks for:

1. **The WiFi network name and password** the boxes will use.
2. **How many units** you want to make.
3. **A name for each**, suggested automatically (`NMU_16`, `NMU_17`, …).

Then it swaps boards on your prompt: it flashes one, tells you to unplug it
and plug in the next, and repeats.

Each unit gets its own certificate and its own private key, built into the
firmware. **Two units never share an identity**, so if one box is stolen, only
that box is compromised.

### If a board is not found

Put it into flashing mode by hand. **Leave the cable plugged in the whole
time:**

1. Hold down the small **BOOT** button and keep holding it.
2. Press **RESET** once and let go of RESET.
3. Now let go of **BOOT**.

Then run the tool again and give that unit **the same name as before**.

### After flashing

The unit reboots by itself. Give it about a minute — it waits a random
few seconds on purpose, so that a whole shelf of units switched on together
does not contact the server all at once.

You do **not** tell a noise unit the server's address. It finds the server on
its own, and keeps working if the server's address changes.

---

## 5. Build an air unit (AMU)

There are three ways. Which one you use depends only on what you are starting
from.

**All three end the same way:** put the card in the Pi and switch it on.

### The short version

| Situation | Use |
|---|---|
| You have a golden image and a blank card | **Way 1** — fastest |
| No golden image | **Way 2** — write a card with Raspberry Pi Imager |
| The Pi is already running on the network | **Way 3** — install over the network |

> **A golden image is a complete copy of a working air unit.** It exists
> because an air unit needs software that takes 20–40 minutes to compile and
> fails in obscure ways; copying a unit that already works avoids all of it.
>
> **The image is not supplied with this project** — it is far too large to
> publish. So if you are starting fresh, **build your first air unit with
> Way 2**, then make an image from it (below) and use Way 1 for all the rest.

---

### Way 1 — from the golden image

Needs `~/amu-golden.img` on the server. Put a **blank SD card** in the
server's card slot, then:

```
sudo omega-clone-amu-card AMU_11
```

It asks you to **type the device path back** to confirm, because this erases
the card completely. Then it asks for the WiFi network name and password
(press Enter twice to keep the ones the image already had).

It writes the image, strips out the donor unit's identity — its private key,
its certificate, its SSH keys, its machine id — issues this unit its own
identity, and sets the hostname.

Takes several minutes.

### Way 2 — from a card written by Raspberry Pi Imager

**Step 1. On the server, make this unit's software bundle:**

```
omega-make-amu-bundle AMU_11
```

This issues AMU_11 its certificate and packs its software. **Do not skip it** —
step 3 refuses to run without it.

**Step 2. Write an SD card with Raspberry Pi Imager.** At *"Would you like to
apply OS customisation settings?"* choose **EDIT SETTINGS** and fill in:

| Setting | Value |
|---|---|
| **hostname** | `amu11` — see the box below |
| **username** | `amu` |
| **wireless** | your network name and password |
| **services** | enable SSH with password authentication |

Then click **YES** to apply them.

> **The hostname rule, and it is strict.** Take the unit name, make it
> lowercase, and **delete the underscore**:
>
> | Unit | Hostname |
> |---|---|
> | `AMU_11` | `amu11` |
> | `AMU_12` | `amu12` |
>
> `amu-11` and `amu_11` are both **wrong** and the tool will refuse the card.
> That refusal is deliberate: a hostname the fleet cannot resolve produces a
> unit that installs perfectly and is then unreachable.

**Step 3.** Put that card in the **server**, and:

```
sudo omega-prepare-amu-card AMU_11
```

It does not write an image. It puts this unit's software and identity onto the
card you already made, switches on the I2C and serial buses the sensors need,
and arms an installer that runs the first time the Pi is switched on.

**Step 4.** Put the card in the Pi and power it on.

**First boot takes up to 40 minutes.** Most of that is compiling the security
library. It is not stuck. Watch it from the server with:

```
ssh amu@amu11.local 'sudo cat /var/log/omega_firstboot.log'
```

If it fails it tries again, up to five times, a minute apart.

### Way 3 — onto a Pi that is already running

Use this when the Pi is already booted and on the network, and you have no
card reader on the server.

**First, on the Pi, switch on the sensor buses** — Way 3 is the only way that
does not do this for you:

```
sudo raspi-config
```

Enable **I2C** and **Serial Port** (hardware serial **on**, login shell over
serial **off**), then reboot the Pi.

**Then, on the server:**

```
omega-make-amu-bundle AMU_11
```

It prints the exact two commands to run. They look like this:

```
scp ~/amu_bundles/AMU_11.tar.gz amu@amu11.local:~/
ssh amu@amu11.local 'tar -xzf AMU_11.tar.gz && ./AMU_11/install_amu.sh'
```

The install takes 20–40 minutes.

### Make a golden image, once you have one working unit

Take the SD card out of a working, up-to-date air unit, put it in the server,
and:

```
sudo omega-read-amu-card
```

That writes `~/amu-golden.img`, which is where Way 1 looks. It takes several
minutes; if interrupted, run it again and it continues.

Do this once, and again after a software fix, so new units start up to date.

### Repair a unit already in service

Put its card in the server and run:

```
sudo omega-patch-amu-card
```

It installs the current software while **keeping** the unit's name, its
identity, and any readings it had not yet sent. It is also the tool that
installs remote access and switches on the hardware watchdog.

---

## 6. Using the system day to day

Open **https://smartageing.local** on the server.

### The dashboard

- **Time buttons** — 15 minutes, 1 hour, 6 hours, 24 hours, 7 days. They
  change what the two charts show.
- **Noise chart and air chart** — one dot for every reading as it arrives, not
  a joined line. Each unit has its own colour, derived from its name, so it
  stays the same every time. A reading that triggered an alarm is drawn as a
  larger solid dot.
- **Event log** — the most recent readings as they arrive, newest first.
- **Alarm banner** — appears across the top when a unit reports an alarm.

### Retiring or suspending a box

In the **key panel**, each unit has a status and a button.

- **Revoke** — the server refuses that unit from its very next transmission,
  even though its certificate is still cryptographically valid. Use this the
  moment a box is lost or stolen.
- **Restore** — puts it back.

The change takes effect on the next reading. Nothing needs restarting.

### Changing how often boxes report

In the **heartbeat panel**, set a number of minutes for noise units, air
units, or both, and press **Program & send on next ACK**.

Each box is told its new interval inside the reply to its next reading. You do
not have to touch the boxes, and nothing restarts.

### Asking the fleet questions in plain language

On the server, type `claude` and ask. Nine tools are available:

**Reading stored data** (the database only, no box is contacted):

| Tool | Answers |
|---|---|
| `list_devices` | Every unit that has ever reported, its type, how many readings, when last seen |
| `latest_reading` | That unit's most recent stored reading |
| `device_stats` | Minimum, maximum, average, median and spread per measurement |
| `activity_report` | What a unit has been doing over a period |
| `set_heartbeat` | Changes the reporting interval, same as the dashboard |

**Asking a box right now** (air units, which are always on):

| Tool | Answers |
|---|---|
| `live_read` | A fresh reading taken now, about a second |
| `live_status` | Whether that unit is reachable and answering |

**Asking a noise unit** (which never accepts incoming connections, so the
question waits for it to make contact):

| Tool | Answers |
|---|---|
| `nmu_read_now` | The current sound level |
| `nmu_status` | Free memory, how many readings are buffered, how long it has been up |

Examples: *"which units have reported in the last hour?"*, *"what is AMU_11
reading right now?"*, *"set the air units to report every 10 minutes"*.

### The daily report

Every night at 03:30 the server prepares a summary for each unit and each
measurement, and sends it to the research endpoint.

The system is delivered pushing to a **local stand-in endpoint**, so the path
is proven end to end before a real one exists. Pointing it at the real address
is an edit to two lines in one file and needs no restart — including what the
receiving side must accept. See
[CONFIGURATION.md §7](CONFIGURATION.md#7-the-daily-report).

At 03:15 the database checks its own size and trims the oldest readings if it
has grown past its limit.

---

## 7. When something is wrong

| What you see | What it means | What to do |
|---|---|---|
| Dashboard refuses you | The certificate gate is working | Run `omega-install-operator-cert`, then choose **operator** when Firefox asks |
| Dashboard does not load at all | The gateway is down | On the server: `sudo nginx -t` |
| A unit stopped reporting | Could be power, WiFi, or revocation | Check the key panel first — a revoked unit is refused on purpose |
| A new air unit never appears | First boot is still running | `ssh amu@amu11.local 'sudo cat /var/log/omega_firstboot.log'` |
| `omega-prepare-amu-card` refuses the card | The Imager hostname does not match | Re-image with the right hostname — `AMU_11` needs `amu11` |
| `omega-prepare-amu-card` says there is no bundle | Step 1 was skipped | Run `omega-make-amu-bundle AMU_11` first |
| A board is not found when flashing | It is not in flashing mode | Hold BOOT, tap RESET, release BOOT — cable stays in |
| Nothing arrives from any unit at all | The server may be listening on the wrong port | `ss -lunp \| grep 11400` — it must be there |
| You edited something and nothing changed | Whatever reads that file has not re-read it | [CONFIGURATION.md §11](CONFIGURATION.md#11-what-to-restart-after-a-change) lists what to restart, per file |
| The nightly report says `REFUSED` | The endpoint address is not `https` | [CONFIGURATION.md §7](CONFIGURATION.md#7-the-daily-report) — the refusal is the safety check doing its job |

Health check, any time, changes nothing:

```
./setup-server --check
```

More detail: [deploy/TROUBLESHOOT.md](deploy/TROUBLESHOOT.md).

---

## 8. What is deliberately not automatic

These are decisions, not gaps.

- **A new unit is not trusted automatically.** Someone with access to the
  server has to build it. There is no way for a box to join by asking.
- **The certificate authority is never contacted while the system runs.** It
  signs identities when a unit is made, and takes no part afterwards.
- **Boxes are never told the server's address by hand.** They discover it, so
  a change of address never leaves a unit stranded.
- **The dashboard is reachable only through the certificate gate.** The web
  application itself listens only on the server's own internal address, so
  there is no way around the gate from another machine.

---

## 9. Where to read more

| Document | What it covers |
|---|---|
| [CONFIGURATION.md](CONFIGURATION.md) | Every setting you can change, what it does, and what breaks |
| [START_HERE.md](START_HERE.md) | The same build sequence, shorter |
| [deploy/TROUBLESHOOT.md](deploy/TROUBLESHOOT.md) | Symptom-by-symptom fault finding |
| [deploy/MCP_LIVE.md](deploy/MCP_LIVE.md) | The AI operator layer in depth |
| [deploy/SETUP_MANUAL.md](deploy/SETUP_MANUAL.md) | The step-by-step path, for when the one-command setup cannot be used |
| [evidence/](evidence/) | The recorded experiments behind every claim made here |
