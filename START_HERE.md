# Omega - build the whole system from nothing

This folder is everything needed to recreate the sensor system: the server,
the dashboard, the certificates, and the tools that build new sensor units.

You do not need to be a programmer. You need one Ubuntu machine, a pendrive,
and about an hour for the first install.

---

## 1. What you are building

**Sensor units** measure noise (NMU) and air quality (AMU) and send readings
over the network. **A server** receives them, stores them, shows them on a web
page, and keeps itself tidy.

Everything is encrypted, and both ends prove who they are before a single
reading is sent. A unit that is not part of the fleet cannot join it, and a
unit that is removed can be locked out permanently.

| Piece | What it is |
|---|---|
| **NMU** | Noise unit. A small microcontroller board with a microphone. |
| **AMU** | Air unit. A Raspberry Pi with air-quality sensors and an SD card. |
| **Server** | An ordinary Ubuntu PC. Receives, stores and displays everything. |
| **Certificate authority** | The thing that decides who belongs to the fleet. |

---

## 2. What you need before starting

**Hardware**

- One PC with Ubuntu installed and nothing else on it. This becomes the server.
- A pendrive (USB stick), 1 GB or more.
- The units themselves: ESP32-S3 boards for NMUs, Raspberry Pi 4 plus SD cards
  for AMUs.
- An SD card reader in or on the server.

**The two things that must travel with you**

1. **This folder**, copied onto the pendrive.
2. **The fleet's certificate authority** - two files, `ca-key.pem` and
   `ca-cert.pem`, placed in the `ca/` folder inside this one.

   *Only if you are rebuilding an existing fleet.* For a brand-new fleet,
   leave `ca/` empty and one will be created for you.

**One thing that does not fit on the pendrive**

The AMU **golden image** is about 29 GB - a complete copy of a working air
unit's SD card. It is too big for a pendrive and is not in this folder.

Keep it on the server, or on an external drive plugged into the server, at:

```
/home/<your-user>/amu-golden.img
```

If you do not have one, section 6 explains how to make it from a working unit.
Without it you can still install the server and build NMUs - only AMU cloning
needs it.

---

## 3. Install the server

Copy this folder onto the pendrive. Plug the pendrive into the Ubuntu machine
and copy the folder onto the machine itself - do not run it from the pendrive,
because the installer writes as it goes and a pendrive is slow.

Open a terminal in the folder and run:

```
./setup-server
```

It tells you whether it is adopting an existing certificate authority or
creating a new one, and waits for you to type `yes`. **Read that line.** It is
the one choice that cannot be undone later.

It will ask for your password when it needs to install software. Everything
else is automatic.

**It is safe to run again.** If it fails halfway, fix what it complains about
and run it again - it continues rather than starting over.

To see what it would do without changing anything:

```
./setup-server --check
```

---

## 4. After the install

**Open the dashboard.** On the server itself:

```
omega-install-operator-cert
```

That puts your pass into the browser and opens `https://smartageing.local`.

The dashboard refuses anyone without that pass. **That is the security working,
not a fault.** To view it from another computer, copy `operator.p12` from
`~/omega_brick4/operator/` onto that computer and import it into the browser as
a personal certificate. The password is `omega` unless it was changed.

**Ask the fleet questions in plain language.** On the server:

```
claude
> which units have reported in the last hour?
> what is the temperature in AMU_12 right now?
```

This is Claude Code, installed for you. It has a fixed set of tools for
reaching the fleet - it can only call those, and it cannot invent commands
that reach a device.

---

## 5. Build a noise unit (NMU)

Plug the ESP32-S3 board into the server by USB, then:

```
sudo omega-make-units
```

It finds the board, gives the unit its own identity, builds the software and
writes it to the board. The unit appears on the dashboard by itself within a
minute or two.

Each unit gets its own certificate. **No two units share an identity**, so one
unit being lost or stolen never affects the others.

---

## 6. Build an air unit (AMU)

There are **two ways**, and which one you use depends only on what you are
starting from. Both end with a card you put in the Pi and switch on.

### Way 1 - from the golden image (the normal way)

A **golden image** is a complete snapshot of a working unit. This is
deliberate: an air unit needs compiled libraries and enabled hardware that take
a long time to install from scratch and fail in obscure ways. Copying a unit
that already works avoids all of it.

You need `~/amu-golden.img` on the server (see below). Put a **blank SD card**
in the card slot, then:

```
sudo omega-clone-amu-card AMU_11
```

It writes the image, removes the donor unit's identity - its private key, its
certificate, its SSH host keys and its machine id - issues this unit its own
identity, and asks for the WiFi network name and password.

### Way 2 - from a card written by Raspberry Pi Imager

Use this when you have no golden image. Write a card with **Raspberry Pi
Imager**, setting the hostname, the username `amu`, and the WiFi there. Then:

```
sudo omega-prepare-amu-card AMU_11
```

It does **not** write an image - it puts this unit's software and identity onto
the card you already made, and arms an installer that runs the first time the
Pi is switched on. First boot takes a few minutes.

If you give it a blank card it will refuse, saying it found no Imager settings
on it. That is this tool telling you it is the wrong one for a blank card, not
a fault - use Way 1.

### Making the golden image in the first place

Take the SD card out of a working, up-to-date AMU, put it in the server, and:

```
sudo omega-read-amu-card
```

That writes `~/amu-golden.img`, which is where Way 1 looks. It takes several
minutes. If it is interrupted, run it again and it continues. Do this once, and
again after a software fix if you want new units to start up to date.

**To repair a unit already in service** without erasing it - to install a
software fix, or restore remote access - put its card in the server and run:

```
sudo omega-patch-amu-card
```

It keeps the unit's name, its identity and any readings it had not yet sent.

---

## 7. The daily report to head office

Once a day the server prepares a summary for every unit and every measurement -
lowest, highest, average, middle value and spread - and sends it to an address
you choose.

**Until you give it an address, it prints the report to a log instead of sending
it.** Nothing is lost and nothing fails; it simply has nowhere to go yet.

To turn the sending on, edit the maintenance schedule on the server:

```
sudo nano /etc/cron.d/omega-maintenance
```

Add these two lines near the top, with your real values:

```
OMEGA_STATS_URL=https://your-office-server.example/api/reports
OMEGA_STATS_TOKEN=the-secret-token-they-gave-you
```

Save and exit. The change takes effect at the next daily run (03:30).

The server tries three times with increasing waits if the office server does
not answer, and **prints the report rather than discarding it** if all three
fail - so a report is never silently lost.

---

## 8. About the certificate authority

Every unit is built to trust one authority. The server proves itself with a
certificate from that authority, and so does every unit. That is what stops an
outsider joining the fleet or impersonating the server.

**Two files matter:**

- `ca-cert.pem` - the public half. Safe to copy anywhere. It is on every unit.
- `ca-key.pem` - the private half. **Whoever holds this can create a unit that
  the whole fleet trusts.**

**Keep `ca-key.pem` off the internet and off shared drives.** Store it on a
drive that lives in a drawer. It is not needed for day-to-day running - only
when building a new unit or rebuilding the server.

**To start a completely new fleet** (first installation, or because the private
half leaked):

```
./provisioning/new-fleet-ca
```

It makes you type `NEW FLEET` to continue, moves any previous authority aside
rather than deleting it, and then tells you what to do next. **Every existing
unit must be rebuilt afterwards** - they will not talk to a server holding a
different authority, and no setting changes that.

---

## 9. If something does not work

| What you see | What it means |
|---|---|
| Dashboard says the connection was refused | Your browser has no operator pass. Run `omega-install-operator-cert`. |
| A unit is missing from the dashboard | Give it two minutes. If still missing, check it has power and is on the same network. |
| `command not found` for an `omega-` tool | Run `sudo ln -sf ~/bin/omega-* /usr/local/bin/` |
| The card tool says no card found | The card is not seated, or it is in a reader the machine has not noticed. Re-seat it. |
| Everything looks installed but nothing arrives | Run `./setup-server --check`. It reports each piece as OK or MISSING. |

`deploy/TROUBLESHOOT.md` in this folder goes further.

---

## 10. What is deliberately not automatic

Stated plainly, because a surprise later is worse than a limitation now.

- **The golden image is not included.** It is 29 GB. Make it once from a working
  unit (section 6) and keep it with the server.
- **The head-office address is not filled in.** Nobody can guess it. Section 7.
- **The certificate authority does not travel in this folder.** That is the
  point of it being a secret.
- **This installs everything on ONE machine** - server, authority and factory
  together. That is a prototype choice, chosen because fewer machines means
  fewer things to go wrong during a trial. Anyone with administrator access to
  that machine can add a unit to the fleet. The alternative, already built, is
  to keep the authority on a pendrive instead: see `provisioning/pendrive/`.
