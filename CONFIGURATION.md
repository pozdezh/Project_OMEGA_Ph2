# What to change, and where

Organised by **what you want to do**, not by which file things live in.

Every default printed here was read out of the source. If a value here and the
code ever disagree, the code is right — please report it.

**Three rules before you change anything.**

1. **The server is changed while it runs.** Settings live in files it re-reads,
   or in a service definition you reload.
2. **An air unit is changed on the unit**, in one text file, then restarted.
3. **A noise unit cannot be changed at all without reflashing it.** Everything
   is compiled in. Plan accordingly.

---

## Quick index

| I want to… | Go to |
|---|---|
| Change how often boxes report | [§1](#1-how-often-boxes-report) |
| Retire, suspend or restore a box | [§2](#2-retire-or-restore-a-box) |
| Change the air-quality alarm levels | [§3](#3-air-quality-alarm-levels) |
| Change the noise sensitivity | [§4](#4-noise-sensitivity) |
| Change the WiFi network | [§5](#5-wifi-network) |
| Move the server, or change its address | [§6](#6-moving-the-server) |
| Set up the daily report to the research endpoint | [§7](#7-the-daily-report) |
| Change how much history the database keeps | [§8](#8-database-size) |
| Change how fast a box recovers from an outage | [§9](#9-outage-recovery-timing) |
| Know what I must never touch | [§10](#10-do-not-change-these) |

---

## 1. How often boxes report

**Use the dashboard.** Heartbeat panel, enter minutes, press **Program & send
on next ACK**. Both box types can be set separately. Each box is told its new
value inside the reply to its next reading — no restart, nothing to touch.

This is the right way. The settings below exist only as the value a box uses
*before* the server has told it anything.

| Where | Setting | Default | Note |
|---|---|---|---|
| Server: `~/omega_brick4/device_config.json` | `nmu.hb`, `amu.hb` | `30` | Minutes. What the dashboard writes. |
| NMU: `nmu/omega_config.h:59` | `DEFAULT_HEARTBEAT_INTERVAL_MS` | `300000` | 5 minutes. **Leave it.** See the warning below. |
| AMU: `config/global.ini` | `alarm_heartbeat_interval` | `1800` | Seconds. Used while an alarm is active. |

> **Why the noise unit's built-in default is 5 minutes and not 30.** The
> recovery ladder timings are *derived* from the heartbeat. The code comment
> at `omega_config.h:53-58` records what happened when it was 30: the derived
> reboot step became 105 minutes, so a freshly rebooted unit that still could
> not reach the server sat silent for nearly two hours. This default is only
> ever used during recovery, so it has to be the fast value, not the quiet one.

---

## 2. Retire or restore a box

**Use the dashboard.** Key panel → **Revoke**.

The server refuses that unit from its very next transmission, even though its
certificate is still cryptographically valid and unexpired. **Restore** puts it
back. The change takes effect on the next reading, and nothing restarts.

The list lives in `~/omega_brick4/device_config.json` under `revoked`. You can
edit it by hand; the server notices the file changed and reloads it.

```json
{
  "devices": [],
  "revoked": ["AMU_13"],
  "nmu": { "cfg_ver": 1, "hb": 30 },
  "amu": { "cfg_ver": 1, "hb": 30 }
}
```

> **`devices` is an allow-list, and it is a trap.** Empty means *any box
> holding a certificate from our authority is welcome*, which is the normal
> setting. Adding the **first** name to it does not admit one more box — it
> switches the door from open to invitation-only and locks out every unit
> already in the field.

> If this file is missing or unreadable, the server **refuses to start**. That
> is deliberate: with no file the revocation list would be empty, and a box you
> revoked would be quietly readmitted. A loud failure at startup is better than
> a silent one in a residence.

---

## 3. Air-quality alarm levels

**File:** `config/global.ini` on the air unit, section `[TRIGGER_THRESHOLDS]`.

After editing: `sudo systemctl restart omega-amu`

| Setting | Default | Meaning |
|---|---|---|
| `co2_critical_high` | `1400` | CO₂ alarm, parts per million |
| `pm_critical_high` | `55` | Particulate alarm |
| `temp_critical_high` | `30` | Too hot, °C |
| `temp_critical_low` | `12` | Too cold, °C |
| `hum_critical_high` | `70` | Too damp, % |
| `hum_critical_low` | `25` | Too dry, % |
| `lux_critical_high` | `800` | Too bright |
| `lux_critical_low` | `5` | Too dark |
| `alarm_heartbeat_interval` | `1800` | Seconds between reports while an alarm is active |

**The `_delta` and `_step` settings decide when a *change* is worth reporting**,
so the unit does not send a message every two seconds as a reading wobbles:

| Setting | Default |
|---|---|
| `co2_delta` | `150` |
| `pm_delta` | `15` |
| `temp_step` | `1.5` |
| `hum_step` | `5` |
| `lux_step` | `100` |

> **Do not set these too small.** The CO₂ sensor's own stated accuracy is about
> ±72 ppm at 1400 ppm, so a deadband narrower than that makes the unit flap
> between alarm and clear on sensor noise alone, filling the database with
> events that mean nothing.

---

## 4. Noise sensitivity

**File:** `nmu/omega_config.h`. **Requires reflashing the unit.**

| Line | Setting | Default | Meaning |
|---|---|---|---|
| `11` | `MIC_SENS` | `0.63` | Microphone sensitivity, volts per pascal. Calibration. |
| `12` | `REF_DB` | `94.0` | Reference sound level for the decibel conversion. Calibration. |
| `13` | `WAKEUP_MARGIN_DB` | `6.0` | How far above the room's background a sound must rise to start a recording |
| `14` | `SUSTAIN_MARGIN_DB` | `1.5` | How far above background it must stay to count as still going |
| `15` | `MAX_SILENCE` | `4` | Quiet chunks that end a recording. Each chunk is 125 ms, so 4 = half a second. |
| `16` | `MAX_CHUNKS` | `80` | Longest recording, in chunks. 80 = 10 seconds. |
| `19` | `SENTRY_LOOP_DELAY_MS` | `130` | How often it checks the room |

**Too many events?** Raise `WAKEUP_MARGIN_DB`.
**Missing quiet events?** Lower it.

`MIC_SENS` and `REF_DB` are calibration against a reference meter. Change them
only if you have measured against one.

---

## 5. WiFi network

**Noise units — reflash.** The network is compiled in. Run
`sudo omega-make-units`, give it the new network, and give each unit **the same
name it had before** so it keeps its identity.

**Air units — one file, or one tool.**

- On a running unit: edit `wifi_ssid` and `wifi_pass` in
  `config/global.ini`, then also update the system's own network settings.
- **Easier:** put the unit's card in the server and run
  `sudo omega-patch-amu-card`, which updates both and keeps the unit's
  identity and its unsent readings.

---

## 6. Moving the server

**You usually do not need to do anything.** Boxes are not told the server's
address. They find it, in this order:

1. The address that worked last time, remembered on the box.
2. A name lookup on the local network (`_omega._udp`).
3. A broadcast question: *"is there a server here?"*
4. Only then, a fixed address, if one was configured.

So the server can change address and the fleet follows it. This is why nothing
in the manual asks you to type an IP address into a box.

**If you replace the server machine itself**, that is different and it matters:

> Put the fleet's `ca-key.pem` **and** `ca-cert.pem` into the `ca/` folder
> before running `./setup-server`. The boxes trust the **certificate
> authority**, not the machine. Adopt the old authority and every box keeps
> working without knowing anything changed. Create a new one and every box in
> the field is orphaned, permanently, with no way to fix it except rebuilding
> all of them.

**The port** is `11400`, and it is not a casual setting: it is compiled into
every noise unit and written into every air unit's config. Changing it means
rebuilding the whole fleet. It is set in one place,
`deploy/server/install_server.sh`, as `OMEGA_UDP_PORT`.

---

## 7. The daily report

Every night at **03:30** the server prepares a summary for each unit and each
measurement — minimum, maximum, average, median and spread — and sends it to
the research endpoint over HTTPS with a bearer token.

**Until you fill in the address and the token, it prints the report instead of
sending it.** Nothing is lost and nothing fails.

**File:** `/etc/cron.d/omega-maintenance` on the server. Add:

```
OMEGA_STATS_URL=https://the-endpoint-address/path
OMEGA_STATS_TOKEN=the-token-they-gave-you
```

> **Re-running the installer overwrites this file.** Keep a copy of those two
> lines somewhere, and put them back after an update.

| Setting | Default | Meaning |
|---|---|---|
| `OMEGA_STATS_URL` | *(unset)* | Where to send. Must be `https`. |
| `OMEGA_STATS_TOKEN` | *(unset)* | Bearer token |
| `OMEGA_STATS_WINDOW_H` | `24` | Hours covered by the first report |

If a send fails it retries three times with a growing pause. If all three fail
the report is **printed** rather than discarded, and the "last delivered"
marker does not advance — so the next run covers the missed period too, and no
day is ever silently lost.

---

## 8. Database size

The database trims itself. At **03:15** a job checks its size on disk; if it
has grown past the limit it deletes the oldest readings and compacts the file.

**The trigger is size, not age** — deliberately. A rule like "keep 30 days"
gives a database whose size depends on how noisy the building is; a size rule
gives one that fits the disk you actually have.

**File:** `/etc/cron.d/omega-maintenance`.

| Setting | Default | Meaning |
|---|---|---|
| `OMEGA_DB_MAX_SIZE_MB` | `500` | Trimming starts above this |
| `OMEGA_DB_PRUNE_FRAC` | `0.15` | How much to remove: 15% of the oldest rows |

`OMEGA_DB_PRUNE_FRAC` is clamped between `0.10` and `0.20` whatever you write.
Removing too little means it runs again almost immediately; too much throws
away history nobody asked to lose.

---

## 9. Outage recovery timing

When a box cannot reach the server it works through a ladder of increasingly
drastic remedies, timed from **the last confirmed contact** — not from whether
a socket looks open, which on this kind of network can look fine for hours
after the other end has gone.

**Noise unit** — `nmu/omega_config.h`, **requires reflashing**:

| Line | Setting | Default | Meaning |
|---|---|---|---|
| `50` | `WIFI_CONNECT_TIMEOUT_MS` | `12000` | Wait for WiFi and an address before giving up |
| `51` | `DISCOVERY_TIMEOUT_MS` | `2500` | Budget for the "where is the server?" broadcast |
| `52` | `SYNC_RETRY_INTERVAL_MS` | `300000` | How often to look again while no server is known |
| `63` | `BOOT_JITTER_MAX_MS` | `30000` | Random wait at power-on, so a shelf of units does not connect in lockstep |
| `69` | `OUTAGE_RADIO_RESET_MS` | `300000` | Earliest a full radio restart may happen |
| `70` | `OUTAGE_REBOOT_MS` | `900000` | Earliest the unit may reboot itself |
| `95` | `BACKLOG_RETRY_MIN_MS` | `5000` | First retry of buffered readings |
| `96` | `BACKLOG_RETRY_MAX_MS` | `60000` | Slowest that retry becomes |
| `48` | `MAX_MISSED_ACKS` | `5` | Failures before the urgent reconnect loop starts |

> **The two `_MS` values above are floors, not the actual times.** Each step is
> also held clear of that unit's own heartbeat by a multiplier
> (`OUTAGE_RADIO_RESET_BEATS_X10 = 15`, i.e. 1.5 heartbeats;
> `OUTAGE_REBOOT_BEATS_X10 = 35`, i.e. 3.5). The larger of the two wins.
> Silence shorter than a heartbeat is not evidence of a fault — it is what a
> quiet room looks like.

**Air unit** — `amu_config.py`, restart the service after editing:

| Setting | Default | Meaning |
|---|---|---|
| `RECOVERY_TICK_S` | `5.0` | How often the ladder advances |
| `BACKLOG_RETRY_MIN_S` | `5.0` | First retry of buffered readings |
| `BACKLOG_RETRY_MAX_S` | `60.0` | Slowest that retry becomes |
| `BOOT_JITTER_S` | `30.0` | Random wait at power-on |
| `MAX_BUFFER_SIZE` | `100` | Readings held during an outage |
| `MAX_MISSED_ACKS` | `5` | Failures before escalating |

---

## 10. Do not change these

Each of these is safe to read and harmful to edit. They are listed so that
nobody changes one by accident while looking for something else.

| Where | Setting | Why not |
|---|---|---|
| `nmu/omega_config.h:28` | `MIRROR_MAGIC` | Guards the **layout** of readings saved to flash. Change it and every box silently throws away the readings it was holding through an outage. |
| `nmu/omega_config.h:31` | `RTC_STATE_MAGIC` | Guards the counters that survive a self-restart. Change it and units lose their reading numbering across a reboot. |
| `nmu/omega_config.h:107` | `NET_TASK_STACK_BYTES` | The memory the security handshake needs. It was raised to this value to fix a real crash. |
| `nmu/omega_config.h:99-105` | `CPU_MHZ_*`, `RADIO_SLEEP_ENABLED` | Processor speed and radio sleep. Tuned together against the handshake and sampling timings. |
| `nmu/partitions.csv` | the whole file | The flash memory map. **The filename itself matters**: rename it and the build silently uses a different layout that boots and runs fine until the day an over-the-air update is needed. |
| `deploy/.../install_server.sh` | `OMEGA_WEB_HOST` | Keeps the dashboard on the server's internal address only. Change it to `0.0.0.0` and the dashboard — including the buttons that revoke a box — answers anyone on the network with no certificate at all. |
| Server | `OMEGA_PORT` / device port | Compiled into every box. Changing it on one side only means total silence with no error message. |
| Anywhere | `ca-key.pem` | Never copy it anywhere it does not need to be. Anyone holding it can make a box your whole fleet will trust. |

---

## Where the files actually are

**Server** (`~/omega_brick4/`):

| File | What |
|---|---|
| `device_config.json` | Allow-list, revocation list, heartbeats |
| `sensor_data.db` | The readings |
| `pki/` | The server's own certificate and the authority's certificate |
| `pki_provisioning/` | The authority itself, including `ca-key.pem` |
| `/etc/cron.d/omega-maintenance` | The two nightly jobs and their settings |
| `/etc/systemd/system/omega-listener.service` | The telemetry service |
| `/etc/systemd/system/omega-web.service` | The dashboard service |

After editing a `.service` file: `sudo systemctl daemon-reload && sudo systemctl restart omega-listener`

**Air unit** (`~/omega_amu/`):

| File | What |
|---|---|
| `config/global.ini` | Everything an operator changes |
| `pki/` | This unit's certificate and private key |
| `offline_buffer.json` | Readings waiting for the link to come back |
| `amu_config.py` | Timings and sensor tuning |

After editing: `sudo systemctl restart omega-amu`
To watch it: `journalctl -u omega-amu -f`

**Noise unit** (`nmu/`, on the server before flashing):

| File | What |
|---|---|
| `config.h` | WiFi, server address, port. Generated per unit. |
| `omega_certs.h` | This unit's identity. Generated per unit — never edit by hand. |
| `omega_config.h` | Every timing and threshold |
| `partitions.csv` | Flash memory map. Do not touch. |

Any change here means reflashing the unit.
