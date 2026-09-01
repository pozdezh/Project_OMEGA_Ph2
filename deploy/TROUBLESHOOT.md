# Project Omega - Brick 4 Troubleshooting

Start with the built-in health check on each machine:

- Server: `./install_server.sh --check`
- AMU: `./install_amu.sh --check`

Each line reports OK or what is missing. Most problems below were seen on real
hardware and are now guarded against by the installers.

---

## Server

**Dashboard does not load at http://server-ip:8080**
- `sudo systemctl status omega-web` - is it active? Restart: `sudo systemctl restart omega-web`.
- Check the firewall allows port 8080.

**A device connects but is rejected / no data appears**
- `journalctl -u omega-listener -f` and watch while the device reports.
- "REJECTED connection: identity=... (not allowed/revoked)" means the device id
  is in the `revoked` list or not in `devices` in `device_config.json`. Fix it
  in the dashboard or edit `device_config.json` (the listener hot-reloads it).
- If EVERY device is rejected, the config is likely stale. The installer now
  replaces an old-format config automatically; if you edited it by hand, make
  sure it still has `nmu`, `amu`, `devices` and `revoked` keys.

**Listener will not start / "missing ...pem"**
- The three server files must be in the target `pki/` folder:
  `ca-cert.pem`, `omega-server-cert.pem`, `omega-server-key.pem`. Re-copy from
  the server's `./pki` and re-run the installer.

**A device handshake fails ("no session")**
- The device's certificate must be signed by the SAME CA as the server's. If
  you re-ran `omega_pki.py init`, you made a NEW CA - re-issue and re-flash all
  device certs, and reinstall the server cert.

---

## AMU (Raspberry Pi)

**Installer refuses with [E51] "do NOT run as root"**
- Correct behaviour. Run it as the normal user: `./install_amu.sh` (no sudo).
  Running as root creates a second process that fights the sensor GPIO.

**Service fails: "ModuleNotFoundError: No module named 'board'"**
- The sensor libraries are missing from the service venv. Re-run `./install_amu.sh`
  (it installs them into the venv and verifies `import board`). If it still
  fails, enable I2C/SPI: `sudo raspi-config` -> Interface Options.

**"Unable to set line 4 to input" / GPIO busy**
- Two processes are using the DHT pin. Make sure only the systemd service runs:
  `sudo systemctl restart omega-amu`, and that you did NOT also start the script
  by hand or via cron. A reboot clears a stuck pin.

**No data reaching the server**
- `journalctl -u omega-amu -f`. Look for "DTLS connect failed" (wrong
  server_host, server down, or cert mismatch) vs sensor errors.
- Confirm `server_host` in `config/global.ini` is the server's current IP.
- Verify identity files exist: `ls ~/omega_amu/pki` should show `ca-cert.pem`
  and this unit's `AMU_0X-cert.pem` / `-key.pem`.

**Readings pause then resume**
- Normal. On a lost session the unit buffers events and flushes them when the
  session returns; heartbeats are not buffered.

---

## NMU (ESP32-S3)

**Upload fails: "Checksum error" / "chip stopped responding"**
- Use a USB 2.0 port and a known-good data cable. Set upload speed to 115200.
  If needed, hold BOOT, tap RESET, release BOOT, then upload.

**Serial Monitor shows nothing**
- Set the monitor to 115200 baud. Note: native USB Serial drops during light
  sleep - that silence is expected between events.

**"handshake failed" on Serial**
- The `omega_certs.h` on the device must be this unit's, signed by the current
  CA, and `config.h` `OMEGA_SERVER_IP` must be the server. Regenerate with
  `omega_pki.py arduino ./pki NMU_0X omega_certs.h` and re-flash.

**Compile error about a certificate string**
- `omega_certs.h` must be present next to the sketch. Regenerate it; do not edit
  the PEM blocks by hand.

---

## Certificates / identity (all devices)

- One CA signs everyone. If you ever regenerate the CA, EVERY device and the
  server must get fresh certs. Prefer `omega_pki.py device ...` to add units
  without touching the CA.
- `ca-key.pem` must never be on a device and never committed to git.
- Device private keys (`*-key.pem`, `omega_certs.h`) are secret; the installers
  set key files to mode 600.

---

## Live query ("ask the AMU now") not answering

Symptom: an operator query returns `via: "queued"` instead of `via: "direct"`,
or reports `answered: false`.

This is not a fault by itself. There are two routes to a device and the system
falls back automatically:

- `via: "direct"` - the server called the AMU and it answered, about 1 second.
- `via: "queued"` - the direct call did not answer, so the question was left
  for the device to collect on its next transmission. Normal for an NMU, which
  never accepts incoming calls at all.

Check in this order:

1. `systemctl is-active omega-amu-live` on the Pi. If it is not active, only
   the queued route exists. Start it: `sudo systemctl restart omega-amu-live`.
2. `journalctl -u omega-amu-live -n 30` - look for `live: read_now -> True`.
3. Confirm the server knows where to call. The device must appear under
   `"endpoints"` in the server's `device_config.json`, as a HOSTNAME (e.g.
   `amu1.local`), never an IP - the address is resolved fresh on every call so
   a DHCP change needs no edit here.
4. The caller must be the SERVER's own certificate, not the operator's. This
   is deliberate: an operator credential that leaks cannot reach devices
   directly, only the server.

If the queued route also returns nothing, the device simply has not
transmitted yet. A quiet AMU speaks at most every 5 minutes (its heartbeat).
The question stays queued - asking again shortly collects the answer.

## A brand-new unit fails immediately with "ModuleNotFoundError"

Fixed 2026-08-21, but the symptom is worth recognising. Both installers used
to omit one file each that their own code imports, so a FRESH install died at
startup while every existing machine kept working. If this ever recurs, run

```
py -3.12 deploy/test_installer_payload.py
```

which names the missing file and the module that imports it. Then re-run the
installer.

**Dashboard still shows the old page after editing templates/index.html**
- Restart the web service: `sudo systemctl restart omega-web`, then Ctrl+F5
  in the browser. app.py runs Flask with `debug=False`, which compiles each
  template once and caches it in memory - the file on disk is never re-read
  while the process lives. Restarting is safe at any time: omega-web only
  serves the dashboard, and the listener is a separate service, so no device
  session drops and no reading is lost.
