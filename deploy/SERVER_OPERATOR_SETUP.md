# Brick 4 server operator setup

## Apply the current server version

Copy the current `brick4_dtls13/deploy/server` package to the server, then run the installer from its `deploy/server` directory:

```bash
sudo ./install_server.sh
```

The installer:

- starts `omega-listener` on UDP `11400`
- starts `omega-web` on port `8081`
- enables both services for boot and restarts
- disables the obsolete Brick 1 `smart_listener` and `smart_web` services
- installs the HTTPS dashboard gateway
- advertises `_omega._udp` on port `11400`
- installs retention and daily-stat jobs

The installer preserves the existing device configuration and certificates.

## One manual action required

The server requires the Linux sudo password. Run this after the corrected files have been copied:

```bash
sudo systemctl daemon-reload
sudo systemctl restart omega-listener omega-web
sudo systemctl disable --now smart_listener.service smart_web.service
sudo systemctl restart nginx avahi-daemon
```

Check:

```bash
systemctl --type=service --state=running | grep -E 'omega|smart'
ss -ltnup | grep -E ':(443|11400|9443)'
```

Keep `omega-boss-sim.service` only when testing the report push locally. The real report destination is configured with `OMEGA_STATS_URL` and `OMEGA_STATS_TOKEN` in the maintenance environment.

## Firefox operator certificate

This requires the operator to choose a password for the Firefox certificate bundle.

On the server PC, run:

```bash
openssl pkcs12 -export \
  -out "$HOME/Desktop/omega-operator.p12" \
  -inkey /home/smart/omega_brick4/pki/operator-key.pem \
  -in /home/smart/omega_brick4/pki/operator-cert.pem \
  -certfile /home/smart/omega_brick4/pki/ca-cert.pem \
  -name omega-operator
```

In Firefox:

1. Open Settings.
2. Search for `certificates`.
3. Open **View Certificates**.
4. Select **Your Certificates**.
5. Click **Import**.
6. Select `Desktop/omega-operator.p12`.
7. Enter the password chosen during `openssl` export.
8. Open `https://smartageing.local` and select the operator certificate if Firefox asks.

Delete the `.p12` file from the Desktop after importing it.

## MCP

There are two consoles and either or both can run:

- On this Ubuntu hub, using Claude Code (terminal). `install_hub.sh` phase 6
  writes the exact `claude mcp add` line into `~/omega_brick4/operator/README_OPERATOR.txt`.
  Claude Code needs a paid Claude plan or API credits.
- On a separate Windows or macOS PC, using the Claude Desktop app, which works
  on the free plan. Setup for that machine is in `MCP_LIVE.md`.

Both present the same `operator` certificate to the same nginx gate. Do not
copy the operator private key into the repository.

The virtual gate already verifies MCP tool registration, stored-data tools,
live AMU tools, mutual certificate authentication, and rogue-operator refusal.
Real use still requires a console to be configured with the operator
certificate paths as above.
