# Project Omega - MCP fleet access

The "talk to the fleet in plain language" layer. You point a Claude client at
the Omega MCP server and type informal requests. Claude maps each request to
ONE deterministic tool call; the tool does the real work. The model never
invents the command sent to a device - it only picks which fixed tool to call.

## Two consoles, both optional, both able to run at once

The same `mcp_server.py`, the same `operator` certificate, and the same nginx
gate serve two independent clients:

| Console | Runs on | Client | Claude plan needed |
|---|---|---|---|
| Ubuntu hub | the server PC itself | Claude Code (terminal) | paid plan (Pro/Max/Team/Enterprise) or API credits - no free tier |
| Desktop | a separate Windows or macOS PC | Claude Desktop app | works on the **free** plan (a locally configured MCP server is not a paid feature) |

Neither requires the other. Running both at the same time is fine - they are
two clients of one server, not two copies of a service.

> **Rewritten 2026-08-21.** The previous version of this file described an
> architecture that no longer exists: it said stored-data tools read the
> database directly and that live tools opened DTLS from the laptop to the
> device. Neither is true today, and the reasons for each change are recorded
> below. It also documented six tools; there are eight.

## Where each piece actually runs

```
  Claude Desktop  (operator PC)
        |  stdio JSON-RPC
  mcp_server.py   (operator PC)  - 8 tool definitions, holds the operator cert
        |  HTTPS + client certificate  (mutual TLS)
  nginx           (server)       - ssl_verify_client: no cert, no entry
        |
  app.py /api/mcp/*  (server)    - all the real logic lives here
        |                    \
  SQLite (stored)        DTLS 1.3 to the device (live)
```

**The operator PC holds almost no logic.** Each of the eight tools is a
two-line wrapper around one HTTPS call. Everything that touches data or
devices runs on the server. That matters for the security story: the
enforcement point is the server, not the laptop.

## The eight tools

**Stored data** - a database read on the server, no device contacted:

| tool | returns |
|---|---|
| `list_devices()` | every device that has reported, type, row count, last seen |
| `latest_reading(device_id)` | that device's most recent stored record |
| `device_stats(device_id, hours)` | min/max/avg/median/sd per variable, plus n_missing / n_malformed / n_implausible |
| `set_heartbeat(device_type, minutes)` | pushes a new heartbeat, delivered on each device's next ACK |

**Live, for an always-on AMU** - the server opens a fresh mutual-auth DTLS 1.3
session to the device:

| tool | returns |
|---|---|
| `live_read(device_id)` | a fresh reading pulled from the AMU now (~1 s) |
| `live_status(device_id)` | whether the AMU is reachable and answering now |

**Queued, for the NMU** - a device that never accepts inbound connections:

| tool | returns |
|---|---|
| `nmu_read_now(device_id)` | ambient dB, asked on the NMU's next contact |
| `nmu_status(device_id)` | free heap, buffered count, uptime |

## Why the live path goes through the server, not from the laptop

The original design had `mcp_server.py` open DTLS straight to the AMU. It was
abandoned for a concrete reason: **wolfSSL has no prebuilt Windows wheel**
(FINDINGS #24), so the operator machine cannot speak DTLS at all without a
from-source build most operators will not manage.

Bridging through the server is also better security, not just easier: the
operator credential can now only ever talk to the server, so a stolen laptop
cannot reach a device directly. The AMU enforces this from its own side too -
`LIVE_ALLOWED_CALLERS` accepts only the SERVER's certificate, not the
operator's, even though both are valid under the same CA.

## Two routes to a device, and the reply says which one answered

`live_read` returns a `via` field:

- `via: "direct"` - the server called the AMU and it answered. ~1 s measured.
- `via: "queued"` - the direct call did not answer in time, so the question
  was left in the mailbox, rode down on the device's next ACK, and the answer
  came back on its next transmission. Seconds to minutes.

If neither answered, `answered` is false and `queued` is true: the question is
still waiting and asking again shortly collects it.

The NMU tools are always the queued route - it has no listening port by
design. Measured round trip on a live unit: **28 s**.

## Setup A - a Windows or macOS PC with Claude Desktop (free plan is fine)

Copy from the server (from `~/omega_brick4/operator/`, or wherever
`omega_pki.py init` wrote them) to the PC:

- `ca-cert.pem`, `operator-cert.pem`, `operator-key.pem`
- `mcp_server.py` (from `deploy/server/payload/` or `brick4_dtls13/server/`)

Then:

1. Put the three PEM files in a folder, e.g. `C:\omega\pki\`, and
   `mcp_server.py` anywhere, e.g. `C:\omega\`. Nothing else is needed - no
   database copy, no device config, no payload folder. Delete the PEM copies
   from the USB stick or Downloads folder once they are in place.
2. Install the one Python dependency: `py -3.12 -m pip install mcp anyio`
   (the live tools go through the server, so `wolfssl` is not needed here).
3. Edit `claude_desktop_config.json` (Settings -> Developer -> Edit Config):

```json
{
  "mcpServers": {
    "omega-fleet": {
      "command": "py",
      "args": ["-3.12", "C:\\path\\to\\server\\mcp_server.py"],
      "env": {
        "OMEGA_API_BASE": "https://smartageing.local",
        "OMEGA_PKI_DIR": "C:\\omega\\pki",
        "OMEGA_OPERATOR_CN": "operator"
      }
    }
  }
}
```

Those three variables are the complete set the tool actually reads. Anything
else (`OMEGA_DB`, `OMEGA_DEVICE_CONFIG`, `OMEGA_LIVE_PORT`) is a leftover from
the old design and is ignored.

4. Restart Claude Desktop fully - quit from the tray, not just close the
   window, or the old tool definitions stay loaded.

The operator machine must be able to resolve `smartageing.local` and reach the
server over HTTPS. It does NOT need to be on the same WLAN as the devices any
more, because it never contacts them directly.

## Setup B - the Ubuntu hub itself, with Claude Code

Claude Desktop is macOS and Windows only, so on the server PC the console is
Claude Code (the terminal tool). `install_hub.sh` phase 6 already places the
operator files in `~/omega_brick4/operator/` and writes this into
`README_OPERATOR.txt`:

```
curl -fsSL https://claude.ai/install.sh | bash
claude mcp add omega -- ~/omega_brick4/venv/bin/python ~/omega_brick4/mcp_server.py
claude
```

The environment file the installer writes already sets `OMEGA_PKI_DIR`; set
`OMEGA_API_BASE=https://smartageing.local` and `OMEGA_OPERATOR_CN=operator`
too if they are not already exported. Claude Code needs a paid Claude plan or
API credits; this is the only part of the project that does.

Both setups can be live at the same time. They present the same operator
certificate to the same gate.

## Where the AMU's address comes from

The server resolves each AMU by **mDNS hostname** (e.g. `amu1.local`), listed
under `"endpoints"` in the server's `device_config.json`.

Never a raw IP: `device_live.live_query()` passes the string straight to
`socket.connect()`, which resolves it fresh on EVERY call, so a DHCP lease
change never breaks it. Proven live 2026-08-20 with `getent hosts`,
`avahi-resolve`, and Python's own `getaddrinfo()`. Requires `nss-mdns` on the
server (already configured: `/etc/nsswitch.conf`'s `hosts:` line includes
`mdns4_minimal`). One-time entry per new AMU, not per-lease maintenance.

## Example prompts

| You type | Claude calls |
|---|---|
| "which units are online?" | `list_devices()` |
| "what's the CO2 in AMU_01 right now?" | `live_read("AMU_01")` |
| "is room 2's air unit responding?" | `live_status("AMU_02")` |
| "how loud is it at NMU_02 right now?" | `nmu_read_now("NMU_02")` |
| "is NMU_02 healthy?" | `nmu_status("NMU_02")` |
| "give me today's stats for AMU_01" | `device_stats("AMU_01", 24)` |
| "set the noise units to report every 5 minutes" | `set_heartbeat("nmu", 5)` |

## Why this is a clean result for the paper

- **Determinism.** The model's only job is prompt -> one typed tool call with
  structured arguments. Execution is a fixed command vocabulary
  (`read_now` / `status`), auditable and reproducible. An out-of-vocabulary
  instruction is refused, not improvised.
- **One enforcement point.** Every tool crosses the same nginx mTLS gate.
  Verified by measurement 2026-08-21: a request presenting no client
  certificate is refused with **HTTP 400**, before it reaches any application
  code.
- **Same trust model everywhere.** The operator is just another CA-signed
  identity. The live channel reuses the exact mutual-auth DTLS the fleet
  already uses, so the AI layer adds convenience without weakening the
  security model - and the device-side whitelist means the operator identity
  cannot be escalated into direct device access.
- **Honest about reachability.** The two device types answer by different
  routes and the reply says which, with a timestamp and an age. A reading is
  never presented as current when it is not.

Grounding / prior art worth citing: IoT-MCP (arXiv 2510.01260), the safety-first
Device Context Protocol (arXiv 2605.26159), and MCP-over-MQTT on ESP32 (EMQX).
