"""Brick 4 MCP server - lets Claude query the fleet directly (Phase 2 goal 5).

A minimal Model Context Protocol server over stdio, run on the OPERATOR's own
machine (not the server - see FINDINGS #22 for why: Windows/Claude Desktop
cannot reliably spawn a wrapper command like ssh.exe). Stored-data tools call
the server's own Flask API over mutual-TLS HTTPS - the SAME nginx mTLS gate
and CA-signed operator identity the dashboard's revoke/restore actions
already use (see server/app.py's module docstring), reached live, every
call - no local database copy, nothing to go stale.

    OMEGA_API_BASE=https://smartageing.local OMEGA_PKI_DIR=/path/pki \
    python3 mcp_server.py

Tools (stored data, via the server's API - live, not a snapshot):
    list_devices()                       -> every reporting device + last seen
    latest_reading(device_id)            -> that device's most recent record
    device_stats(device_id, hours=24)    -> min/max/avg/median/sd per variable
    set_heartbeat(device_type, minutes)  -> push a new heartbeat (nmu|amu)

Tools (LIVE, bridged through the server to an always-on AMU's own DTLS
session - the operator's machine does not need a working wolfssl install,
which has no prebuilt Windows wheel; see FINDINGS #24):
    live_read(device_id)                 -> a fresh reading pulled from the AMU now
    live_status(device_id)               -> the AMU's liveness, pulled now

Tools (QUEUED, for the NMU - a device that never accepts inbound connections):
    nmu_read_now(device_id)              -> ambient dB, asked on the NMU's next contact
    nmu_status(device_id)                -> heap/buffer/uptime, asked on the NMU's next contact
    These do not open a connection to the device - the NMU cannot be called
    the way the AMU can. The question rides out on the NMU's next ACK and the
    answer rides back on its own next transmission (see server/nmu_mailbox.py).
    The server holds the HTTP request open while it waits (up to
    OMEGA_NMU_QUERY_TIMEOUT_S, default 90s) and returns "answered: false" if
    the device has not spoken again within that window - it may still answer
    later; asking again is always safe.

Each tool is a deterministic call - the model only maps the prompt to one of
these; it never improvises the command sent to a device.
"""

import json
import os
import ssl
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from io import TextIOWrapper

import anyio
from mcp.server.fastmcp import FastMCP
from mcp.server.stdio import stdio_server

API_BASE = os.environ.get("OMEGA_API_BASE", "https://smartageing.local").rstrip("/")
PKI_DIR = os.environ.get("OMEGA_PKI_DIR", os.path.join(os.path.dirname(__file__), "pki"))
OPERATOR_CN = os.environ.get("OMEGA_OPERATOR_CN", "operator")
API_TIMEOUT_S = float(os.environ.get("OMEGA_API_TIMEOUT_S", "10"))
LIVE_TIMEOUT_S = float(os.environ.get("OMEGA_LIVE_TIMEOUT_S", "20"))
NMU_QUERY_TIMEOUT_S = float(os.environ.get("OMEGA_NMU_QUERY_TIMEOUT_S", "90"))

mcp = FastMCP("omega-fleet")


def _mtls_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.load_verify_locations(os.path.join(PKI_DIR, "ca-cert.pem"))
    ctx.load_cert_chain(
        os.path.join(PKI_DIR, OPERATOR_CN + "-cert.pem"),
        os.path.join(PKI_DIR, OPERATOR_CN + "-key.pem"))
    return ctx


_SSL_CONTEXT = None


def _api_call(path, method="GET", body=None, timeout_s=API_TIMEOUT_S):
    """One mutual-TLS HTTPS call to the server's Flask API, presenting the
    operator certificate on every request - nginx's ssl_verify_client gate
    (see server/app.py's module docstring) is what actually authorizes it,
    same as every other operator action in this project."""
    global _SSL_CONTEXT
    if _SSL_CONTEXT is None:
        _SSL_CONTEXT = _mtls_context()
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(
        API_BASE + path, data=data, method=method,
        headers={"Content-Type": "application/json"} if data else {})
    with urllib.request.urlopen(req, timeout=timeout_s, context=_SSL_CONTEXT) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _stamp_utc(reply, epoch_key):
    """Adds a human-readable UTC string AND an age next to a raw epoch field.

    Raw epoch seconds are not usable by a reader. Left unconverted they get
    dropped from the answer entirely - observed 2026-08-21, where a fleet
    summary reported readings with no time at all because every stored-data
    tool returned bare integers like 1787282343. The age matters as much as
    the timestamp: "44 seconds old" is what tells someone whether a value is
    current, and it needs no timezone reasoning to be understood."""
    epoch = reply.get(epoch_key)
    if isinstance(epoch, (int, float)):
        reply[epoch_key + "_utc"] = time.strftime("%Y-%m-%d %H:%M:%S UTC",
                                                  time.gmtime(epoch))
        # Local wall-clock too. Without it the reader has to work out the
        # site's offset and whether summer time applies, and a wrong guess
        # produces a confident, plausible, wrong time that nobody catches.
        # This process runs on the operator's own machine, in the same
        # timezone as the fleet, so the OS already knows the answer.
        reply[epoch_key + "_local"] = time.strftime("%Y-%m-%d %H:%M:%S %Z",
                                                    time.localtime(epoch))
        reply[epoch_key + "_age_s"] = round(max(0.0, time.time() - epoch), 1)
    return reply


@mcp.tool()
def list_devices() -> list:
    """List every device that has reported, with type, row count and when it
    was last seen. Each entry carries "last_seen_utc" and "last_seen_age_s" -
    report the age, because a device that last spoke hours ago is a finding,
    not a detail."""
    devices = _api_call("/api/mcp/devices")
    if isinstance(devices, list):
        for device in devices:
            if isinstance(device, dict):
                _stamp_utc(device, "last_seen")
    return devices


@mcp.tool()
def latest_reading(device_id: str) -> dict:
    """Return the most recent STORED reading for a device (e.g. AMU_01,
    NMU_02).

    This is a database read, NOT a live measurement. "timestamp_utc" and
    "timestamp_age_s" say when it was actually recorded, which may be
    seconds or hours ago. ALWAYS state that age when reporting the values -
    an undated reading presented as "current" is wrong even when the numbers
    are right. Use live_read (AMU) or nmu_read_now (NMU) when the user wants
    a genuinely fresh value."""
    reply = _api_call("/api/mcp/latest/" + urllib.parse.quote(device_id, safe=""))
    return _stamp_utc(reply, "timestamp") if isinstance(reply, dict) else reply


@mcp.tool()
def device_stats(device_id: str, hours: int = 24) -> dict:
    """Descriptive statistics (min/max/avg/median/sd) per variable over the
    last N hours, plus the window the numbers actually cover.

    Each variable also reports n_missing / n_malformed / n_implausible -
    counts of values that were excluded and why. State them when they are
    non-zero: an average computed over data with 200 gaps or a 9000 ppm CO2
    spike is a different claim from a clean one, and hiding that is how a
    statistic becomes misleading."""
    reply = _api_call("/api/mcp/stats/%s?hours=%d"
                      % (urllib.parse.quote(device_id, safe=""), hours))
    if isinstance(reply, dict):
        now = int(time.time())
        reply["window_end"] = now
        reply["window_start"] = now - int(hours) * 3600
        _stamp_utc(reply, "window_end")
        _stamp_utc(reply, "window_start")
    return reply


@mcp.tool()
def activity_report(device_id: str, hours: int = 24) -> dict:
    """Explain WHY a device transmitted over the last N hours, and which way
    its readings moved. Use this for open questions about behaviour - "why is
    this unit so chatty", "how did room 4 behave overnight", "is anything
    trending badly" - rather than latest_reading (one value) or device_stats
    (min/max/avg only).

    Returns three things and you should use all three when answering:

      causes - every trigger reason ranked by how many transmissions it
        caused. This is the direct answer to "why did it send so much".
      alarm_episodes - each alarm with when it started, whether it has
        cleared, and how long it has lasted. An alarm marked "ongoing" with a
        long duration_s is the most important thing in the reply and must be
        stated first, in plain language, with the duration in hours.
      trends - per variable: first value, last value, direction, min and max
        over the window.

    Timestamps are epoch seconds; convert them and give the reader clock
    times and durations, never raw numbers. State the window you actually
    covered. If a variable is missing from trends there were too few samples
    to judge a direction - say so rather than inferring one."""
    reply = _api_call("/api/mcp/activity/%s?hours=%d"
                      % (urllib.parse.quote(device_id, safe=""), hours))
    if isinstance(reply, dict):
        _stamp_utc(reply, "window_start")
        _stamp_utc(reply, "window_end")
        for episode in reply.get("alarm_episodes", []):
            if isinstance(episode, dict):
                _stamp_utc(episode, "started")
                if episode.get("cleared"):
                    _stamp_utc(episode, "cleared")
    return reply


@mcp.tool()
def set_heartbeat(device_type: str, minutes: int) -> dict:
    """Push a new heartbeat interval in minutes for a device type ('nmu' or 'amu').
    Delivered to each matching device on its next authenticated ACK."""
    device_type = device_type.lower()
    if device_type not in ("nmu", "amu"):
        return {"error": "device_type must be 'nmu' or 'amu'"}
    return _api_call("/api/config/heartbeat", method="POST",
                     body={device_type + "_hb": minutes})


@mcp.tool()
def live_read(device_id: str) -> dict:
    """Pull a FRESH reading from an always-on AMU right now, over a
    mutual-auth DTLS 1.3 session. Never a database read.

    AND NEVER A DATABASE WRITE. What comes back here is shown to the caller
    and then discarded, deliberately:

      * It is not a measurement the unit CHOSE to report. The AMU samples
        every two seconds and transmits only when a rule fires; a live read
        returns the sample already in hand, which may be one its own logic
        judged not worth sending. Storing it would make the record answer
        "what the fleet reported, plus whatever an operator glanced at",
        and every statistic over that window would quietly include the
        looking.

      * For an NMU it is not even the same QUANTITY. A live read returns
        audioAmbientDb() - the rolling ambient level with no event attached.
        noise_data holds something else: the MEAN chunk-wise dB SPL of a
        detected event, averaged over 125 ms chunks from the first to the
        last one still carrying sound, with the confirmed trailing silence
        removed (omega_audio.cpp, audioCaptureTriggeredEvent). Same column
        name, different physical meaning, different measurement window, and
        nothing downstream would flag the mixture.

    The single write path into the database is session.py's call to
    storage.ingest_telemetry, reached only by a record a device decided to
    send. Keep it that way.

    Two routes, tried in order, and the reply says which one answered in its
    "via" field:
      via="direct" - the server called the device and it answered, ~1-2s.
      via="queued" - the direct call did not answer in time, so the question
                     rode down on the device's next ACK and the answer came
                     back on its next transmission. Slower (seconds to a few
                     minutes) but uses the path proven on the NMU.
    If neither answered, "answered" is false and "queued" is true: the
    question is still waiting and asking again shortly will collect it.

    The reading carries "sampled_at", "age_s" and "stale". The AMU answers
    from the sample it refreshes every 2s rather than sampling on command,
    so age_s is normally 0-2. ALWAYS report that age, and say so explicitly
    if "stale" is true - that means the device's sampling loop has stopped
    and the values, while real, are old."""
    reply = _api_call("/api/mcp/live/" + urllib.parse.quote(device_id, safe=""),
                      method="POST", body={"cmd": "read_now"}, timeout_s=LIVE_TIMEOUT_S)
    return _stamp_utc(reply, "ts")


@mcp.tool()
def live_status(device_id: str) -> dict:
    """Check whether an always-on AMU is reachable and responding right now,
    over a direct DTLS session, bridged the same way as live_read. State
    the "ts"/"ts_utc" capture time in the response."""
    reply = _api_call("/api/mcp/live/" + urllib.parse.quote(device_id, safe=""),
                      method="POST", body={"cmd": "status"}, timeout_s=LIVE_TIMEOUT_S)
    return _stamp_utc(reply, "ts")


def _nmu_ask(device_id: str, cmd: str) -> dict:
    """Ask the server to queue cmd for device_id and wait for it to answer on
    its own next contact - see nmu_read_now/nmu_status docstrings and
    server/nmu_mailbox.py for why this is queue-and-wait, not a direct call
    like the AMU's live tools. The server holds the HTTPS request open for
    the wait; this call's own timeout must exceed the server's."""
    try:
        reply = _api_call("/api/mcp/nmu_ask/" + urllib.parse.quote(device_id, safe=""),
                          method="POST", body={"cmd": cmd},
                          timeout_s=NMU_QUERY_TIMEOUT_S + 5)
    except (urllib.error.URLError, TimeoutError) as error:
        return {"ok": False, "answered": False, "device_id": device_id, "error": str(error)}
    if not reply.get("answered"):
        return {"ok": False, "answered": False, "device_id": device_id,
                "error": ("no contact from %s within %.0fs - it has not "
                         "transmitted since the question was queued; the "
                         "question is still pending and will be asked on its "
                         "next contact, try again shortly"
                         % (device_id, NMU_QUERY_TIMEOUT_S))}
    reply["ok"] = True
    reply["device_id"] = device_id
    return _stamp_utc(reply, "answered_at")


@mcp.tool()
def nmu_read_now(device_id: str) -> dict:
    """Ask a NOISE unit (NMU) for a fresh ambient dB reading. The NMU never
    accepts inbound connections, so this does NOT call it directly like
    live_read does for the AMU - the question is queued and answered on the
    device's own next transmission, which may take a little while. Waits up
    to ~90s; if the device hasn't spoken yet, returns answered=false rather
    than hanging - the question stays queued and can be asked again. The
    reply's "answered_at"/"answered_at_utc" fields are WHEN the device's
    own transmission carrying the answer arrived - always state that time
    (or how many seconds/minutes ago), never present it as instantaneous."""
    return _nmu_ask(device_id, "read_now")


@mcp.tool()
def nmu_status(device_id: str) -> dict:
    """Ask a NOISE unit (NMU) for its liveness (free memory, buffered event
    count, uptime), queued the same way as nmu_read_now - see its docstring,
    including the "answered_at"/"answered_at_utc" timing fields."""
    return _nmu_ask(device_id, "status")


async def _run_stdio_bom_tolerant():
    """Identical to FastMCP.run_stdio_async(), except the stdin TextIOWrapper
    uses utf-8-sig instead of utf-8, so a leading byte-order mark is stripped
    rather than fed to the JSON-RPC parser as a corrupt first message. Some
    Windows MCP clients write one ahead of the first real message - see
    FINDINGS #21. utf-8-sig behaves identically to utf-8 when no BOM is
    present, so this is a no-op for every other client."""
    stdin = anyio.wrap_file(TextIOWrapper(sys.stdin.buffer, encoding="utf-8-sig", errors="replace"))
    async with stdio_server(stdin=stdin) as (read_stream, write_stream):
        await mcp._mcp_server.run(
            read_stream, write_stream, mcp._mcp_server.create_initialization_options())


if __name__ == "__main__":
    anyio.run(_run_stdio_bom_tolerant)
