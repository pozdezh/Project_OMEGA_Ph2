import errno
import os
import random
import subprocess
import time

import amu_config
import clock
import recovery
import dtls_client
import server_discovery

_last_cfg_ver = None
_consecutive_ack_fails = 0

_placeholder = amu_config.STATIC_SERVER or ("0.0.0.0", amu_config.UDP_PORT)
_discovery_done = False

client = dtls_client.DtlsClient(amu_config.PKI_DIR, amu_config.DEVICE_ID,
                                _placeholder[0], _placeholder[1])


rejected_servers = 0

# Has a handshake EVER succeeded against the address the client currently
# holds? Until it has, that address is a guess and must not be preferred over
# discovery.
#
# This existed as a bug until 2026-08-26. The client is constructed with the
# configured server_host, and connect() tried that address before calling
# _try_candidates() - so on a network where the configured address happened to
# be right, discovery NEVER RAN. The unit worked, and the discovery path it
# was supposed to rely on was dead code in production. It surfaced only when a
# packet capture of an AMU restart showed a handshake with no mDNS query in
# front of it, while the NMU's showed the full sequence.
#
# find_servers() already ranks correctly - cache, mDNS, broadcast, then the
# configured address LAST, exactly as ARCHITECTURE.md section 13 describes.
# The fault was never the ranking; it was reaching a configured address
# without consulting the ranking at all.
_server_established = False


# Errno values that mean "the packet never got anywhere": no route, host
# down, nothing listening, no answer in time. None of these say anything at
# all about the identity of whatever is (not) at the far end.
#
# The Linux numbers are listed EXPLICITLY alongside the symbols, because the
# symbols resolve to different values per platform - ENETUNREACH is 101 on the
# Pi this runs on and 10051 on the Windows machine the gate runs on. Relying on
# the symbols alone would make this rule untestable off the device, which is
# the same trap the NMU's millis() arithmetic fell into on 2026-08-27.
_UNREACHABLE_ERRNOS = frozenset((
    errno.ENETUNREACH, errno.EHOSTUNREACH, errno.ENETDOWN,
    errno.ECONNREFUSED, errno.ETIMEDOUT, errno.EHOSTDOWN,
    101, 113, 100, 111, 110, 112))


def _failure_was_unreachable():
    """Did the last connect fail because the network was dead, rather than
    because the far end failed to prove who it was?

    Checked by errno rather than by exception type on purpose: wolfSSL's
    SSLError inherits from OSError, so an isinstance test would quietly
    classify a genuine authentication failure as a dead link - the exact
    confusion this function exists to end, in the more dangerous direction.
    """
    return getattr(client.last_error, "errno", None) in _UNREACHABLE_ERRNOS


def _try_candidates():
    """Try every discovered candidate until one AUTHENTICATES.

    See ARCHITECTURE.md section 13. Discovery is untrusted; the handshake is
    what selects the server, and only a completed handshake writes the cache.
    """
    global rejected_servers
    candidates = server_discovery.find_servers(
        static_fallback=amu_config.STATIC_SERVER, log=print)
    global _server_established
    for host, port in candidates:
        client.set_server(host, port)
        if client.connect():
            server_discovery.confirm_server(host, port)
            _server_established = True
            return True
        if _failure_was_unreachable():
            # Nothing answered. Saying "failed mutual auth" here would be a
            # lie with consequences: it reads as an impostor on the network
            # and sends whoever is debugging after a security incident that
            # never happened. It is a dead link. Say so, and do NOT count it
            # against the rejection tally, which is a security signal.
            print("discovery: %s:%d could not be reached (%s) - not an "
                  "authentication failure" % (host, port, client.last_error))
            continue
        rejected_servers += 1
        print("discovery: %s:%d answered but failed mutual auth - rejected "
              "(total rejected: %d)" % (host, port, rejected_servers))
    server_discovery.forget_cached()
    return False


# nmcli MUST be bounded. Both calls below ran with no timeout until
# 2026-08-25, and on that day two units went silent for 45 minutes while
# otherwise completely alive - sampling every two seconds, answering live
# queries - because the network worker had entered hard_network_reset() and
# nmcli never returned.
#
# The damage was not the stall itself but where it happened: the recovery
# ladder is advanced from that same thread. A blocked worker therefore takes
# the ladder down with it, so the unit could not even reboot itself out of the
# fault - the one guarantee this design rests on. A hung external command must
# cost one cycle, never the thread.
WIFI_CHECK_TIMEOUT_S = 15.0
WIFI_CONNECT_TIMEOUT_S = 45.0


def connect_wifi(ssid, password):
    """Bring the WLAN up, preferring the saved profile over a fresh association.

    `nmcli dev wifi connect <ssid> password <pass>` FAILS on every unit in
    this fleet and always has. The WiFi profile is owned by netplan, not by
    NetworkManager's own store, so nmcli will not build a new one over it:
    "802-11-wireless-security.key-mgmt: property is missing", exit 1. Proven
    on AMU_15 on 2026-08-26 by running both commands back to back - that one
    exit 1, and `nmcli connection up netplan-wlan0-<ssid>` exit 0.

    The second defect was worse than the first: the old code discarded the
    exit code and returned True unconditionally, with capture_output
    swallowing the error text. Recovery rung 1 therefore reported success
    while doing nothing at all, and every unit that ever needed it silently
    climbed to the reboot rung instead.
    """
    print("Verifying WLAN connection to: %s..." % ssid)
    try:
        # LC_ALL=C: nmcli's "yes"/"no" column is localized (e.g. "si" under a
        # Spanish locale), so an unforced locale silently breaks this check
        # and every boot pays for a needless reconnect.
        env = dict(os.environ, LC_ALL="C")
        check = subprocess.run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
                               capture_output=True, text=True, env=env,
                               timeout=WIFI_CHECK_TIMEOUT_S)
        if any(line.startswith("yes:" + ssid) for line in check.stdout.split("\n")):
            print("Already connected to %s." % ssid)
            return True

        listing = subprocess.run(["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show"],
                                 capture_output=True, text=True, env=env,
                                 timeout=WIFI_CHECK_TIMEOUT_S)
        profile = None
        for line in listing.stdout.split("\n"):
            name, _, kind = line.rpartition(":")
            if kind != "802-11-wireless" or not name:
                continue
            # Match on the profile's real SSID, never on its name. netplan
            # calls it netplan-wlan0-<ssid> and nmcli calls it <ssid>; both
            # conventions are guesses, the SSID field is a fact.
            got = subprocess.run(["nmcli", "-t", "-f", "802-11-wireless.ssid",
                                  "connection", "show", name],
                                 capture_output=True, text=True, env=env,
                                 timeout=WIFI_CHECK_TIMEOUT_S)
            if got.stdout.strip().rpartition(":")[2] == ssid:
                profile = name
                break

        if profile:
            up = subprocess.run(["sudo", "-n", "nmcli", "connection", "up", profile],
                                capture_output=True, text=True, env=env,
                                timeout=WIFI_CONNECT_TIMEOUT_S)
            if up.returncode == 0:
                print("WLAN reconnected via saved profile %s." % profile, flush=True)
                return True
            print("WLAN profile %s failed to activate (exit %d): %s"
                  % (profile, up.returncode, up.stderr.strip()), flush=True)
            return False

        # No saved profile: a first-ever association on a card that has never
        # seen this network. This is the only case the old command was ever
        # correct for.
        joined = subprocess.run(["sudo", "-n", "nmcli", "dev", "wifi", "connect",
                                 ssid, "password", password],
                                capture_output=True, text=True, env=env,
                                timeout=WIFI_CONNECT_TIMEOUT_S)
        if joined.returncode == 0:
            print("WLAN joined %s for the first time." % ssid, flush=True)
            return True
        print("WLAN join of %s failed (exit %d): %s"
              % (ssid, joined.returncode, joined.stderr.strip()), flush=True)
        return False
    except subprocess.TimeoutExpired:
        # Say plainly which failure this was. "nmcli hung" and "no such
        # network" need different responses from whoever reads the log.
        print("WiFi reconnect timed out - nmcli did not return. Continuing; "
              "the recovery ladder escalates from here if the link stays down.",
              flush=True)
        return False
    except Exception as error:
        print("Network check bypassed or failed: %s" % error)
        return False


def connect_wifi_initial():
    connect_wifi(amu_config.WIFI_SSID, amu_config.WIFI_PASS)


def hard_network_reset():
    print("Performing soft-network reset...")
    connect_wifi(amu_config.WIFI_SSID, amu_config.WIFI_PASS)


def prepare_crypto():
    """Build the DTLS context on the main thread before any worker starts.
    See DtlsClient.prepare - wolfSSL only initialises on the main thread."""
    return client.prepare()


def ensure_session():
    """Make sure a live DTLS session exists, rediscovering the server if the
    old address stopped answering.

    This is what makes startup order irrelevant: a device booted before the
    server keeps failing here harmlessly, buffering as it goes, and connects
    by itself the moment the server appears. A server that moves to a new
    DHCP address is re-found without touching the device.
    """
    if client.connected():
        return True
    time.sleep(random.uniform(0.0, amu_config.RECONNECT_JITTER_S))
    # Retry the current address ONLY if a handshake has already succeeded
    # against it. A reconnect to a known-good server should not pay for
    # discovery; a first connection must not skip it.
    if _server_established and client.connect():
        return True
    return _try_candidates()


def _apply_config(ack):
    global _last_cfg_ver
    if "t" in ack:
        clock.apply_server_time(ack["t"])
    if "hb" in ack:
        amu_config.apply_learned_heartbeat_s(int(ack["hb"]) * 60)
    if "idle" in ack:
        # The server publishes exactly how long it will tolerate silence, so
        # re-handshake before it hangs up rather than after. Acting at 80% of
        # its limit leaves margin for clock skew and a slow round trip.
        client.set_stale_after(int(ack["idle"]) * 0.8)
    if "cfg_ver" in ack:
        _last_cfg_ver = ack["cfg_ver"]


def _to_record(payload):
    """The plain-JSON record sent inside the DTLS channel. The server forces
    identity from the certificate, so id here is advisory only."""
    # ts is resolved HERE, at the moment of sending, not at capture. A record
    # captured before the clock could be trusted carries ts = 0 plus a marker
    # saying how long ago it happened; by the time it actually goes out the
    # clock is usually known, and the reading can be dated when it was TAKEN
    # rather than when it arrived. Both the live path and the backlog drain
    # pass through here, so both are covered. The marker itself is dropped by
    # this whitelist and never reaches the wire.
    record = {"id": amu_config.DEVICE_ID, "type": "airq",
              "ts": clock.resolve_stamp(payload["ts"], payload.get("_cap")),
              "event": payload["event"], "hb": payload.get("hb", False),
              "sensors": payload.get("sensors", {}), "cause": payload.get("cause", "")}
    if _last_cfg_ver is not None:
        record["cfg_ver"] = _last_cfg_ver
    # The answer to an operator question, when this record is carrying one
    # (server/session.py reads "qr"). This whitelist drops any key it does
    # not name, which silently swallowed the answer until 2026-08-20 - the
    # device logged that it had answered while the server never saw a reply.
    if payload.get("qr") is not None:
        record["qr"] = payload["qr"]
    return record


def deliver(payload):
    """Ensure a session and send one record, returning True on a verified ACK."""
    global _consecutive_ack_fails
    if not ensure_session():
        return False
    ack = client.send_and_ack(_to_record(payload))
    if ack is None:
        return False
    _apply_config(ack)
    _consecutive_ack_fails = 0
    recovery.note_contact()
    return True


def take_question():
    """Hand over an operator question delivered on a recent ACK, or None.
    No socket I/O - see dtls_client.take_question."""
    return client.take_question()


def mark_ack_failure():
    global _consecutive_ack_fails
    _consecutive_ack_fails += 1
    print("ACK failure streak: %d/%d" % (_consecutive_ack_fails, amu_config.MAX_MISSED_ACKS))
    client.close()
    if _consecutive_ack_fails >= amu_config.MAX_MISSED_ACKS:
        _consecutive_ack_fails = 0
        hard_network_reset()
        time.sleep(amu_config.RECONNECT_BACKOFF_S)


def close():
    client.close()
