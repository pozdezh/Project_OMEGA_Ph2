"""Brick 4 operator-side live client - MCP tool -> AMU, directly.

Migrated to wolfSSL DTLS 1.3, mirroring amu/dtls_client.py's proven
client-side pattern - see FINDINGS #11. The MCP server calls live_query()
to open a one-shot mutual-authenticated DTLS session straight to an
always-on AMU's live command port and exchange one deterministic
command/reply. It presents the operator certificate and verifies the AMU
against the CA, so it never talks to an impostor and no device can be
impersonated. This bypasses both the Flask REST API and the store-and-
forward server path entirely - the operator speaks to the device itself.
"""

import json
import os
import select
import socket
import struct

LIVE_PORT = 5001
RECV_BYTES = 8192
DEFAULT_TIMEOUT_S = 6.0


def _set_recv_timeout(sock, seconds):
    """Kernel-level timeout, honoured during the handshake only - see
    amu/dtls_client.py's identical note on why not socket.settimeout()."""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO,
                    struct.pack("ll", int(seconds), 0))


def live_query(pki_dir, operator_cn, host, port, command, timeout_s=DEFAULT_TIMEOUT_S):
    """Send one command dict to an AMU's live server and return its reply
    dict. Raises on connection/handshake/timeout failure so the MCP tool can
    report it rather than hang.

    wolfssl is imported here, not at module level, so importing this module
    (and therefore mcp_server, which registers this as a tool) never depends
    on wolfssl being installed on whatever machine is merely loading the
    tool registry - only actually CALLING this function does.
    """
    import wolfssl
    raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _set_recv_timeout(raw, timeout_s)
    raw.connect((host, port))
    conn = None
    try:
        ctx = wolfssl.SSLContext(wolfssl.PROTOCOL_DTLSv1_3, server_side=False)
        ctx.load_cert_chain(
            os.path.join(pki_dir, operator_cn + "-cert.pem"),
            os.path.join(pki_dir, operator_cn + "-key.pem"),
        )
        ctx.load_verify_locations(os.path.join(pki_dir, "ca-cert.pem"))
        ctx.verify_mode = wolfssl.CERT_REQUIRED
        conn = ctx.wrap_socket(raw, server_side=False)
        conn.send(json.dumps(command).encode("utf-8"))
        # select(), not conn.recv() alone: wolfSSL rewrites SO_RCVTIMEO to
        # "block forever" once the handshake completes (same issue documented
        # in amu/dtls_client.py) - select() keeps its own clock and is immune.
        ready, _, _ = select.select([raw], [], [], timeout_s)
        if not ready:
            raise TimeoutError(
                "no reply from %s:%d within %.1fs" % (host, port, timeout_s))
        data = conn.recv(RECV_BYTES)
        return json.loads(data.decode("utf-8"))
    finally:
        for closeable in (conn, raw):
            try:
                if closeable is not None:
                    closeable.close()
            except Exception:
                pass
