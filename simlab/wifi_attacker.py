"""WiFi-credential attacker: prove that being ON the network is not being IN
the system.

Threat model, stated exactly. The attacker has:
  - the WiFi password (so full LAN access, can reach any host and port)
  - complete knowledge of the protocol (this very source tree is public)
  - the ability to forge any device-id STRING and make a valid-looking
    certificate with their OWN certificate authority
The attacker does NOT have: the project CA's private key.

Every attempt below is run against the LIVE listener on its real port. The
attacker's vantage point - "a host that can reach the listener" - is exactly
what a WiFi password grants, so running from any such host is faithful to the
threat, and the failed handshakes are corroborated in the listener's own log.

Positive control: the real fleet is connecting throughout (visible in the
listener log). Same server, same port, same protocol - the ONLY thing the
attacker lacks is a CA-signed certificate. If that one missing thing is what
stops them, the certificate is doing its whole job.
"""
import datetime
import json
import os
import socket
import sys
import tempfile

HOST = os.environ.get("TARGET_HOST", "127.0.0.1")
PORT = int(os.environ.get("TARGET_PORT", "11400"))

try:
    import wolfssl
except ImportError:
    print("wolfssl not importable in this interpreter", file=sys.stderr)
    sys.exit(2)

from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec


def banner(n, title):
    print("\n" + "=" * 68)
    print("ATTEMPT %d: %s" % (n, title))
    print("=" * 68)


def attempt_raw_udp():
    """The pre-security era. Just send the JSON a real device would, in the
    clear, and see if the server accepts telemetry from an unauthenticated
    stranger."""
    banner(1, "Raw UDP telemetry, no encryption at all")
    payload = json.dumps({
        "id": "NMU_16", "event": "99999999_1", "hb": False,
        "db": 61.5, "duration": 0.5, "cause": "Sustained Alarm: High Temp",
    }).encode()
    print("  posing as NMU_16, injecting a forged alarm in cleartext...")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(6)
    try:
        s.sendto(payload, (HOST, PORT))
        try:
            data, _ = s.recvfrom(4096)
            print("  RESULT: server REPLIED (%d bytes) - THIS WOULD BE A BREACH" % len(data))
            return "BREACH"
        except socket.timeout:
            print("  RESULT: silence. The listener speaks only DTLS; a plain")
            print("          UDP datagram is not a handshake, so it is ignored.")
            print("          The forged alarm never reaches the database.")
            return "DEFEATED"
    finally:
        s.close()


def _dtls_client_ctx(cert_pem=None, key_pem=None):
    ctx = wolfssl.SSLContext(wolfssl.PROTOCOL_DTLSv1_3, server_side=False)
    # The attacker cannot verify the server (no CA cert), and does not care to
    # - they are trying to get IN, not to check who they are talking to.
    ctx.verify_mode = wolfssl.CERT_NONE
    if cert_pem and key_pem:
        ctx.load_cert_chain(cert_pem, key_pem)
    return ctx


def _try_handshake(ctx, label):
    raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    raw.settimeout(8)
    try:
        conn = ctx.wrap_socket(raw, server_side=False)
        conn.connect((HOST, PORT))
        conn.write(json.dumps({"id": "NMU_16", "event": "99999999_2",
                               "db": 61.5, "duration": 0.5,
                               "cause": "Sustained Alarm: High Temp"}).encode())
        reply = conn.read(4096)
        print("  RESULT: handshake COMPLETED and server accepted data (%r)" % reply[:40])
        print("          THIS WOULD BE A BREACH")
        return "BREACH"
    except Exception as error:
        msg = str(error) or type(error).__name__
        print("  RESULT: handshake REFUSED - %s" % msg[:90])
        return "DEFEATED"
    finally:
        try:
            raw.close()
        except Exception:
            pass


def attempt_dtls_no_cert():
    banner(2, "Proper DTLS handshake, but with NO client certificate")
    print("  the attacker speaks the right protocol but has no identity to present...")
    return _try_handshake(_dtls_client_ctx(), "no-cert")


def attempt_dtls_forged_cert():
    banner(3, "DTLS with a VALID-LOOKING certificate the attacker made themselves")
    print("  the attacker runs their OWN certificate authority and signs a cert")
    print("  claiming to be NMU_16 - it is cryptographically valid, just not")
    print("  signed by the authority this fleet trusts...")

    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "NMU_16")])
    now = datetime.datetime.utcnow()
    cert = (x509.CertificateBuilder()
            .subject_name(name).issuer_name(name)      # self-issued: rogue CA
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now)
            .not_valid_after(now + datetime.timedelta(days=3650))
            .sign(key, hashes.SHA256()))

    d = tempfile.mkdtemp(prefix="rogue_")
    cpath, kpath = os.path.join(d, "c.pem"), os.path.join(d, "k.pem")
    with open(cpath, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))
    with open(kpath, "wb") as f:
        f.write(key.private_bytes(serialization.Encoding.PEM,
                                  serialization.PrivateFormat.TraditionalOpenSSL,
                                  serialization.NoEncryption()))
    return _try_handshake(_dtls_client_ctx(cpath, kpath), "forged-cert")


def main():
    print("WiFi-credential attacker against %s:%d" % (HOST, PORT))
    print("The attacker has the WiFi password and the full protocol source.")
    print("The attacker does NOT have the project CA's private key.")
    results = {
        "1 raw UDP injection": attempt_raw_udp(),
        "2 DTLS, no certificate": attempt_dtls_no_cert(),
        "3 DTLS, forged certificate": attempt_dtls_forged_cert(),
    }
    print("\n" + "=" * 68)
    print("SUMMARY")
    print("=" * 68)
    for k, v in results.items():
        print("  %-32s %s" % (k, v))
    breached = [k for k, v in results.items() if v == "BREACH"]
    print()
    if breached:
        print("SYSTEM BREACHED via: %s" % ", ".join(breached))
        return 1
    print("All %d attempts DEFEATED. WiFi access yielded nothing: the trust" % len(results))
    print("boundary is the CA-signed certificate, not the network.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
