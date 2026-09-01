"""Phase 0 de-risk proof: real mutual-authenticated DTLS 1.2, on the laptop.

Deterministic and socket-free: it drives two OpenSSL endpoints joined by memory
BIOs (see dtls_loopback) so the result does not depend on Windows UDP loopback.
It proves the three security-relevant facts of the true-DTLS track:

  1. a trusted device (CA-signed AMU_01 cert) completes the handshake, both
     sides negotiate a forward-secret ECDHE-ECDSA cipher, and the SERVER learns
     the identity "AMU_01" from the VERIFIED certificate - not from any payload;
  2. an untrusted client (a well-formed cert from a DIFFERENT, unknown CA -
     i.e. an attacker who has the WLAN password but not a provisioned identity)
     is REJECTED during the handshake;
  3. the negotiated cipher is ECDHE (ephemeral) - forward secrecy.

Exit 0 only if the trusted path succeeds and the untrusted path is refused.
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "provisioning"))

import dtls_loopback
import omega_pki

CIPHERS = "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-ECDSA-AES128-GCM-SHA256"


def _build_pki(out_dir):
    ca_key, ca_cert = omega_pki.create_ca()
    omega_pki._write_key(ca_key, os.path.join(out_dir, "ca-key.pem"))
    omega_pki._write_cert(ca_cert, os.path.join(out_dir, "ca-cert.pem"))
    omega_pki._issue_and_save(out_dir, ca_key, ca_cert, omega_pki.SERVER_COMMON_NAME, True)
    omega_pki._issue_and_save(out_dir, ca_key, ca_cert, "AMU_01", False)
    # attacker: a well-formed cert, but signed by an unknown (rogue) CA
    rogue_ca_key, rogue_ca_cert = omega_pki.create_ca()
    r_key, r_cert = omega_pki.issue_leaf(rogue_ca_key, rogue_ca_cert, "AMU_01", False)
    omega_pki._write_key(r_key, os.path.join(out_dir, "rogue-key.pem"))
    omega_pki._write_cert(r_cert, os.path.join(out_dir, "rogue-cert.pem"))


def main():
    out_dir = tempfile.mkdtemp(prefix="omega_dtls_proof_")
    _build_pki(out_dir)

    trusted = dtls_loopback.handshake(out_dir, "omega-server", "AMU_01", CIPHERS)
    ok_identity = trusted["server_view_of_client"] == "AMU_01"
    ok_verify = trusted["server_verify_ok"] and trusted["client_verify_ok"]
    ok_forward_secret = (trusted["cipher"] or "").startswith("ECDHE-")

    ok_negative = False
    try:
        dtls_loopback.handshake(out_dir, "omega-server", "rogue", CIPHERS)
    except dtls_loopback.HandshakeError:
        ok_negative = True

    print("trusted handshake cipher: %s" % trusted["cipher"])
    print("  server read identity from verified cert: %s" % trusted["server_view_of_client"])
    print("mutual verification ok: %s" % ("PASS" if ok_verify else "FAIL"))
    print("forward secrecy (ephemeral ECDHE): %s" % ("PASS" if ok_forward_secret else "FAIL"))
    print("cert-bound identity: %s" % ("PASS" if ok_identity else "FAIL"))
    print("untrusted client (WLAN access, non-CA cert) refused: %s" %
          ("PASS" if ok_negative else "FAIL"))

    passed = ok_identity and ok_verify and ok_forward_secret and ok_negative
    print("RESULT: %s" % ("PASS - mutual auth, forward secrecy and cert-bound "
                          "identity proven in-memory over DTLS 1.2 (see module "
                          "docstring: 1.3 needs hardware)" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
