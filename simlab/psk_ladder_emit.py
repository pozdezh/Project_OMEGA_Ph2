"""OFF-versus-ON OFF/ON ladder: emit ONE real AMU reading four ways at once.

Runs ON a live AMU. Reads the sampling loop's own live cache (the file
main.py rewrites every cycle) and re-sends that SAME payload over three
unprotected-to-less-unprotected channels, while the production service
carries the identical data over DTLS 1.3 on its real port untouched.

  rung A  plain JSON over UDP        - the pre-security state
  rung B  AES-128-ECB + HMAC-SHA256  - the parallel pre-shared-key project (Rull Ventura) design
  rung C  AES-256-GCM                - her stated future work == Omega brick1
  rung D  DTLS 1.3 mutual-auth PKI   - Omega brick4, NOT emitted here; it is
                                       the production service doing its job

Why the cache and not the sensors: the running service owns the I2C and
serial buses. Re-reading the sensors from a second process would contend
with it and risk disturbing a deployed unit. The cache is written
atomically by that service, so what this script sends is provably the same
reading the DTLS side sent, not a re-measurement and not a mock.

Nothing listens on the rung A/B/C ports. That is deliberate: the claim under
test is what an EAVESDROPPER can read off the wire, and a packet on the wire
is on the wire whether or not anything accepts it. It also means the server
needs no change of any kind for this experiment.

Rung B is a faithful reconstruction of the predecessor's DOCUMENTED design (per-device
pre-shared key, AES-128-ECB, HMAC-SHA256 over the ciphertext so the MAC is
checked before decrypting). It is NOT her source code, which was not
available. Rung C imports brick1's real frozen omega_crypto.py unmodified.

The two keys below are throwaway experiment constants with no production
role whatsoever - no deployed device or server has ever held them. They are
in git on purpose so the run is reproducible.
"""
import json
import os
import socket
import sys
import time

from cryptography.hazmat.primitives import hashes, hmac, padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import omega_crypto

LIVE_CACHE = os.environ.get("OMEGA_LIVE_CACHE", "/dev/shm/omega_amu_latest.json")
SERVER = os.environ.get("LADDER_SERVER", "192.168.0.112")

PORT_CLEARTEXT = 11501
PORT_ECB_HMAC = 11502
PORT_GCM = 11503

RECORD_COUNT = int(os.environ.get("LADDER_RECORDS", "20"))
RECORD_INTERVAL_S = float(os.environ.get("LADDER_INTERVAL_S", "3.0"))

AES_BLOCK_BITS = 128
ECB_KEY_LEN_BYTES = 16
HMAC_TAG_LEN_BYTES = 32

EXPERIMENT_PSK_ECB = bytes.fromhex("a1b2c3d4e5f60718293a4b5c6d7e8f90")
EXPERIMENT_MASTER_GCM = bytes.fromhex(
    "0f1e2d3c4b5a69788796a5b4c3d2e1f00f1e2d3c4b5a69788796a5b4c3d2e1f0")

DIRECTION_TO_SERVER = omega_crypto.DIR_DEVICE_TO_SERVER


def read_live_reading(path):
    """Return the sampling loop's newest published reading, or None."""
    try:
        with open(path, "r") as handle:
            cached = json.load(handle)
    except (IOError, OSError, ValueError):
        return None
    if not isinstance(cached, dict) or "reading" not in cached:
        return None
    return cached["reading"]


def build_payload(device_id, reading, event, cause):
    """Build the record in the EXACT key order main.py uses, so the bytes
    this experiment measures are the bytes the real sender would produce."""
    return {"id": device_id, "type": "airq", "ts": int(time.time()),
            "hb": False, "cause": cause, "sensors": reading, "event": event}


def seal_ecb_hmac(key, plaintext):
    """the predecessor's construction: AES-128-ECB over PKCS7-padded plaintext, then
    HMAC-SHA256 appended over the ciphertext. Encrypt-then-MAC, so a
    receiver checks the cheap 32-byte tag before decrypting anything."""
    padder = padding.PKCS7(AES_BLOCK_BITS).padder()
    padded = padder.update(plaintext) + padder.finalize()
    encryptor = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
    ciphertext = encryptor.update(padded) + encryptor.finalize()
    tag = hmac.HMAC(key, hashes.SHA256())
    tag.update(ciphertext)
    return ciphertext + tag.finalize()


def seal_gcm(master_key, device_id, session_id, counter, payload, plaintext):
    """Brick1's real construction, via brick1/crypto/omega_crypto.py."""
    device_key = omega_crypto.derive_device_key(master_key, device_id)
    nonce = omega_crypto.build_nonce(session_id, counter, DIRECTION_TO_SERVER)
    aad = omega_crypto.build_telemetry_aad(
        device_id, payload["type"], payload["ts"], payload["event"], 0)
    return nonce + omega_crypto.encrypt(device_key, nonce, aad, plaintext)


def emit(sock, blob, port):
    sock.sendto(blob, (SERVER, port))
    return len(blob)


def main():
    device_id = os.environ.get("LADDER_DEVICE_ID")
    if not device_id:
        print("set LADDER_DEVICE_ID to this unit's real id", file=sys.stderr)
        return 2

    reading = read_live_reading(LIVE_CACHE)
    if reading is None:
        print("no live sample at %s - is the AMU service running?" % LIVE_CACHE,
              file=sys.stderr)
        return 3

    session_id = int(time.time()) & 0xFFFFFFFF
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Rull Ventura ladder: %s -> %s" % (device_id, SERVER))
    print("live cache    : %s" % LIVE_CACHE)
    print("rung A port   : %d  cleartext JSON" % PORT_CLEARTEXT)
    print("rung B port   : %d  AES-128-ECB + HMAC-SHA256 (predecessor)" % PORT_ECB_HMAC)
    print("rung C port   : %d  AES-256-GCM (brick1)" % PORT_GCM)
    print("rung D        : the production DTLS 1.3 service, untouched")
    print("records       : %d every %.1fs" % (RECORD_COUNT, RECORD_INTERVAL_S))
    print("")

    for counter in range(1, RECORD_COUNT + 1):
        reading = read_live_reading(LIVE_CACHE) or reading
        event = "%d_%d" % (session_id, counter)
        payload = build_payload(device_id, reading, event, "LADDER")
        plaintext = json.dumps(payload).encode("utf-8")

        size_a = emit(sock, plaintext, PORT_CLEARTEXT)
        size_b = emit(sock, seal_ecb_hmac(EXPERIMENT_PSK_ECB, plaintext),
                      PORT_ECB_HMAC)
        size_c = emit(sock, seal_gcm(EXPERIMENT_MASTER_GCM, device_id,
                                     session_id, counter, payload, plaintext),
                      PORT_GCM)

        print("record %2d  event=%s  A=%dB  B=%dB  C=%dB"
              % (counter, event, size_a, size_b, size_c))
        if counter < RECORD_COUNT:
            time.sleep(RECORD_INTERVAL_S)

    sock.close()
    print("")
    print("done - %d records per rung" % RECORD_COUNT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
