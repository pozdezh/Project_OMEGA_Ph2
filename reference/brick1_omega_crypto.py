"""Project Omega Brick 1 crypto primitives.

AES-256-GCM per-packet encryption and per-device authentication for the UDP
link. Pure primitives only: no I/O, no transport, no JSON. Byte layout and
constants follow SPEC.md (the single source of truth) and are pinned by
test_vector.json. Shared by the server (listener) and the AMU sender; the NMU
C++ module mirrors these function shapes.
"""

import struct

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.kdf.hkdf import HKDF

# SPEC Section 2 constants (no magic numbers anywhere else).
SALT = b"omega-v1"
KEY_LEN_BYTES = 32
NONCE_LEN_BYTES = 12
TAG_LEN_BYTES = 16
DIR_DEVICE_TO_SERVER = 0x00
DIR_SERVER_TO_DEVICE = 0x01
AAD_FIELD_SEPARATOR = "|"
RESERVED_LEN_BYTES = 3
RESERVED_FILL = b"\x00"
AAD_TYPE_ACK = "ack"
ACK_HEARTBEAT_FIELD = "0"

# Field-width bounds used to reject out-of-range nonce inputs.
_UINT32_MIN = 0
_UINT32_MAX = 0xFFFFFFFF
_DIRECTION_VALUES = (DIR_DEVICE_TO_SERVER, DIR_SERVER_TO_DEVICE)


def derive_device_key(master_key, device_id):
    """Return K_dev = HKDF-SHA256(master_key, SALT, device_id) (SPEC Section 3)."""
    if len(master_key) != KEY_LEN_BYTES:
        raise ValueError("master_key must be KEY_LEN_BYTES")
    kdf = HKDF(
        algorithm=hashes.SHA256(),
        length=KEY_LEN_BYTES,
        salt=SALT,
        info=device_id.encode("utf-8"),
    )
    return kdf.derive(master_key)


def build_nonce(session_id, event_counter, direction):
    """Return the 12-byte nonce for one packet (SPEC Section 4, big-endian)."""
    if not _UINT32_MIN <= session_id <= _UINT32_MAX:
        raise ValueError("session_id out of uint32 range")
    if not _UINT32_MIN <= event_counter <= _UINT32_MAX:
        raise ValueError("event_counter out of uint32 range")
    if direction not in _DIRECTION_VALUES:
        raise ValueError("direction must be a known DIR_* value")
    nonce = (
        struct.pack(">I", session_id)
        + struct.pack(">I", event_counter)
        + bytes([direction])
        + RESERVED_FILL * RESERVED_LEN_BYTES
    )
    if len(nonce) != NONCE_LEN_BYTES:
        raise ValueError("nonce length mismatch")
    return nonce


def build_telemetry_aad(device_id, msg_type, ts, event, heartbeat_int):
    """Return the canonical telemetry AAD bytes (SPEC Section 5)."""
    fields = (device_id, msg_type, str(ts), event, str(heartbeat_int))
    return AAD_FIELD_SEPARATOR.join(fields).encode("utf-8")


def build_ack_aad(device_id, ts, event):
    """Return the canonical ACK AAD bytes (SPEC Section 5)."""
    fields = (device_id, AAD_TYPE_ACK, str(ts), event, ACK_HEARTBEAT_FIELD)
    return AAD_FIELD_SEPARATOR.join(fields).encode("utf-8")


def encrypt(key, nonce, aad, plaintext):
    """Return ciphertext with the 16-byte GCM tag appended (SPEC Section 6)."""
    if len(key) != KEY_LEN_BYTES:
        raise ValueError("key must be KEY_LEN_BYTES")
    if len(nonce) != NONCE_LEN_BYTES:
        raise ValueError("nonce must be NONCE_LEN_BYTES")
    return AESGCM(key).encrypt(nonce, plaintext, aad)


def decrypt(key, nonce, aad, ciphertext_and_tag):
    """Return plaintext; raise InvalidTag on auth failure (SPEC Section 9)."""
    if len(key) != KEY_LEN_BYTES:
        raise ValueError("key must be KEY_LEN_BYTES")
    if len(nonce) != NONCE_LEN_BYTES:
        raise ValueError("nonce must be NONCE_LEN_BYTES")
    if len(ciphertext_and_tag) < TAG_LEN_BYTES:
        raise ValueError("ciphertext shorter than TAG_LEN_BYTES")
    return AESGCM(key).decrypt(nonce, ciphertext_and_tag, aad)
