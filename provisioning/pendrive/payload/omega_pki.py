"""Omega DTLS provisioning - offline elliptic-curve PKI.

Brick 3 replaces the pre-shared symmetric keys of Bricks 1-2 with asymmetric
identity keys. Trust is anchored in ONE offline Certificate Authority whose
private key lives on the server (or an air-gapped box) and never ships to a
device. The CA signs:

  - one server certificate (CN "omega-server"), presented by the DTLS server;
  - one certificate per device (CN = device_id, e.g. "AMU_01"/"NMU_01"),
    flashed/installed on that device.

At handshake time each side proves identity by signing with its private key;
the peer verifies against the CA certificate it already holds. The device_id
therefore cannot be spoofed: it is bound into a CA-signed certificate, so the
server reads identity from the verified peer certificate, never from payload.

Curve is NIST P-256 (secp256r1): small certificates (a few hundred bytes, fits
an ESP32 flash string) and hardware-accelerated on the ESP32-S3. Keys are
PEM files; leaf private keys are written mode 600.

CLI (run once on the server / provisioning box):
    python3 omega_pki.py init <out_dir>
        create ca, server, and the default AMU_01..03 / NMU_01..03 device certs
    python3 omega_pki.py device <out_dir> <device_id>
        issue one more device cert against the existing CA in <out_dir>

Never prints or copies the CA private key. Idempotent for leaves: re-issuing a
device_id overwrites only that device's files.
"""

import datetime
import os
import stat
import sys

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.serialization import pkcs12
from cryptography.x509.oid import NameOID

CURVE = ec.SECP256R1()
SIGNATURE_HASH = hashes.SHA256()
CA_VALIDITY_DAYS = 3650
LEAF_VALIDITY_DAYS = 3650
CA_COMMON_NAME = "omega-ca"
SERVER_COMMON_NAME = "omega-server"
OPERATOR_COMMON_NAME = "operator"
DEFAULT_DEVICES = ("AMU_01", "AMU_02", "AMU_03", "NMU_01", "NMU_02", "NMU_03")
SERVER_HOSTNAME = "smartageing.local"
PRIVATE_KEY_MODE = 0o600
CERT_MODE = 0o644


def _now():
    return datetime.datetime.now(datetime.timezone.utc)


def _new_key():
    return ec.generate_private_key(CURVE)


def _write_key(key, path):
    data = key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption(),
    )
    with open(path, "wb") as handle:
        handle.write(data)
    os.chmod(path, PRIVATE_KEY_MODE)


def _write_cert(cert, path):
    with open(path, "wb") as handle:
        handle.write(cert.public_bytes(serialization.Encoding.PEM))
    os.chmod(path, CERT_MODE)


def _name(common_name):
    return x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, common_name)])


def create_ca():
    key = _new_key()
    subject = _name(CA_COMMON_NAME)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(subject)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(_now())
        .not_valid_after(_now() + datetime.timedelta(days=CA_VALIDITY_DAYS))
        .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
        .add_extension(
            x509.KeyUsage(
                digital_signature=False, content_commitment=False,
                key_encipherment=False, data_encipherment=False,
                key_agreement=False, key_cert_sign=True, crl_sign=True,
                encipher_only=False, decipher_only=False,
            ),
            critical=True,
        )
        .sign(key, SIGNATURE_HASH)
    )
    return key, cert


def issue_leaf(ca_key, ca_cert, common_name, is_server):
    key = _new_key()
    # Dual-purpose (client + server auth) so a node can act in both roles: the
    # AMU is a DTLS client to the main server AND a DTLS server for the live MCP
    # channel. OpenSSL's SSL layer enforces this EKU purpose, so a client-only
    # cert would be rejected when the AMU serves.
    usage = x509.ExtendedKeyUsage(
        [x509.ExtendedKeyUsageOID.SERVER_AUTH, x509.ExtendedKeyUsageOID.CLIENT_AUTH]
    )
    builder = (
        x509.CertificateBuilder()
        .subject_name(_name(common_name))
        .issuer_name(ca_cert.subject)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(_now())
        .not_valid_after(_now() + datetime.timedelta(days=LEAF_VALIDITY_DAYS))
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(usage, critical=False)
    )
    if is_server:
        # Every current browser ignores the Common Name for hostname matching
        # and requires a Subject Alternative Name entry instead (enforced by
        # Chrome since 2017, and the CA/Browser Forum baseline requirements).
        # A cert with CN=omega-server and no SAN is silently rejected by every
        # browser, not a warning - the padlock never appears at all.
        #
        # The SAN is the mDNS name, not an IP: the DTLS layer already proves
        # DHCP-agnostic naming works, and baking in an IP here would recreate
        # the exact hardcoded-address problem that was solved there.
        builder = builder.add_extension(
            x509.SubjectAlternativeName([x509.DNSName(SERVER_HOSTNAME)]),
            critical=False,
        )
    cert = builder.sign(ca_key, SIGNATURE_HASH)
    return key, cert


class CaKeyMissing(Exception):
    """Raised in place of a raw FileNotFoundError so every caller can print
    the same one-line, no-traceback message: this happens in normal use
    whenever the pendrive holding the CA key is not plugged in."""


def _load_ca(out_dir, ca_key_dir=None):
    """ca_key_dir lets the private key be read from removable media (a
    pendrive) while everything else - ca-cert.pem, every issued certificate -
    stays on the machine doing the signing. The key is never copied onto that
    machine's own disk; it is only opened where it already is."""
    key_path = os.path.join(ca_key_dir or out_dir, "ca-key.pem")
    if not os.path.exists(key_path):
        raise CaKeyMissing(key_path)
    with open(key_path, "rb") as handle:
        ca_key = serialization.load_pem_private_key(handle.read(), password=None)
    with open(os.path.join(out_dir, "ca-cert.pem"), "rb") as handle:
        ca_cert = x509.load_pem_x509_certificate(handle.read())
    return ca_key, ca_cert


def _issue_and_save(out_dir, ca_key, ca_cert, common_name, is_server):
    key, cert = issue_leaf(ca_key, ca_cert, common_name, is_server)
    _write_key(key, os.path.join(out_dir, common_name + "-key.pem"))
    _write_cert(cert, os.path.join(out_dir, common_name + "-cert.pem"))


def cmd_init(out_dir, force_new_ca=False, with_default_devices=False):
    """Bring a directory up to a working PKI, reusing an existing CA if one is
    already there. Copying an offline ca-key.pem/ca-cert.pem into <out_dir> and
    running init is therefore the whole bootstrap on a fresh machine.

    Generating a second CA over the top of the first is never a recovery step -
    it silently orphans every certificate the fleet already trusts - so it
    happens only on an explicit --force-new-ca."""
    os.makedirs(out_dir, exist_ok=True)
    ca_key_path = os.path.join(out_dir, "ca-key.pem")
    ca_cert_path = os.path.join(out_dir, "ca-cert.pem")
    have_key = os.path.exists(ca_key_path)
    have_cert = os.path.exists(ca_cert_path)

    if (have_key or have_cert) and not force_new_ca:
        if not (have_key and have_cert):
            missing = "ca-cert.pem" if have_key else "ca-key.pem"
            print("REFUSED: %s holds half a CA - %s is missing." % (out_dir, missing))
            print("Restore the missing half from your offline copy. Generating a")
            print("new CA here would orphan every certificate already deployed.")
            return 3
        ca_key, ca_cert = _load_ca(out_dir)
        print("adopted the existing CA in %s (ca-key.pem untouched)" % out_dir)
    else:
        if force_new_ca and (have_key or have_cert):
            print("WARNING: --force-new-ca replaces the CA. Every device and")
            print("server certificate already issued becomes untrusted and every")
            print("unit must be reflashed or reinstalled.")
        ca_key, ca_cert = create_ca()
        _write_key(ca_key, ca_key_path)
        _write_cert(ca_cert, ca_cert_path)
        print("created a new CA in %s" % out_dir)

    for common_name, is_server in ((SERVER_COMMON_NAME, True),
                                   (OPERATOR_COMMON_NAME, False)):
        if os.path.exists(os.path.join(out_dir, common_name + "-cert.pem")):
            print("  kept    %s-cert.pem/-key.pem" % common_name)
            continue
        _issue_and_save(out_dir, ca_key, ca_cert, common_name, is_server)
        print("  issued  %s-cert.pem/-key.pem" % common_name)

    if with_default_devices:
        for device_id in DEFAULT_DEVICES:
            if os.path.exists(os.path.join(out_dir, device_id + "-cert.pem")):
                continue
            _issue_and_save(out_dir, ca_key, ca_cert, device_id, False)
            print("  issued  %s-cert.pem/-key.pem" % device_id)

    print("ca-cert.pem is the trust anchor - it ships to every device and the server.")
    print("KEEP ca-key.pem OFFLINE. It never goes on a device.")
    return 0


def cmd_device(out_dir, device_id, force=False, ca_key_dir=None):
    """Issue a device identity. Refuses to overwrite an existing one unless
    forced - see ARCHITECTURE.md 17 (duplicate identity protection)."""
    existing = os.path.join(out_dir, device_id + "-cert.pem")
    if os.path.exists(existing) and not force:
        print("REFUSED: %s already has a certificate at %s" % (device_id, existing))
        print("Two boards sharing one name both authenticate and their data")
        print("merges under a single device id. Pick an unused number, or")
        print("pass --force to deliberately replace this unit's identity.")
        return 1
    try:
        ca_key, ca_cert = _load_ca(out_dir, ca_key_dir)
    except CaKeyMissing as error:
        print("REFUSED: no ca-key.pem at %s" % error)
        print("Plug in the drive holding the CA key and pass its path with")
        print("--ca-key-dir, or copy ca-key.pem into %s." % out_dir)
        return 4
    _issue_and_save(out_dir, ca_key, ca_cert, device_id, False)
    print("issued %s-cert.pem/-key.pem" % device_id)
    return 0


def cmd_reissue_server(out_dir, ca_key_dir=None):
    """Re-issue ONLY the server leaf (now carrying its SAN) against the
    EXISTING CA. The CA and every device certificate are untouched, so
    already-provisioned devices keep trusting the result with no changes on
    their side - only omega-server-cert.pem/-key.pem are replaced."""
    try:
        ca_key, ca_cert = _load_ca(out_dir, ca_key_dir)
    except CaKeyMissing as error:
        print("REFUSED: no ca-key.pem at %s" % error)
        print("Plug in the drive holding the CA key and pass its path with")
        print("--ca-key-dir, or copy ca-key.pem into %s." % out_dir)
        return 4
    _issue_and_save(out_dir, ca_key, ca_cert, SERVER_COMMON_NAME, True)
    print("re-issued %s-cert.pem/-key.pem with SAN=DNS:%s"
          % (SERVER_COMMON_NAME, SERVER_HOSTNAME))
    print("restart the DTLS listener and reload nginx to pick it up")
    return 0


def cmd_export_operator(out_dir, password):
    """Bundle the operator identity (cert + private key + CA chain) into a
    single password-protected .p12 file, the format both Windows and macOS
    import as a browser client certificate. The plain .pem files this reads
    already exist from `init`; this only repackages them for import - no new
    key material is generated, so it can be re-run at any time."""
    with open(os.path.join(out_dir, OPERATOR_COMMON_NAME + "-key.pem"), "rb") as handle:
        key = serialization.load_pem_private_key(handle.read(), password=None)
    with open(os.path.join(out_dir, OPERATOR_COMMON_NAME + "-cert.pem"), "rb") as handle:
        cert = x509.load_pem_x509_certificate(handle.read())
    with open(os.path.join(out_dir, "ca-cert.pem"), "rb") as handle:
        ca_cert = x509.load_pem_x509_certificate(handle.read())

    bundle = pkcs12.serialize_key_and_certificates(
        b"omega-operator",
        key,
        cert,
        [ca_cert],
        serialization.BestAvailableEncryption(password.encode("utf-8")),
    )
    out_path = os.path.join(out_dir, "operator.p12")
    with open(out_path, "wb") as handle:
        handle.write(bundle)
    os.chmod(out_path, PRIVATE_KEY_MODE)
    print("wrote %s (password-protected - install as a client certificate "
          "in the browser that should access the dashboard)" % out_path)
    return 0


def _read_text(path):
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read().strip()


def cmd_arduino(out_dir, device_id, out_path):
    """Emit an Arduino/PlatformIO header embedding this device's identity for
    the NMU sketch: the CA cert, the device cert and the device private key as
    raw string literals. The CA PRIVATE key is never included."""
    ca = _read_text(os.path.join(out_dir, "ca-cert.pem"))
    cert = _read_text(os.path.join(out_dir, device_id + "-cert.pem"))
    key = _read_text(os.path.join(out_dir, device_id + "-key.pem"))
    lines = [
        "// Generated by omega_pki.py for %s - DO NOT COMMIT (contains the" % device_id,
        "// device private key). Flash only to %s." % device_id,
        "#ifndef OMEGA_CERTS_H",
        "#define OMEGA_CERTS_H",
        "",
        'static const char* OMEGA_DEVICE_ID = "%s";' % device_id,
        "",
        'static const char* OMEGA_CA_CERT_PEM = R"PEM(',
        ca,
        ')PEM";',
        "",
        'static const char* OMEGA_DEVICE_CERT_PEM = R"PEM(',
        cert,
        ')PEM";',
        "",
        'static const char* OMEGA_DEVICE_KEY_PEM = R"PEM(',
        key,
        ')PEM";',
        "",
        "#endif  // OMEGA_CERTS_H",
        "",
    ]
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
    print("wrote %s for %s (KEEP OUT OF GIT - holds the device key)" % (out_path, device_id))
    return 0


def _flag_value(argv, flag):
    """Value following FLAG in argv, e.g. ["--ca-key-dir", "/media/usb"]."""
    if flag in argv:
        index = argv.index(flag)
        if index + 1 < len(argv):
            return argv[index + 1]
    return None


def main(argv):
    ca_key_dir = _flag_value(argv, "--ca-key-dir")
    if len(argv) >= 3 and argv[1] == "init":
        return cmd_init(argv[2],
                        force_new_ca="--force-new-ca" in argv,
                        with_default_devices="--with-default-devices" in argv)
    if len(argv) >= 4 and argv[1] == "device":
        return cmd_device(argv[2], argv[3], "--force" in argv, ca_key_dir)
    if len(argv) >= 4 and argv[1] == "arduino":
        out_path = argv[4] if len(argv) >= 5 and not argv[4].startswith("--") else "omega_certs.h"
        return cmd_arduino(argv[2], argv[3], out_path)
    if len(argv) >= 3 and argv[1] == "reissue-server":
        return cmd_reissue_server(argv[2], ca_key_dir)
    if len(argv) >= 4 and argv[1] == "export-operator":
        return cmd_export_operator(argv[2], argv[3])
    print("usage: omega_pki.py init <out_dir> [--force-new-ca] [--with-default-devices]")
    print("       omega_pki.py device <out_dir> <device_id> [--force] [--ca-key-dir <path>]")
    print("       omega_pki.py arduino <out_dir> <device_id> [out.h]")
    print("       omega_pki.py reissue-server <out_dir> [--ca-key-dir <path>]")
    print("       omega_pki.py export-operator <out_dir> <password>")
    print()
    print("--ca-key-dir reads ca-key.pem from a DIFFERENT directory than <out_dir>")
    print("(e.g. a mounted pendrive) so the private key never has to be copied")
    print("onto this machine's own disk. Everything else is still read from and")
    print("written to <out_dir> as usual.")
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
