"""Deterministic in-memory DTLS 1.2 handshake, no UDP sockets.

Live UDP DTLS loopback is unreliable on Windows (the routing demux of
python3-dtls and MSYS openssl both mishandle datagram loopback); none of that
affects the Linux server/AMU or the ESP32. To verify the true-DTLS crypto on
this laptop regardless, we drive two OpenSSL SSL objects joined by memory BIOs
and pump the handshake bytes between them in-process. This proves exactly the
security-relevant facts - ECDHE-ECDSA mutual authentication, forward-secret
cipher selection, and cert-bound identity - with zero networking.

It binds the OpenSSL 1.1 shared libraries that python3-dtls already ships, so
there is no extra dependency. The stateless-cookie anti-DoS exchange is a
server socket feature (DTLSv1_listen) and is out of scope here; it is exercised
separately in the live server tests.
"""

import ctypes
import os

from dtls import openssl as _pkg  # only to locate the shipped DLLs

_PKG_DIR = os.path.dirname(_pkg.__file__)
libcrypto = ctypes.CDLL(os.path.join(_PKG_DIR, "libcrypto-1_1-x64.dll"))
libssl = ctypes.CDLL(os.path.join(_PKG_DIR, "libssl-1_1-x64.dll"))

SSL_ERROR_WANT_READ = 2
SSL_ERROR_WANT_WRITE = 3
SSL_VERIFY_PEER = 0x01
SSL_VERIFY_FAIL_IF_NO_PEER_CERT = 0x02
SSL_FILETYPE_PEM = 1
X509_V_OK = 0
NID_commonName = 13
MEM_CHUNK = 4096
MAX_STEPS = 200

_VP = ctypes.c_void_p
_CP = ctypes.c_char_p


def _bind(lib, name, restype, argtypes):
    fn = getattr(lib, name)
    fn.restype = restype
    fn.argtypes = argtypes
    return fn


libssl.OPENSSL_init_ssl(0, None)

BIO_new = _bind(libcrypto, "BIO_new", _VP, [_VP])
BIO_s_mem = _bind(libcrypto, "BIO_s_mem", _VP, [])
BIO_read = _bind(libcrypto, "BIO_read", ctypes.c_int, [_VP, _CP, ctypes.c_int])
BIO_write = _bind(libcrypto, "BIO_write", ctypes.c_int, [_VP, _CP, ctypes.c_int])
X509_get_subject_name = _bind(libcrypto, "X509_get_subject_name", _VP, [_VP])
X509_NAME_get_text_by_NID = _bind(
    libcrypto, "X509_NAME_get_text_by_NID", ctypes.c_int, [_VP, ctypes.c_int, _CP, ctypes.c_int]
)
X509_free = _bind(libcrypto, "X509_free", None, [_VP])

DTLSv1_2_server_method = _bind(libssl, "DTLSv1_2_server_method", _VP, [])
DTLSv1_2_client_method = _bind(libssl, "DTLSv1_2_client_method", _VP, [])
SSL_CTX_new = _bind(libssl, "SSL_CTX_new", _VP, [_VP])
SSL_CTX_free = _bind(libssl, "SSL_CTX_free", None, [_VP])
SSL_CTX_use_certificate_chain_file = _bind(
    libssl, "SSL_CTX_use_certificate_chain_file", ctypes.c_int, [_VP, _CP]
)
SSL_CTX_use_PrivateKey_file = _bind(
    libssl, "SSL_CTX_use_PrivateKey_file", ctypes.c_int, [_VP, _CP, ctypes.c_int]
)
SSL_CTX_load_verify_locations = _bind(
    libssl, "SSL_CTX_load_verify_locations", ctypes.c_int, [_VP, _CP, _CP]
)
SSL_CTX_set_verify = _bind(libssl, "SSL_CTX_set_verify", None, [_VP, ctypes.c_int, _VP])
SSL_CTX_set_cipher_list = _bind(libssl, "SSL_CTX_set_cipher_list", ctypes.c_int, [_VP, _CP])
SSL_new = _bind(libssl, "SSL_new", _VP, [_VP])
SSL_free = _bind(libssl, "SSL_free", None, [_VP])
SSL_set_bio = _bind(libssl, "SSL_set_bio", None, [_VP, _VP, _VP])
SSL_set_accept_state = _bind(libssl, "SSL_set_accept_state", None, [_VP])
SSL_set_connect_state = _bind(libssl, "SSL_set_connect_state", None, [_VP])
SSL_do_handshake = _bind(libssl, "SSL_do_handshake", ctypes.c_int, [_VP])
SSL_read = _bind(libssl, "SSL_read", ctypes.c_int, [_VP, _CP, ctypes.c_int])
SSL_write = _bind(libssl, "SSL_write", ctypes.c_int, [_VP, _CP, ctypes.c_int])
SSL_get_error = _bind(libssl, "SSL_get_error", ctypes.c_int, [_VP, ctypes.c_int])
SSL_get_peer_certificate = _bind(libssl, "SSL_get_peer_certificate", _VP, [_VP])
SSL_get_verify_result = _bind(libssl, "SSL_get_verify_result", ctypes.c_long, [_VP])
SSL_get_current_cipher = _bind(libssl, "SSL_get_current_cipher", _VP, [_VP])
SSL_CIPHER_get_name = _bind(libssl, "SSL_CIPHER_get_name", _CP, [_VP])


class HandshakeError(Exception):
    pass


def _b(path):
    return path.encode("utf-8")


def _make_ctx(method_fn, cert, key, ca, verify_mode, ciphers):
    ctx = SSL_CTX_new(method_fn())
    if not ctx:
        raise HandshakeError("SSL_CTX_new failed")
    if SSL_CTX_use_certificate_chain_file(ctx, _b(cert)) != 1:
        raise HandshakeError("load cert failed: %s" % cert)
    if SSL_CTX_use_PrivateKey_file(ctx, _b(key), SSL_FILETYPE_PEM) != 1:
        raise HandshakeError("load key failed: %s" % key)
    if SSL_CTX_load_verify_locations(ctx, _b(ca), None) != 1:
        raise HandshakeError("load CA failed: %s" % ca)
    SSL_CTX_set_verify(ctx, verify_mode, None)
    if SSL_CTX_set_cipher_list(ctx, _b(ciphers)) != 1:
        raise HandshakeError("set cipher list failed")
    return ctx


def _new_ssl_with_mem_bios(ctx):
    rbio = BIO_new(BIO_s_mem())
    wbio = BIO_new(BIO_s_mem())
    ssl = SSL_new(ctx)
    if not ssl:
        raise HandshakeError("SSL_new failed")
    SSL_set_bio(ssl, rbio, wbio)  # SSL owns the BIOs after this
    return ssl, rbio, wbio


def _drain_into(src_wbio, dst_rbio):
    """Move all pending bytes from one peer's write BIO to the other's read BIO."""
    buf = ctypes.create_string_buffer(MEM_CHUNK)
    moved = 0
    while True:
        n = BIO_read(src_wbio, buf, MEM_CHUNK)
        if n <= 0:
            break
        BIO_write(dst_rbio, buf, n)
        moved += n
    return moved


def _drain_capture(src_wbio, dst_rbio):
    """Like _drain_into, but also return the exact bytes moved across the wire."""
    chunks = []
    buf = ctypes.create_string_buffer(MEM_CHUNK)
    while True:
        n = BIO_read(src_wbio, buf, MEM_CHUNK)
        if n <= 0:
            break
        chunks.append(buf.raw[:n])
        BIO_write(dst_rbio, buf, n)
    return b"".join(chunks)


def _peer_common_name(ssl):
    cert = SSL_get_peer_certificate(ssl)
    if not cert:
        return None
    try:
        name = X509_get_subject_name(cert)
        out = ctypes.create_string_buffer(256)
        length = X509_NAME_get_text_by_NID(name, NID_commonName, out, 256)
        if length < 0:
            return None
        return out.value.decode("utf-8")
    finally:
        X509_free(cert)


def _cipher_name(ssl):
    cipher = SSL_get_current_cipher(ssl)
    if not cipher:
        return None
    raw = SSL_CIPHER_get_name(cipher)
    return raw.decode("utf-8") if raw else None


def _step(ssl, name):
    """Advance one handshake step; return True when complete."""
    rc = SSL_do_handshake(ssl)
    if rc == 1:
        return True
    err = SSL_get_error(ssl, rc)
    if err in (SSL_ERROR_WANT_READ, SSL_ERROR_WANT_WRITE):
        return False
    raise HandshakeError("%s handshake failed, SSL_get_error=%d" % (name, err))


class DtlsChannel:
    """An established in-memory mutual-auth DTLS 1.2 session between a server
    and a client SSL object, with the handshake already complete. Application
    records are moved across by draining one peer's write BIO into the other's
    read BIO - the same job a UDP socket does in production, minus the network.
    Use as a context manager so the OpenSSL objects are always freed."""

    def __init__(self, pki_dir, server_cn, client_cn, ciphers):
        ca = os.path.join(pki_dir, "ca-cert.pem")
        self._server_ctx = _make_ctx(
            DTLSv1_2_server_method,
            os.path.join(pki_dir, server_cn + "-cert.pem"),
            os.path.join(pki_dir, server_cn + "-key.pem"),
            ca, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, ciphers,
        )
        self._client_ctx = _make_ctx(
            DTLSv1_2_client_method,
            os.path.join(pki_dir, client_cn + "-cert.pem"),
            os.path.join(pki_dir, client_cn + "-key.pem"),
            ca, SSL_VERIFY_PEER, ciphers,
        )
        self._server, self._s_rbio, self._s_wbio = _new_ssl_with_mem_bios(self._server_ctx)
        self._client, self._c_rbio, self._c_wbio = _new_ssl_with_mem_bios(self._client_ctx)
        SSL_set_accept_state(self._server)
        SSL_set_connect_state(self._client)
        self._last_wire = b""
        self._handshake()

    def _handshake(self):
        server_done = client_done = False
        for _i in range(MAX_STEPS):
            client_done = _step(self._client, "client") or client_done
            _drain_into(self._c_wbio, self._s_rbio)
            server_done = _step(self._server, "server") or server_done
            _drain_into(self._s_wbio, self._c_rbio)
            if server_done and client_done:
                return
        raise HandshakeError("handshake did not converge")

    def info(self):
        return {
            "cipher": _cipher_name(self._client),
            "server_view_of_client": _peer_common_name(self._server),
            "client_view_of_server": _peer_common_name(self._client),
            "server_verify_ok": SSL_get_verify_result(self._server) == X509_V_OK,
            "client_verify_ok": SSL_get_verify_result(self._client) == X509_V_OK,
        }

    def client_common_name(self):
        return _peer_common_name(self._server)

    def client_to_server(self, data):
        """Encrypt data on the client and hand the plaintext to the server."""
        return self._transfer(self._client, self._c_wbio, self._s_rbio, self._server, data)

    def server_to_client(self, data):
        """Encrypt data on the server and hand the plaintext to the client."""
        return self._transfer(self._server, self._s_wbio, self._c_rbio, self._client, data)

    def last_wire(self):
        """The raw encrypted bytes of the most recent transfer - what a
        passive eavesdropper on the wire would capture."""
        return self._last_wire

    def client_to_server_tampered(self, data, flip_offset=-1):
        """Encrypt on the client, FLIP ONE BIT in flight, then deliver.

        Models an attacker who can rewrite bytes on a shared WLAN - the
        realistic active attack against a residence network, and the one a
        pre-shared-key scheme without authenticated encryption would not
        notice. Returns (wire_bytes, accepted); accepted is True only if the
        server took the altered record anyway, which it must never do.
        """
        n = SSL_write(self._client, data, len(data))
        if n <= 0:
            raise HandshakeError("SSL_write failed")

        chunks = []
        buf = ctypes.create_string_buffer(MEM_CHUNK)
        while True:
            moved = BIO_read(self._c_wbio, buf, MEM_CHUNK)
            if moved <= 0:
                break
            chunks.append(buf.raw[:moved])
        wire = bytearray(b"".join(chunks))
        if not wire:
            raise HandshakeError("nothing on the wire to tamper with")

        # Default to the last byte: inside the authentication tag, which is
        # what every record carries and what makes the forgery detectable.
        index = flip_offset if flip_offset >= 0 else len(wire) - 1
        wire[index] ^= 0x01
        payload = bytes(wire)
        BIO_write(self._s_rbio, payload, len(payload))

        got_buf = ctypes.create_string_buffer(MEM_CHUNK)
        got = SSL_read(self._server, got_buf, MEM_CHUNK)
        self._last_wire = payload
        return payload, got > 0

    def _transfer(self, writer, writer_wbio, reader_rbio, reader, data):
        n = SSL_write(writer, data, len(data))
        if n <= 0:
            raise HandshakeError("SSL_write failed")
        self._last_wire = _drain_capture(writer_wbio, reader_rbio)
        buf = ctypes.create_string_buffer(MEM_CHUNK)
        got = SSL_read(reader, buf, MEM_CHUNK)
        if got <= 0:
            raise HandshakeError("SSL_read failed")
        return buf.raw[:got]

    def close(self):
        for ssl in (self._server, self._client):
            if ssl:
                SSL_free(ssl)
        for ctx in (self._server_ctx, self._client_ctx):
            if ctx:
                SSL_CTX_free(ctx)
        self._server = self._client = self._server_ctx = self._client_ctx = None

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


def handshake(pki_dir, server_cn, client_cn, ciphers):
    """Run one in-memory mutual-auth DTLS 1.2 handshake and return its facts
    (cipher, cert-bound identities, verify results). Raises HandshakeError if
    the handshake does not complete (e.g. an untrusted client certificate)."""
    with DtlsChannel(pki_dir, server_cn, client_cn, ciphers) as channel:
        return channel.info()
