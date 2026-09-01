// Omega DTLS 1.3 client for the ESP32-S3 NMU (Brick 4). See omega_dtls.h for
// why this replaces the mbedTLS 1.2 version.

#include "omega_dtls.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <string.h>

static const int MAX_RECORD_ATTEMPTS = 3;

// DTLS handshake retry pacing, in seconds. First retry after 1 s, and never
// wait longer than 4 s between retries. Without the cap the library doubles
// the wait indefinitely (1, 2, 4, 8, 16...) and one connect attempt against a
// dead server outlasts the 30 s task watchdog, rebooting the board.
static const int DTLS_RETRY_INIT_S = 1;
static const int DTLS_RETRY_MAX_S = 4;

OmegaDtls::OmegaDtls()
    : ctx_(nullptr), ssl_(nullptr), sockfd_(-1), connected_(false),
      ready_(false), handshakes_(0), resend_attempts_(0),
      recovered_by_resend_(0), records_lost_(0), stale_acks_(0),
      last_handshake_ms_(0) {}

OmegaDtls::~OmegaDtls() {
  close();
  if (ctx_ != nullptr) {
    wolfSSL_CTX_free(ctx_);
    ctx_ = nullptr;
  }
}

int OmegaDtls::begin(const char* ca_pem, const char* device_cert_pem,
                     const char* device_key_pem) {
  if (wolfSSL_Init() != WOLFSSL_SUCCESS) {
    return -1;
  }
  ctx_ = wolfSSL_CTX_new(wolfDTLSv1_3_client_method());
  if (ctx_ == nullptr) {
    return -2;
  }

  // Trust anchor: the CA cert is the ONLY key material shipped to the device
  // in advance. The server's public key arrives inside its certificate at
  // every handshake and is verified against this.
  if (wolfSSL_CTX_load_verify_buffer(
          ctx_, (const unsigned char*)ca_pem, strlen(ca_pem) + 1,
          WOLFSSL_FILETYPE_PEM) != WOLFSSL_SUCCESS) {
    return -3;
  }
  if (wolfSSL_CTX_use_certificate_buffer(
          ctx_, (const unsigned char*)device_cert_pem,
          strlen(device_cert_pem) + 1, WOLFSSL_FILETYPE_PEM) != WOLFSSL_SUCCESS) {
    return -4;
  }
  if (wolfSSL_CTX_use_PrivateKey_buffer(
          ctx_, (const unsigned char*)device_key_pem,
          strlen(device_key_pem) + 1, WOLFSSL_FILETYPE_PEM) != WOLFSSL_SUCCESS) {
    return -5;
  }

  // Mutual authentication: we verify the server AND the server verifies us.
  // An attacker holding only the WLAN password gets no session.
  wolfSSL_CTX_set_verify(ctx_, WOLFSSL_VERIFY_PEER, nullptr);
  ready_ = true;
  return 0;
}

// select() never consults SO_RCVTIMEO, so it is immune to wolfSSL rewriting
// that option to "block forever" once the handshake completes.
bool OmegaDtls::waitReadable(int fd, uint32_t timeout_ms) {
  if (fd < 0) {
    return false;
  }
  fd_set readable;
  FD_ZERO(&readable);
  FD_SET(fd, &readable);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return select(fd + 1, &readable, nullptr, nullptr, &tv) > 0;
}

bool OmegaDtls::connect(const char* server_ip, uint16_t port,
                        uint32_t handshake_timeout_ms) {
  if (!ready_) {
    return false;
  }
  close();

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    return false;
  }

  struct sockaddr_in peer;
  memset(&peer, 0, sizeof(peer));
  peer.sin_family = AF_INET;
  peer.sin_port = htons(port);
  if (inet_pton(AF_INET, server_ip, &peer.sin_addr) != 1) {
    close();
    return false;
  }

  // A finite timeout IS honoured during the handshake - wolfSSL needs one to
  // drive its own retransmissions. It stops being honoured afterwards.
  struct timeval tv;
  tv.tv_sec = handshake_timeout_ms / 1000;
  tv.tv_usec = (handshake_timeout_ms % 1000) * 1000;
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (::connect(sockfd_, (struct sockaddr*)&peer, sizeof(peer)) != 0) {
    close();
    return false;
  }

  ssl_ = wolfSSL_new(ctx_);
  if (ssl_ == nullptr) {
    close();
    return false;
  }
  wolfSSL_set_fd(ssl_, sockfd_);

  // CAP THE HANDSHAKE RETRY BACKOFF. This is what stopped the board rebooting
  // whenever the server went away.
  //
  // DTLS runs over UDP, which does not resend lost packets itself, so the
  // library does it: if a handshake message goes unanswered it retries after
  // 1 s, then 2, then 4, 8, 16... doubling each time. Against a server that is
  // simply GONE, nothing ever answers, so a single wolfSSL_connect() call can
  // sit there for a minute or more - far longer than the socket timeout set
  // above, which limits one wait, not the whole sequence.
  //
  // Meanwhile the task watchdog reboots the board after 30 s of a task not
  // checking in, and nothing can check in from inside that blocking call. So a
  // server reboot rebooted every NMU with it, losing their buffered events -
  // confirmed on the bench (reset reason 6, task watchdog, reproducible with
  // no serial cable attached).
  //
  // Capping the backoff bounds the whole attempt to a few seconds. The device
  // then returns, buffers the event, and retries on its own schedule - which
  // is the behaviour that was always intended.
  wolfSSL_dtls_set_timeout_init(ssl_, DTLS_RETRY_INIT_S);
  wolfSSL_dtls_set_timeout_max(ssl_, DTLS_RETRY_MAX_S);

  const uint32_t started = millis();
  const int rc = wolfSSL_connect(ssl_);
  last_handshake_ms_ = millis() - started;

  if (rc != WOLFSSL_SUCCESS) {
    const int err = wolfSSL_get_error(ssl_, rc);
    Serial.printf("DTLS: handshake FAILED err=%d after %u ms\n", err,
                  (unsigned)last_handshake_ms_);
    close();
    return false;
  }

  connected_ = true;
  handshakes_++;
  Serial.printf("DTLS: session #%u up in %u ms (%s / %s)\n",
                (unsigned)handshakes_, (unsigned)last_handshake_ms_,
                version(), cipher());
  return true;
}

OmegaSendResult OmegaDtls::sendRecord(const char* json_record, char* ack_out,
                                      size_t ack_cap, uint32_t ack_timeout_ms,
                                      const char* expect_event) {
  if (!connected_ || ssl_ == nullptr) {
    return OMEGA_SEND_BROKEN;
  }
  const int len = (int)strlen(json_record);
  // The ACK names the event it answers. Matching it is what stops a LATE ack
  // for an earlier record - which UDP can deliver after that record's own read
  // already timed out - being counted as confirmation of this one, which would
  // drop the current record from the buffer without it ever being stored.
  // See FINDINGS #58.
  // The quoted event id, not "\"ack\":\"...\"" - the server serialises with a
  // space after the colon, and the event id is unique enough that its presence
  // in the ACK is conclusive on its own.
  char want[32] = "";
  if (expect_event != nullptr && expect_event[0] != '\0') {
    snprintf(want, sizeof(want), "\"%s\"", expect_event);
  }

  for (int attempt = 1; attempt <= MAX_RECORD_ATTEMPTS; attempt++) {
    if (wolfSSL_write(ssl_, json_record, len) != len) {
      // A real write error, not silence: the session is genuinely unusable.
      close();
      return OMEGA_SEND_BROKEN;
    }

    const uint32_t deadline = millis() + ack_timeout_ms;
    while ((int32_t)(deadline - millis()) > 0 &&
           waitReadable(sockfd_, (uint32_t)(deadline - millis()))) {
      memset(ack_out, 0, ack_cap);
      const int received = wolfSSL_read(ssl_, ack_out, (int)ack_cap - 1);
      if (received > 0) {
        ack_out[received] = '\0';
        if (want[0] != '\0' && strstr(ack_out, want) == nullptr) {
          stale_acks_++;
          continue;
        }
        if (attempt > 1) {
          recovered_by_resend_++;
          Serial.printf("DTLS: recovered on attempt %d, session kept\n", attempt);
        }
        // The server piggybacks reauth=1 when its session-age policy expires.
        // Honour it: that re-handshake is where our certificate gets checked
        // again, which is how revocation is enforced.
        if (strstr(ack_out, "\"reauth\":1") != nullptr) {
          close();
          return OMEGA_SEND_REAUTH;
        }
        return OMEGA_SEND_OK;
      }
    }

    // Silence. Re-send the SAME record on the SAME session - the event id is
    // unchanged, so if it was the ACK that was lost the server sees a
    // duplicate and its (id, event) index absorbs it.
    resend_attempts_++;
  }

  records_lost_++;
  close();
  return OMEGA_SEND_LOST;
}

void OmegaDtls::close() {
  if (ssl_ != nullptr) {
    wolfSSL_shutdown(ssl_);
    wolfSSL_free(ssl_);
    ssl_ = nullptr;
  }
  if (sockfd_ >= 0) {
    lwip_close(sockfd_);
    sockfd_ = -1;
  }
  connected_ = false;
}

const char* OmegaDtls::version() const {
  if (ssl_ == nullptr) {
    return "none";
  }
  return wolfSSL_get_version(ssl_);
}

const char* OmegaDtls::cipher() const {
  if (ssl_ == nullptr) {
    return "none";
  }
  return wolfSSL_get_cipher(ssl_);
}
