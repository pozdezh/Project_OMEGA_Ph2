// Omega DTLS 1.3 client for the ESP32-S3 NMU (Brick 4).
//
// WHY THIS REPLACES THE BRICK 3 VERSION
//
// Brick 3 used mbedTLS DTLS 1.2. On this board that path never worked: the
// device panics and reboot-loops roughly 13-15 s into every connection
// attempt, before the handshake timeout, and raising the task stack to 32 KB
// did not help (LOGS/OPEN_ISSUE_dtls12_nmu_panic.md, still open). The same
// board completes a wolfSSL DTLS 1.3 handshake reliably - proven over a
// 1-hour soak on real hardware. So 1.3 here is not the adventurous choice,
// it is the only one that has ever run on the NMU.
//
// WHAT IS DIFFERENT BEYOND THE LIBRARY
//
//   SESSION REUSE. The session stays open across many records. The handshake
//   costs ~5 s on this chip; the microphone produces an event every 0.5-1 s,
//   so a handshake per event can never keep up - the work arrives ten times
//   faster than it completes. Amortised over a session it is ~200 ms/record.
//   Measured: 96-116 records per handshake.
//
//   select()-GATED READS. wolfSSL's own DTLS receive callback rewrites the
//   socket's SO_RCVTIMEO to {0,0} - POSIX for "block forever" - the instant
//   the handshake completes. Any read relying on that timeout hangs forever
//   on the first lost datagram. select() keeps its own clock and is immune.
//
//   RESEND BEFORE RECONNECT. A lost datagram does not mean a broken session;
//   the key is still valid and the peer still authenticated. Re-send the same
//   record on the same session first. In the soak this cut loss-driven
//   handshakes from 46% of all handshakes to near zero.
//
//   NO HARDCODED SERVER ADDRESS. The address is discovered (mDNS, then UDP
//   broadcast, with the last-good address cached), so a DHCP change no longer
//   bricks the fleet.
//
// The sketch keeps its sampling, buffering and self-heal logic unchanged.

#ifndef OMEGA_DTLS_H
#define OMEGA_DTLS_H

#include <Arduino.h>

#include <wolfssl.h>
#include <wolfssl/ssl.h>

// Outcome of one sendRecord() call, so the caller can tell "the network
// dropped a packet" (retry, keep the session) apart from "this session is
// dead" (rebuild it) apart from "the server wants us to re-authenticate".
enum OmegaSendResult {
  OMEGA_SEND_OK = 0,        // ACK received and matched
  OMEGA_SEND_REAUTH = 1,    // ACK received, server asked for a re-handshake
  OMEGA_SEND_LOST = 2,      // no ACK after every retry; session torn down
  OMEGA_SEND_BROKEN = 3,    // transport/TLS error; session torn down
};

class OmegaDtls {
 public:
  OmegaDtls();
  ~OmegaDtls();

  // One-time load of the CA cert, this device's cert and private key (PEM,
  // NUL-terminated). Returns 0 on success, negative on failure.
  int begin(const char* ca_pem, const char* device_cert_pem,
            const char* device_key_pem);

  // Establish a DTLS 1.3 session. Returns true on a completed mutual-auth
  // handshake; false leaves the caller to buffer and retry.
  bool connect(const char* server_ip, uint16_t port,
               uint32_t handshake_timeout_ms);

  // Send one JSON record over the encrypted channel and wait for the ACK,
  // re-sending on the same session if the reply goes missing.
  OmegaSendResult sendRecord(const char* json_record, char* ack_out,
                             size_t ack_cap, uint32_t ack_timeout_ms,
                             const char* expect_event = nullptr);

  void close();

  bool connected() const { return connected_; }
  const char* version() const;
  const char* cipher() const;

  // Counters for the serial report and the memo.
  uint32_t handshakes() const { return handshakes_; }
  uint32_t resendAttempts() const { return resend_attempts_; }
  uint32_t recoveredByResend() const { return recovered_by_resend_; }
  uint32_t recordsLost() const { return records_lost_; }
  uint32_t staleAcks() const { return stale_acks_; }
  uint32_t lastHandshakeMs() const { return last_handshake_ms_; }

 private:
  // select() on the raw socket: the only timeout wolfSSL cannot overwrite.
  static bool waitReadable(int fd, uint32_t timeout_ms);

  WOLFSSL_CTX* ctx_;
  WOLFSSL* ssl_;
  int sockfd_;
  bool connected_;
  bool ready_;

  uint32_t handshakes_;
  uint32_t resend_attempts_;
  uint32_t recovered_by_resend_;
  uint32_t records_lost_;
  uint32_t stale_acks_;
  uint32_t last_handshake_ms_;
};

#endif  // OMEGA_DTLS_H
