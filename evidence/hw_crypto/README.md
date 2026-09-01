# Hardware against software cryptography on the noise unit

Evidence for the memo's Results section "Hardware crypto acceleration,
measured and declined", and for Decision 3 in the design chapter.

## What was compared

Two development builds of the same DTLS 1.3 client, against the same server,
with the same certificates and the same wolfSSL version:

- **Arduino build, software cryptography.** The toolchain the deployed noise
  units use. wolfSSL's Arduino distribution does not define
  `WOLFSSL_ESP32_CRYPT` and links no Espressif crypto driver symbols.
- **Bare ESP-IDF build, hardware cryptography.** wolfSSL ships its Espressif
  hardware port as an ESP-IDF component, so reaching the accelerators means
  leaving Arduino.

## Result, both builds pinned to 240 MHz

| Metric | Arduino, software | ESP-IDF, hardware | Difference |
|---|---|---|---|
| Handshake, device clock | 4830 ms (n=4) | 4374 ms (n=1) | -456 ms (-9.4%) |
| Handshake, server clock | 4572.5 ms (n=2) | 4037 ms (n=2) | -535.5 ms (-11.7%) |
| Per-record average | 145 ms | 146 ms | +1 ms, none |
| Per-record range | 9-226 ms | 27-228 ms | same band |

## The confound that had to be removed first

The first attempt compared an ESP-IDF build at its silent default of 160 MHz
against an Arduino build at its board default of 240 MHz, and reported
hardware as *slower* (5697 ms against 4830 ms). Neither build set the CPU
frequency explicitly. The confound was found by reading `boards.txt` and the
generated `sdkconfig`, not by assuming either default.

Pinning both to 240 MHz recovered 1323 ms, roughly three times the size of the
effect being measured. The uncontrolled comparison is discarded and is kept
only to show what the confound did.

## Proof the accelerators actually ran

Taken from the run's own counters, not from the build configuration:
44,055 hardware big-number multiplications (the P-256 handshake mathematics)
and 102 hardware SHA-256 operations, with essentially no software fallback.

## Conclusion

Acceleration helps the compute-bound part, the handshake, and does nothing for
the per-record cost, which is dominated by Wi-Fi radio wake latency. A session
is established once and then carries many records, so the accelerated quantity
is the rare one. Nine percent on a once-per-session event does not justify
losing the Arduino flashing and serial workflow used across the fleet, so the
shipped build stays on software cryptography.

## Files

| File | What it is |
|---|---|
| `SESSION_REUSE_AND_HW_ACCEL_RESULTS.md` | Full write-up, including the session-reuse measurements that preceded this comparison |
| `nmu_espidf_240mhz_controlled_20260815.txt` | Raw serial log of the controlled 240 MHz hardware run |
| `nmu_espidf_hw_metrics_20260813.txt` | Raw serial log carrying the hardware-operation counters |

The Wi-Fi network name and the access point's hardware address were replaced
with placeholders in the two serial logs. Nothing else was altered.
