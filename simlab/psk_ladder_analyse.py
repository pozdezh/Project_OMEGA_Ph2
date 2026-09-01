"""Measure the Rull Ventura ladder capture: what an eavesdropper actually gets.

Reads the pcap taken while psk_ladder_emit.py ran, splits it into the
four rungs by UDP port, and scores each on the same axes. Reads the raw
pcap directly - no Wireshark or tshark needed, and no decryption of
anything: this only looks at bytes that are already in the clear on the
wire, which is precisely the eavesdropper's position.

The AMU's own address is LEARNED from the rung A packets rather than
configured, so rung D (the production DTLS session) is filtered to the same
unit that emitted rungs A/B/C. That makes the comparison same-device by
construction, not by assertion.

Axes, and why each one is here:

  readable terms   does any device name, sensor name or JSON key survive
  entropy          bits per byte; plain JSON lands near 4-5, ciphertext
                   near 8. Below ~7.5 means structure is leaking
  repeated blocks  identical 16-byte ciphertext blocks across the whole
                   rung. This is the ECB weakness Rull Ventura named in her own
                   future work: with no IV and one fixed key, equal
                   plaintext blocks always produce equal ciphertext blocks,
                   so an observer sees which records share content
"""
import collections
import math
import struct
import sys

PORT_CLEARTEXT = 11501
PORT_ECB_HMAC = 11502
PORT_GCM = 11503
PORT_DTLS = 11400

AES_BLOCK_BYTES = 16
HMAC_TAG_LEN_BYTES = 32
GCM_NONCE_LEN_BYTES = 12
DTLS_UNIFIED_HEADER_MAX = 8

# Quoted JSON keys and the device-id prefix, never bare two-letter fragments:
# a 2-byte needle turns up by chance roughly once every 64 kB of random data,
# so "pm" or "id" would report false hits against correct ciphertext and make
# a clean result look dirty. Every term here is 4+ bytes.
SEARCH_TERMS = (b"AMU_", b"NMU_", b"airq", b"\"id\"", b"\"type\"", b"\"ts\"",
                b"\"cause\"", b"\"event\"", b"\"sensors\"", b"co2_ppm",
                b"pm2_5", b"scd30", b"dht22", b"temperature_c", b"light_lux")

RUNGS = (
    (PORT_CLEARTEXT, "A", "none (plain JSON)", 0),
    (PORT_ECB_HMAC, "B", "AES-128-ECB + HMAC-SHA256", 0),
    (PORT_GCM, "C", "AES-256-GCM", GCM_NONCE_LEN_BYTES),
    (PORT_DTLS, "D", "DTLS 1.3 (mutual-auth PKI)", DTLS_UNIFIED_HEADER_MAX),
)

LINKTYPE_ETHERNET = 1
LINKTYPE_LINUX_SLL = 113
LINKTYPE_LINUX_SLL2 = 276
IP_PROTO_UDP = 17
ETHERTYPE_IPV4 = 0x0800


def read_pcap(path):
    """Yield (src_ip, sport, dst_ip, dport, udp_payload) for every UDP
    datagram in a classic pcap, for the link types tcpdump produces here."""
    with open(path, "rb") as handle:
        data = handle.read()

    magic = data[:4]
    if magic == b"\xd4\xc3\xb2\xa1":
        endian = "<"
    elif magic == b"\xa1\xb2\xc3\xd4":
        endian = ">"
    else:
        raise SystemExit("not a classic pcap file")

    linktype = struct.unpack(endian + "I", data[20:24])[0]
    offset = 24
    while offset + 16 <= len(data):
        _, _, incl_len, _ = struct.unpack(endian + "IIII", data[offset:offset + 16])
        offset += 16
        packet = data[offset:offset + incl_len]
        offset += incl_len
        parsed = parse_udp(packet, linktype)
        if parsed is not None:
            yield parsed


def parse_udp(packet, linktype):
    if linktype == LINKTYPE_ETHERNET:
        if len(packet) < 14:
            return None
        ethertype = struct.unpack(">H", packet[12:14])[0]
        start = 14
    elif linktype == LINKTYPE_LINUX_SLL:
        if len(packet) < 16:
            return None
        ethertype = struct.unpack(">H", packet[14:16])[0]
        start = 16
    elif linktype == LINKTYPE_LINUX_SLL2:
        if len(packet) < 20:
            return None
        ethertype = struct.unpack(">H", packet[0:2])[0]
        start = 20
    else:
        return None

    if ethertype != ETHERTYPE_IPV4 or len(packet) < start + 20:
        return None
    header_len = (packet[start] & 0x0F) * 4
    if packet[start + 9] != IP_PROTO_UDP:
        return None
    src_ip = ".".join(str(b) for b in packet[start + 12:start + 16])
    dst_ip = ".".join(str(b) for b in packet[start + 16:start + 20])
    udp_start = start + header_len
    if len(packet) < udp_start + 8:
        return None
    sport, dport, udp_len, _ = struct.unpack(">HHHH", packet[udp_start:udp_start + 8])
    payload = packet[udp_start + 8:udp_start + udp_len]
    return src_ip, sport, dst_ip, dport, payload


def shannon_entropy(blob):
    if not blob:
        return 0.0
    counts = collections.Counter(blob)
    total = len(blob)
    return -sum((n / total) * math.log2(n / total) for n in counts.values())


def count_repeated_blocks(records, skip_bytes):
    """Identical 16-byte blocks seen more than once across the whole rung.

    Returns (total_blocks, distinct_blocks, blocks_that_repeat). A mode with
    a per-record IV or nonce should score zero repeats even on identical
    plaintext; ECB with one fixed key cannot."""
    blocks = []
    for record in records:
        body = record[skip_bytes:]
        usable = len(body) - (len(body) % AES_BLOCK_BYTES)
        for index in range(0, usable, AES_BLOCK_BYTES):
            blocks.append(body[index:index + AES_BLOCK_BYTES])
    counts = collections.Counter(blocks)
    repeated = sum(n for n in counts.values() if n > 1)
    return len(blocks), len(counts), repeated


def readable_terms(records):
    joined = b"".join(records)
    return {term.decode(): joined.count(term) for term in SEARCH_TERMS
            if joined.count(term) > 0}


def main():
    if len(sys.argv) < 2:
        print("usage: psk_ladder_analyse.py <capture.pcap>", file=sys.stderr)
        return 2
    path = sys.argv[1]

    datagrams = list(read_pcap(path))

    senders = {src for src, _, _, dport, _ in datagrams if dport == PORT_CLEARTEXT}
    if len(senders) != 1:
        print("expected exactly one rung A sender, found %s" % sorted(senders),
              file=sys.stderr)
        return 3
    amu_ip = senders.pop()

    print("capture       : %s" % path)
    print("datagrams     : %d" % len(datagrams))
    print("AMU (learned) : %s   <- from the rung A packets" % amu_ip)
    print("")

    results = []
    for port, letter, label, skip in RUNGS:
        records = [payload for src, _, _, dport, payload in datagrams
                   if dport == port and src == amu_ip and payload]
        if not records:
            print("rung %s: no records on port %d" % (letter, port))
            continue

        joined = b"".join(records)
        total, distinct, repeated = count_repeated_blocks(records, skip)
        found = readable_terms(records)
        results.append({
            "rung": letter, "label": label, "port": port,
            "records": len(records),
            "avg_size": sum(len(r) for r in records) / len(records),
            "entropy": shannon_entropy(joined),
            "terms": found,
            "blocks": total, "distinct": distinct, "repeated": repeated,
        })

    print("%-4s %-28s %-7s %-7s %-8s %-10s %s"
          % ("RUNG", "PROTECTION", "RECS", "AVG B", "ENTROPY", "REPEATED", "READABLE"))
    print("-" * 92)
    for r in results:
        readable = "none" if not r["terms"] else \
            ", ".join("%s x%d" % (k, v) for k, v in sorted(r["terms"].items()))
        print("%-4s %-28s %-7d %-7.0f %-8.2f %-10s %s"
              % (r["rung"], r["label"], r["records"], r["avg_size"],
                 r["entropy"],
                 "%d/%d" % (r["repeated"], r["blocks"]),
                 readable[:34]))
    print("-" * 92)
    print("REPEATED = identical 16-byte blocks / total blocks in that rung.")
    print("ENTROPY  = bits per byte over the whole rung (8.00 = random).")
    print("")

    for r in results:
        print("rung %s - %s (port %d)" % (r["rung"], r["label"], r["port"]))
        print("  records            : %d, average %.0f bytes"
              % (r["records"], r["avg_size"]))
        print("  entropy            : %.2f bits/byte" % r["entropy"])
        print("  16-byte blocks     : %d total, %d distinct, %d repeated"
              % (r["blocks"], r["distinct"], r["repeated"]))
        if r["terms"]:
            print("  READABLE ON WIRE   : %s"
                  % ", ".join("%s x%d" % (k, v) for k, v in sorted(r["terms"].items())))
        else:
            print("  readable on wire   : nothing")
        print("")

    return 0


if __name__ == "__main__":
    sys.exit(main())
