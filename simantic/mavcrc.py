"""Frame MAVLink v2 from the (UART1) hex lines of a sim --output file and check
each frame's CRC with the message's CRC_EXTRA; report per-id valid/invalid
counts and dump any invalid frames of the ids given on the command line."""
import re, sys

CRC_EXTRA = {0: 50, 1: 124, 24: 24, 27: 144, 29: 115, 30: 39, 36: 222, 65: 118, 132: 85, 147: 154, 253: 83}


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def frames(path):
    """Yield (msgid, payload, crc_ok, seq) for every MAVLink v2 frame on UART1."""
    d = bytearray()
    for l in open(path):
        m = re.match(r'\[[0-9.]+s\] \((UART1)\) ([0-9A-F ]+)$', l.strip())
        if m:
            d += bytes.fromhex(m[2].replace(' ', ''))
    i = 0
    while i < len(d) - 12:
        if d[i] != 0xFD:
            i += 1
            continue
        ln = d[i + 1]
        mid = d[i + 7] | d[i + 8] << 8 | d[i + 9] << 16
        frame = d[i:i + 12 + ln]
        if len(frame) < 12 + ln:
            break
        body, crc = frame[1:10 + ln], frame[10 + ln] | frame[11 + ln] << 8
        ok = mid in CRC_EXTRA and x25(bytes(body) + bytes([CRC_EXTRA[mid]])) == crc
        yield mid, bytes(frame[10:10 + ln]), ok, frame[4]
        i += 12 + ln if ok else 1


if __name__ == "__main__":
    stats = {}
    for mid, payload, ok, seq in frames(sys.argv[1]):
        stats.setdefault(mid, [0, 0])[0 if ok else 1] += 1
        if mid in {int(a) for a in sys.argv[2:]} and not ok:
            print(f"BAD id={mid} len={len(payload)} seq={seq} payload={payload.hex()}")
    for mid, (ok, bad) in sorted(stats.items()):
        print(f"id {mid}: ok={ok} bad={bad}")
