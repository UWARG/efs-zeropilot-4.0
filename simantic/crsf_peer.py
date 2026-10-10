# CRSF receiver on ZeroPilot's UART4, 420000 8N1 (drivers/rc/rc_crsf.cpp).
# Streams RC_CHANNELS_PACKED (0xC8 0x18 0x16 + 22 bytes of 16 x 11-bit + crc8
# DVB-S2 over type+payload) at the ELRS/Crossfire 250 Hz link rate. Sticks
# centred (992), throttle low (172), arm switch high on channel 5.
import os

CHANNELS = [992, 992, 172, 992, 1811, 172, 172, 172, 992, 992, 992, 992, 992, 992, 992, 992]

# ZP4_CRSF_FLTMODE_RAW: optional override for channel[7] (fltModeRaw --
# system_manager.hpp's SM_FLIGHTMODE*_MAX bands), so a test can select a
# closed-loop flight mode (FBWA, FLTMODE2's default) instead of the fixture's
# default MANUAL/passthrough. Unset leaves the original behavior unchanged.
if os.environ.get("ZP4_CRSF_FLTMODE_RAW"):
    CHANNELS[7] = int(os.environ["ZP4_CRSF_FLTMODE_RAW"])


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def rc_frame(ch):
    bits = 0
    for i, v in enumerate(ch):
        bits |= (v & 0x7FF) << (11 * i)
    payload = bits.to_bytes(22, "little")
    body = bytes([0x16]) + payload
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)])


class Peer:
    def __init__(self, ctx):
        self.ctx = ctx
        self.frame = rc_frame(CHANNELS)
        ctx.schedule_periodic(4000)  # 250 Hz

    def on_byte(self, b):
        pass  # ZeroPilot never talks to the receiver

    def on_timer(self):
        self.ctx.send(self.frame)


if __name__ == "__main__":
    f = rc_frame(CHANNELS)
    assert len(f) == 26 and f[0] == 0xC8 and f[1] == 0x18 and f[2] == 0x16
    assert crc8(f[2:25]) == f[25]
    print("ok", f.hex())
