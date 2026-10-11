# Benewake TF02-Pro LiDAR on ZeroPilot's I2C3, 7-bit 0x10
# (drivers/rangefinder/tf02pro.cpp). Command frame 5A len id ... cksum; the
# reply to the last command is served on the next read. Distance in cm.
#
# Timing: the LiDAR measures once per frame at FRAME_RATE_HZ; "obtain data
# frame" (5A 05 00 01 60) returns the latest completed measurement, so
# scene() is sampled at frame boundaries, not at the poll. ZeroPilot never
# sends the frame-rate command (0x03), so the rate stays at the default.
# Source: Benewake TF02-Pro User Manual (REV 01/04/2024), Table 1-1
# and Table 4-1.
FRAME_RATE_HZ = 100  # Hz, default frame rate: TF02-Pro User Manual Table 1-1 (note 2) and Table 4-1
STRENGTH = 3000
TEMP_RAW = (25 + 256) * 8


def scene(t_us):
    # The world: distance in cm at virtual time t_us. Edit to move the target.
    return 1250


def frame(payload):
    body = bytes([0x5A, len(payload) + 3]) + bytes(payload)
    return body + bytes([sum(body) & 0xFF])


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.reply = b""

    def on_write(self, data):
        if len(data) < 3 or data[0] != 0x5A:
            return
        cmd = data[2]
        if cmd == 0x01:                       # firmware version
            self.reply = frame([0x01, 0x02, 0x03, 0x01])
        elif cmd == 0x05:                     # output format -> echo
            self.reply = bytes(data)
        elif cmd == 0x11:                     # save config -> status 0
            self.reply = frame([0x11, 0x00])
        elif cmd == 0x00:                     # trigger a 9-byte data frame
            period_us = 1_000_000 // FRAME_RATE_HZ
            d = scene(self.ctx.now_us // period_us * period_us).to_bytes(2, "little")
            s = STRENGTH.to_bytes(2, "little")
            t = TEMP_RAW.to_bytes(2, "little")
            body = bytes([0x59, 0x59]) + d + s + t
            self.reply = body + bytes([sum(body) & 0xFF])
        else:
            self.ctx.warning(f"unhandled cmd {bytes(data).hex()}")

    def on_read(self, count):
        return (self.reply + bytes(count))[:count]
