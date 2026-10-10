# TDK ICP-20100 barometer on ZeroPilot's I2C2, 7-bit 0x63
# (drivers/barometer/icp_20100.cpp). Register file, 8-bit pointer. Reports
# revision B (VERSION 0xB2) so init() goes straight to the FIR warm-up.
# Data: 20-bit two's complement, P = raw*40/2^17 + 70 kPa, T = raw*65/2^18 + 25 C.
# STM32F7_I2C hands over one NBYTES chunk per on_write and calls on_finish at
# STOP, so a HAL Mem_Write arrives as [reg] then [data]; accumulate until the
# transaction ends, byte 0 is the pointer.
#
# Timing: while MODE_SELECT.MEAS_MODE=1 the part writes one pressure/temperature
# pair to its FIFO every 1/ODR, ODR set by the MEAS_CONFIG mode the firmware
# writes. A full FIFO drops new pairs; an empty one reads 0x00.
# Source: TDK ICP-20100 datasheet DS-000416 rev 1.3.
#
# P_KPA/T_C are the fixed fallback world (~80 m ASL, room temperature) used
# when PLANT has no live pressure/temperature (zp_plant.Plant, the kinematic
# model). zp_jsbsim_plant.JSBSimPlant's pressure_kpa/temp_c (from the model's
# own atmosphere at its current altitude) are read live instead when present
# (simantic-dev/simantic-core#352, W4).
from zp_plant import PLANT

P_KPA, T_C = 100.4, 22.0  # the world: ~80 m ASL, room temperature

MODE_ODR_HZ = (25, 120, 40, 2, 25)  # MEAS_CONFIG mode 0..4: DS-000416 2.2 Table 2 (mode 4 = factory default config)
FIFO_DEPTH = 16                     # pairs: DS-000416 section 7
# MODELED SIMPLIFICATION: the first pair lands one ODR period after MEAS_MODE
# goes to 1; the datasheet gives no separate first-sample latency.

VERSION, DEVICE_STATUS, MODE_SELECT, FIFO_FILL, PRESS_DATA = 0xD3, 0xCD, 0xC0, 0xC4, 0xFA


def raw20(v):
    return v & 0xFFFFF


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.ptr = 0
        self.buf = b""
        self.mode = 0
        self.fill = 0
        self.t0 = 0      # start of the current measurement run
        self.ticks = 0   # ODR ticks of this run already pushed into the FIFO

    def _sample(self):
        p_kpa = getattr(PLANT, "pressure_kpa", P_KPA)
        t_c = getattr(PLANT, "temp_c", T_C)
        p = raw20(int((p_kpa - 70.0) * 131072 / 40.0))
        t = raw20(int((t_c - 25.0) * 262144 / 65.0))
        return bytes([p & 0xFF, (p >> 8) & 0xFF, (p >> 16) & 0x0F,
                      t & 0xFF, (t >> 8) & 0xFF, (t >> 16) & 0x0F])

    def _advance(self):
        # Bring the FIFO up to now: one pair per elapsed ODR tick, none past full.
        if not self.mode & 0x08:
            return
        period_us = 1_000_000 // MODE_ODR_HZ[min(self.mode >> 5, 4)]
        ticks = (self.ctx.now_us - self.t0) // period_us
        self.fill = min(self.fill + ticks - self.ticks, FIFO_DEPTH)
        self.ticks = ticks

    def on_write(self, data):
        self.buf += bytes(data)
        if not self.buf:
            return
        self.ptr = self.buf[0]
        if len(self.buf) < 2:
            return
        v = self.buf[1]
        self._advance()
        if self.ptr == MODE_SELECT:
            if v & 0x08 and not self.mode & 0x08:
                self.t0, self.ticks = self.ctx.now_us, 0
            self.mode = v
            self.ctx.info(f"MODE_SELECT=0x{v:02x} ({f'mode {v >> 5}, measuring' if v & 0x08 else 'standby'})")
        elif self.ptr == FIFO_FILL and v & 0x80:
            self.fill = 0

    def on_finish(self):
        self.buf = b""

    def on_read(self, count):
        self.buf = b""
        self._advance()
        r = self.ptr
        if r == VERSION:
            out = b"\xB2"
        elif r == DEVICE_STATUS:
            out = b"\x01"                          # MODE_SYNC: MODE_SELECT writable
        elif r == FIFO_FILL:                       # DS-000416 13.24: EMPTY bit 6, FULL bit 5, level 4:0
            out = bytes([self.fill | (0x40 if self.fill == 0 else 0) | (0x20 if self.fill == FIFO_DEPTH else 0)])
        elif r == PRESS_DATA:                      # each 6-byte pair read pops one FIFO entry (DS-000416 6.7)
            out = b""
            for _ in range((count + 5) // 6):
                out += self._sample() if self.fill else bytes(6)
                self.fill = max(self.fill - 1, 0)
        else:
            out = b"\x00"
        return (out + bytes(count))[:count]
