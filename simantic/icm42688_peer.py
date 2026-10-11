# ICM-42688-P on ZeroPilot's SPI1 (drivers/imu/imu.cpp). The board has two,
# CS PC4 and PC5; scenario.yaml lists one `spi-device` part per chip select
# with this script, and the sim puts an SPI multiplexer in front of them.
# Framing: the multiplexer calls on_finish when the part's CS goes high.
# Beat 0 is the register byte (bit7 = read), beats 1.. are data.
#
# Timing: once PWR_MGMT0 turns gyro and accel on, the part produces one
# accel+gyro sample every 1/ODR, ODR from the GYRO_CONFIG0 code the firmware
# writes (ACCEL_CONFIG0 carries the same code on ZeroPilot). In stream-to-FIFO
# mode each sample becomes a 16-byte packet; FIFO_COUNT (in records, the
# firmware's INTF_CONFIG0 setting) and FIFO_DATA follow those ticks, and each
# packet's 16-bit timestamp is the sample time in 1 us units.
# Source: TDK ICM-42688-P datasheet DS-000347 rev 1.2.
import math
import struct

# The board sits stationary, nose up PITCH_TRIM_DEG. The driver negates X and
# Z; with these signs the firmware reports roll 0, pitch -10 deg.
PITCH_TRIM_DEG = 10.0

ODR_HZ = {1: 32000, 2: 16000, 3: 8000, 4: 4000, 5: 2000, 6: 1000, 7: 200, 8: 100,
          9: 50, 10: 25, 11: 12.5, 15: 500}  # GYRO_ODR code: DS-000347 5.6 (reset code 6 = 1 kHz)
STARTUP_US = 30_000     # gyro start-up, enable -> drive ready, typ: DS-000347 Table 1 (accel is 10 ms, Table 2)
FIFO_PACKETS = (2048 - 16) // 16  # 2 KB FIFO less one packet reserved: DS-000347 6.3
# MODELED SIMPLIFICATION: a full stream-mode FIFO keeps the newest packets
# (DS-000347 rev 1.2 does not state which end is dropped).

BANK_SEL, WHO_AM_I, DEVICE_CONFIG, GYRO_DATA_X1 = 0x76, 0x75, 0x11, 0x25
FIFO_CONFIG, FIFO_COUNTH, FIFO_DATA, SIGNAL_PATH_RESET = 0x16, 0x2E, 0x30, 0x4B
PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0 = 0x4E, 0x4F, 0x50


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.beat = 0
        self.reg = 0
        self.read = False
        self.out = b""
        self._reset()

    def _reset(self):
        self.bank = 0
        self.regs = {GYRO_CONFIG0: 0x06, ACCEL_CONFIG0: 0x06}  # bank-0 reset values: DS-000347 section 14
        self.t_on = None   # time of the first sample; None while sensors are off
        self.next = 0      # index of the next sample the FIFO hands out

    def _period_us(self):
        return 1e6 / ODR_HZ.get(self.regs[GYRO_CONFIG0] & 0x0F, 1000)

    def _produced(self):
        if self.t_on is None or self.ctx.now_us < self.t_on:
            return 0
        return int((self.ctx.now_us - self.t_on) // self._period_us()) + 1

    def _fifo_count(self):
        if not self.regs.get(FIFO_CONFIG, 0) & 0xC0:   # bypass mode: FIFO unused
            return 0
        n = self._produced()
        self.next = max(self.next, n - FIFO_PACKETS)
        return n - self.next

    def _gyro_raw(self):
        lsb_dps = 16.4 * (1 << ((self.regs[GYRO_CONFIG0] >> 5) & 7))  # GYRO_FS_SEL, 16.4 LSB/dps at +-2000: DS-000347 Table 1, 5.6
        return tuple(int(g * lsb_dps) for g in (0.0, 0.0, 0.0))  # stationary

    def _packet(self):
        if self._fifo_count() == 0:
            return bytes([0x80]) + bytes(15)   # HEADER_MSG: FIFO empty (DS-000347 6.2)
        ts = int(self.t_on + self.next * self._period_us())
        self.next += 1
        lsb_g = 2048 << ((self.regs[ACCEL_CONFIG0] >> 5) & 3)   # ACCEL_FS_SEL: +-16/8/4/2 g (DS-000347 5.6)
        theta = math.radians(PITCH_TRIM_DEG)
        ax, ay, az = math.sin(theta), 0.0, math.cos(theta)  # gravity in the body frame, g
        accel = (int(lsb_g * ax), int(lsb_g * ay), int(lsb_g * az))
        gyro = self._gyro_raw()
        # Packet 3: header accel|gyro|ODR timestamp (0x68), accel, gyro, temp, timestamp (DS-000347 6.1)
        return bytes([0x68]) + struct.pack(">6h", *accel, *gyro) + b"\x00" + struct.pack(">H", ts & 0xFFFF)

    def on_transmit(self, b):
        i = self.beat
        self.beat += 1
        if i == 0:
            self.read = bool(b & 0x80)
            self.reg = b & 0x7F
            if self.read:
                self.out = self._read(self.reg)
            return 0x00
        if self.read:
            if self.reg == FIFO_DATA and self.bank == 0:   # packets pop as they are clocked out
                if (i - 1) % 16 == 0:
                    self.out = self._packet()
                return self.out[(i - 1) % 16]
            return self.out[i - 1] if i - 1 < len(self.out) else 0x00
        self._write(self.reg + i - 1, b)
        return 0x00

    def _write(self, reg, v):
        if reg == BANK_SEL:
            self.bank = v & 7
            return
        if self.bank != 0:
            return
        if reg == DEVICE_CONFIG and v & 0x01:          # soft reset
            self._reset()
            return
        if reg == SIGNAL_PATH_RESET and v & 0x02:      # FIFO_FLUSH
            self.next = self._produced()
        elif reg == PWR_MGMT0:                         # GYRO_MODE 11 = LN, ACCEL_MODE 1x = LP/LN
            on = (v >> 2) & 3 == 3 and v & 3 >= 2
            if not on:
                self.t_on = None
            elif self.t_on is None:
                self.t_on, self.next = self.ctx.now_us + STARTUP_US, 0
        self.regs[reg] = v

    def _read(self, reg):
        if reg == BANK_SEL:
            return bytes([self.bank])
        if reg == WHO_AM_I:
            return b"\x47"
        if reg == GYRO_DATA_X1:
            return struct.pack(">hhh", *self._gyro_raw())
        if reg == FIFO_COUNTH:
            return struct.pack(">H", self._fifo_count())   # records, big endian (INTF_CONFIG0 = 0xF0)
        if reg == FIFO_DATA:
            return b""
        return bytes([self.regs.get(reg, 0x00)]) * 2

    def on_finish(self):
        self.beat = 0
