# INA228 power monitor on ZeroPilot's I2C1, 7-bit 0x45 (drivers/power_module).
# Register file with an 8-bit pointer. STM32F7_I2C hands over one NBYTES chunk
# per on_write and calls on_finish at STOP, so a HAL Mem_Write arrives as
# [reg] then [data...]: accumulate until the transaction ends, byte 0 is the
# pointer.
#
# Timing follows the firmware's own configuration: the conversion cycle is
# (enabled inputs' conversion times) x AVG from ADC_CONFIG, and the CURRENT /
# POWER / ENERGY / CHARGE scale comes from the SHUNT_CAL it writes. Results
# stay at their reset value 0 until the first cycle completes, CNVRF in
# DIAG_ALRT sets on each completed cycle, and ENERGY/CHARGE accumulate with
# virtual time. Source: TI INA228 datasheet SLYS021A (rev. May 2022).

# The board and the world it measures (user-tunable).
PACK_V = 16.8        # V, bus voltage (4S pack)
LOAD_A = 12.0        # A, current through the shunt
DIE_C = 35.0         # degC, die temperature
R_SHUNT = 0.0005     # ohm, ZeroPilot's shunt (the driver's RSHUNT)

# Chip timing, datasheet typ values.
CT_US = (50, 84, 150, 280, 540, 1052, 2074, 4120)  # us per VBUSCT/VSHCT/VTCT code: SLYS021A 6.5 (TCT) and Table 7-6
AVG_N = (1, 4, 16, 64, 128, 256, 512, 1024)        # samples per AVG code: SLYS021A Table 7-6
CONVDLY_STEP_US = 2000                             # CONFIG.CONVDLY step: SLYS021A Table 7-5

CONFIG, ADC_CONFIG, SHUNT_CAL, VBUS, DIETEMP, CURRENT, POWER, ENERGY, CHARGE, DIAG_ALRT = 0, 1, 2, 5, 6, 7, 8, 9, 10, 11
RESET = {CONFIG: 0x0000, ADC_CONFIG: 0xFB68, SHUNT_CAL: 0x1000}  # SLYS021A 7.6.1.1-7.6.1.3


def be(v, n):
    return (int(v) & ((1 << (8 * n)) - 1)).to_bytes(n, "big")


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.ptr = 0
        self.buf = b""
        self._reset()

    def _reset(self):
        self.regs = dict(RESET)
        self.t0 = self.ctx.now_us   # start of the current conversion run
        self.acc_us = 0             # ENERGY/CHARGE time carried over from earlier runs
        self.seen = 0               # cycles already reported through CNVRF

    def _cycle_us(self):
        a = self.regs[ADC_CONFIG]
        mode = a >> 12
        ct = [CT_US[(a >> 9) & 7], CT_US[(a >> 6) & 7], CT_US[(a >> 3) & 7]]  # bus, shunt, temp
        return sum(t for bit, t in zip((1, 2, 4), ct) if mode & bit) * AVG_N[a & 7]

    def _cycles(self):
        mode = self.regs[ADC_CONFIG] >> 12
        cyc = self._cycle_us()
        if mode & 7 == 0 or cyc == 0:          # shutdown
            return 0
        start = self.t0 + ((self.regs[CONFIG] >> 6) & 0xFF) * CONVDLY_STEP_US
        n = max(0, (self.ctx.now_us - start) // cyc)
        return n if mode & 8 else min(n, 1)    # triggered modes convert once

    def _current_lsb(self):
        adcrange = (self.regs[CONFIG] >> 4) & 1
        return (self.regs[SHUNT_CAL] & 0x7FFF) / (13107.2e6 * R_SHUNT * (4 if adcrange else 1))  # SLYS021A Eq. 2

    def _value(self, reg):
        n = self._cycles()
        if reg == DIAG_ALRT:
            cnvrf = n > self.seen
            self.seen = n                      # reading DIAG_ALRT clears CNVRF (SLYS021A 7.3.4)
            return be(0x0001 | cnvrf << 1, 2)  # MEMSTAT=1 (trim OK)
        if reg in RESET:
            return be(self.regs[reg], 2)
        if n == 0:                             # nothing converted yet: reset values
            return bytes({ENERGY: 5, CHARGE: 5, DIETEMP: 2}.get(reg, 3))
        lsb = self._current_lsb()
        if reg == VBUS:
            return be(int(PACK_V / 195.3125e-6) << 4, 3)   # SLYS021A 6.5, 1 LSB = 195.3125 uV
        if reg == DIETEMP:
            return be(int(DIE_C / 7.8125e-3), 2)           # 1 LSB = 7.8125 m degC
        if lsb == 0:                           # SHUNT_CAL = 0 -> current/power/energy/charge read 0 (SLYS021A 7.3.2)
            return bytes(5 if reg in (ENERGY, CHARGE) else 3)
        t_s = (self.acc_us + n * self._cycle_us()) * 1e-6
        if reg == CURRENT:
            return be(int(LOAD_A / lsb) << 4, 3)                     # Eq. 4
        if reg == POWER:
            return be(PACK_V * LOAD_A / (3.2 * lsb), 3)              # Eq. 5
        if reg == ENERGY:
            return be(PACK_V * LOAD_A * t_s / (16 * 3.2 * lsb), 5)   # Eq. 6
        if reg == CHARGE:
            return be(LOAD_A * t_s / lsb, 5)                         # Eq. 7
        return bytes(2)

    def on_write(self, data):
        self.buf += bytes(data)
        if not self.buf:
            return
        self.ptr = self.buf[0]
        if len(self.buf) < 3 or self.ptr not in RESET:
            return
        v = int.from_bytes(self.buf[1:3], "big")
        self.ctx.info(f"write reg 0x{self.ptr:02x} = 0x{v:04x}")
        if self.ptr == CONFIG and v & 0x8000:            # RST: same as power-on reset
            self._reset()
            return
        if self.ptr == CONFIG and v & 0x4000:            # RSTACC: accumulators restart from 0
            self.acc_us = -self._cycles() * self._cycle_us()
        if self.ptr == ADC_CONFIG:                       # writing MODE restarts conversions
            self.acc_us += self._cycles() * self._cycle_us()
            self.t0, self.seen = self.ctx.now_us, 0
        self.regs[self.ptr] = v & 0x7FFF if self.ptr == CONFIG else v  # RST self-clears

    def on_finish(self):
        self.buf = b""

    def on_read(self, count):
        self.buf = b""
        return (self._value(self.ptr) + bytes(count))[:count]
