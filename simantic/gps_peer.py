# A u-blox M9/M10-class UBX GPS receiver as a scripted UART peer, for
# WARG ZeroPilot 4.0's `GPS::configureUBX()` (stm32h753iitx/boardfiles/
# drivers/gps/gps.{cpp,hpp}).
#
# Framing: sync 0xB5 0x62, class(1), id(1), length LE16, payload, then an
# 8-bit Fletcher checksum (ckA/ckB) over class+id+length+payload -- this is
# GPS::calcChecksum()/verifyChecksumUBX() byte for byte. Known-good vectors
# from the driver's own traffic (see tests/warg/zp4-h753/first-light.txt):
#   CFG-VALSET  06 8A 09 00 00 01 00 00 07 00 91 20 01 -> ckA=53 ckB=48
#   CFG-MSG     06 01 03 00 01 07 01                   -> ckA=13 ckB=51
#
# Modeling choice: ACK CFG-VALSET (the M9/M10 path). ZeroPilot's
# configureUBX() tries CFG-VALSET first and only falls back to the legacy
# CFG-MSG path if it goes un-ACKed, so a modern receiver on the other end of
# the wire answers CFG-VALSET and the legacy path is never exercised. The
# legacy CFG-MSG/CFG-RATE handling below exists anyway (cheap, and it is
# exactly what the driver would need from an M8-class receiver), so this
# peer is a plausible stand-in for either generation of receiver.
#
# Timing is NOT a fitted constant: the driver's own CFG-VALSET writes for
# CFG-RATE-MEAS/CFG-RATE-NAV (and, on the legacy path, its own CFG-RATE
# message) carry the requested measurement interval and nav-solution ratio.
# This peer reads those values back out of the frames it is ACKing and
# starts NAV-PVT at exactly the rate the firmware asked for -- it never
# assumes "5 Hz" on its own.

import struct

UBX_SYNC1, UBX_SYNC2 = 0xB5, 0x62

CLASS_NAV = 0x01
CLASS_ACK = 0x05
CLASS_CFG = 0x06

ID_NAV_PVT = 0x07
ID_ACK_ACK = 0x01
ID_CFG_MSG = 0x01
ID_CFG_RATE = 0x08
ID_CFG_VALSET = 0x8A

# CFG-VALSET keys the driver writes (gps.cpp), value width from bits 30:28.
KEY_RATE_MEAS = 0x30210001  # U2
KEY_RATE_NAV = 0x30210002  # U2
KEY_MSGOUT_NAV_PVT_UART1 = 0x20910007  # U1

# u-blox receiver default until CFG-RATE/CFG-VALSET RATE_MEAS says otherwise.
DEFAULT_RATE_MEAS_MS = 1000
DEFAULT_RATE_NAV = 1

FIX_TYPE_3D = 3
GNSS_FIX_OK = 0x01

# A plausible static 3D fix (WARG's test field, Waterloo, ON); velocity is zero.
LAT_DEG = 43.4723
LON_DEG = -80.5449
ALT_M = 334.0
NUM_SV = 11


def checksum(body: bytes) -> bytes:
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def frame(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    body = bytes((msg_class, msg_id)) + struct.pack("<H", len(payload)) + payload
    return bytes((UBX_SYNC1, UBX_SYNC2)) + body + checksum(body)


def ack_ack(msg_class: int, msg_id: int) -> bytes:
    return frame(CLASS_ACK, ID_ACK_ACK, bytes((msg_class, msg_id)))


def nav_pvt(itow_ms: int) -> bytes:
    # Exactly the 92-byte UBX-NAV-PVT layout GPS::parsePVT() reads:
    # iTOW(4) year(2) month day hour min sec valid(1) tAcc(4) nano(4)
    # fixType flags flags2 numSV(1 each) lon lat height hMSL(4 each)
    # hAcc vAcc(4 each) velN velE velD(4 each) gSpeed(4) headMot(4) ...
    payload = bytearray(92)
    struct.pack_into("<I", payload, 0, itow_ms & 0xFFFFFFFF)
    # UTC time: fully resolved (validDate|validTime|fullyResolved = 0x07).
    struct.pack_into("<HBBBBB", payload, 4, 2026, 9, 15, 12, 0, 0)
    payload[11] = 0x07
    # tAcc(12), nano(16) left at 0.
    payload[20] = FIX_TYPE_3D
    payload[21] = GNSS_FIX_OK
    payload[23] = NUM_SV
    struct.pack_into("<iiii", payload, 24,
                      round(LON_DEG * 1e7), round(LAT_DEG * 1e7),
                      round(ALT_M * 1000), round(ALT_M * 1000))  # height == hMSL
    # hAcc/vAcc(40,44) and velN/E/D/gSpeed/headMot left at 0.
    return frame(CLASS_NAV, ID_NAV_PVT, bytes(payload))


class Peer:
    def __init__(self, ctx):
        self.ctx = ctx
        self.rx = bytearray()
        self.itow_ms = 0
        self.pvt_enabled = False
        self.rate_meas_ms = None
        self.rate_nav = None
        self.streaming = False

    def on_byte(self, b):
        self.rx.append(b)
        while True:
            while len(self.rx) >= 2 and not (self.rx[0] == UBX_SYNC1 and self.rx[1] == UBX_SYNC2):
                del self.rx[0]
            if len(self.rx) < 8:
                return
            length = self.rx[4] | (self.rx[5] << 8)
            total = 6 + length + 2
            if len(self.rx) < total:
                return
            raw = bytes(self.rx[:total])
            del self.rx[:total]
            self._handle_frame(raw)

    def _handle_frame(self, raw: bytes):
        body = raw[2:-2]  # class, id, len(2), payload -- what the checksum covers
        if checksum(body) != raw[-2:]:
            self.ctx.warning(f"bad checksum, dropping frame: {raw.hex()}")
            return

        msg_class, msg_id = body[0], body[1]
        payload = body[4:]

        if msg_class == CLASS_CFG and msg_id == ID_CFG_VALSET:
            self._handle_valset(payload)
        elif msg_class == CLASS_CFG and msg_id == ID_CFG_MSG:
            self._handle_cfg_msg(payload)
        elif msg_class == CLASS_CFG and msg_id == ID_CFG_RATE:
            self._handle_cfg_rate(payload)
        else:
            self.ctx.warning(f"unhandled frame class=0x{msg_class:02x} id=0x{msg_id:02x}")

    def _handle_valset(self, payload: bytes):
        # version(1) layers(1) reserved(2) key(4) value(<=4)
        key = struct.unpack_from("<I", payload, 4)[0]
        value_bytes = payload[8:]
        value = int.from_bytes(value_bytes, "little") if value_bytes else None

        self.ctx.send(ack_ack(CLASS_CFG, ID_CFG_VALSET))

        if key == KEY_MSGOUT_NAV_PVT_UART1:
            self.pvt_enabled = value == 1
            self.ctx.info(f"CFG-VALSET MSGOUT-NAV_PVT_UART1={value} -> ACK-ACK")
        elif key == KEY_RATE_MEAS:
            self.rate_meas_ms = value
            self.ctx.info(f"CFG-VALSET RATE_MEAS={value}ms -> ACK-ACK")
            self._maybe_start_streaming()
        elif key == KEY_RATE_NAV:
            self.rate_nav = value
            self.ctx.info(f"CFG-VALSET RATE_NAV={value} -> ACK-ACK")
            self._maybe_start_streaming()
        else:
            self.ctx.info(f"CFG-VALSET key=0x{key:08x} value={value} -> ACK-ACK")

    def _handle_cfg_msg(self, payload: bytes):
        sub_class, sub_id, rate = payload[0], payload[1], payload[2]
        self.ctx.send(ack_ack(CLASS_CFG, ID_CFG_MSG))
        self.ctx.info(f"CFG-MSG class=0x{sub_class:02x} id=0x{sub_id:02x} rate={rate} -> ACK-ACK")
        if sub_class == CLASS_NAV and sub_id == ID_NAV_PVT:
            self.pvt_enabled = rate > 0
            self._maybe_start_streaming()

    def _handle_cfg_rate(self, payload: bytes):
        meas_rate_ms, nav_rate = struct.unpack_from("<HH", payload, 0)
        self.ctx.send(ack_ack(CLASS_CFG, ID_CFG_RATE))
        self.ctx.info(f"CFG-RATE measRate={meas_rate_ms}ms navRate={nav_rate} -> ACK-ACK")
        self.rate_meas_ms = meas_rate_ms
        self.rate_nav = nav_rate
        self._maybe_start_streaming()

    def _maybe_start_streaming(self):
        if self.streaming or not self.pvt_enabled:
            return
        # Start once both halves of the nav-solution period are known, or
        # fall back to the u-blox power-on default if the driver never
        # overrides them (it always does, on this driver's configureUBX(),
        # but a legacy receiver that skips CFG-RATE would still stream).
        meas_ms = self.rate_meas_ms if self.rate_meas_ms is not None else DEFAULT_RATE_MEAS_MS
        nav = self.rate_nav if self.rate_nav is not None else DEFAULT_RATE_NAV
        self.streaming = True
        period_us = meas_ms * nav * 1000
        self.ctx.info(f"NAV-PVT configured, streaming every {meas_ms * nav}ms "
                       f"(measRate={meas_ms}ms x navRate={nav}, as programmed by the driver)")
        self.ctx.schedule_periodic(period_us)

    def on_timer(self):
        self.itow_ms += self.rate_meas_ms if self.rate_meas_ms else DEFAULT_RATE_MEAS_MS
        self.ctx.send(nav_pvt(self.itow_ms))
