# DroneCAN peer node on ZeroPilot's FDCAN1 (CAN.ScriptedCanNode).
#
# ZeroPilot's CANController is the bus allocator: it broadcasts NodeStatus at
# 1 Hz, tracks every other node's NodeStatus for liveness and answers anonymous
# uavcan.protocol.dynamic_node_id.Allocation requests (can_controller.cpp).
# This peer is one client of that: it runs the three-stage anonymous
# allocation handshake (6 + 6 + 4 unique-id bytes), takes the node id the
# allocator hands back, then heartbeats NodeStatus from that id so the
# allocator's canNodes[] table sees one live remote node.
#
# Frame layout (DroneCAN v0 / libcanard):
#   message id  = prio<<24 | type<<8 | src           (src 1..127)
#   anonymous   = prio<<24 | discr<<10 | (type&3)<<8 | 0
#   tail byte   = start<<7 | end<<6 | toggle<<5 | transfer_id
# Single-frame transfers carry no CRC; multi-frame ones open with a 16-bit CRC.

NODESTATUS_ID = 341
ALLOCATION_ID = 1
PRIO_LOW = 24
ALLOCATOR = 1              # CANController::NODE_ID = CANARD_MIN_NODE_ID
UNIQUE_ID = bytes(range(0x50, 0x60))
PREFERRED = 42
STAGES = (UNIQUE_ID[0:6], UNIQUE_ID[6:12], UNIQUE_ID[12:16])
BOOT_DELAY_US = 3_000_000  # firmware brings FDCAN1 up inside initDrivers()


def msg_id(dtype, src, prio=PRIO_LOW):
    return prio << 24 | dtype << 8 | src


def anon_id(dtype, discr=0x1234, prio=PRIO_LOW):
    return prio << 24 | discr << 10 | (dtype & 3) << 8


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.node_id = 0
        self.stage = 0
        self.tid = 0
        self.seen_status = 0
        self.ctx.schedule_oneshot(BOOT_DELAY_US)

    def tail(self):
        t = 0xC0 | (self.tid & 0x1F)
        self.tid += 1
        return t

    def on_timer(self):
        if self.node_id:
            # uptime u32 LE, health/mode/sub_mode packed byte (all 0 = OK/OPERATIONAL), vendor u16
            self.uptime += 1
            payload = self.uptime.to_bytes(4, "little") + b"\x00" + (0).to_bytes(2, "little")
            self.ctx.send(msg_id(NODESTATUS_ID, self.node_id), payload + bytes([self.tail()]), extended=True)
            return
        # allocation stage (re-)start: stage 0 always begins a fresh request
        self.stage = 0
        self.send_stage()
        self.ctx.schedule_oneshot(1_000_000)  # retry if the allocator stays silent

    def send_stage(self):
        first = 1 if self.stage == 0 else 0
        head = bytes([(PREFERRED << 1) | first])
        self.ctx.send(anon_id(ALLOCATION_ID), head + STAGES[self.stage] + bytes([self.tail()]), extended=True)
        self.ctx.info(f"allocation request stage {self.stage + 1}/3")

    def on_frame(self, frame):
        cid, data = frame["id"], bytes(frame["data"])
        src = cid & 0x7F
        dtype = (cid >> 8) & 0xFFFF
        if src == 0 or not data:
            return
        if dtype == NODESTATUS_ID:
            self.seen_status += 1
            if self.seen_status == 1:
                self.ctx.info(f"NodeStatus from node {src}: uptime {int.from_bytes(data[0:4], 'little')} s")
            return
        if dtype != ALLOCATION_ID or src != ALLOCATOR:
            return
        t = data[-1]
        if not t & 0x80:
            return  # continuation of a multi-frame response; the node id is in the first frame
        allocated = (data[0] if t & 0x40 else data[2]) >> 1
        if allocated:
            self.node_id = allocated
            self.uptime = 0
            self.ctx.info(f"allocated node id {allocated}; heartbeating NodeStatus at 1 Hz")
            self.ctx.schedule_periodic(1_000_000)
            return
        if self.stage < 2:
            self.stage += 1
            self.send_stage()


if __name__ == "__main__":
    assert msg_id(NODESTATUS_ID, 1) == 0x18015501
    assert anon_id(ALLOCATION_ID) == 0x18000000 | 0x1234 << 10 | 0x100
    assert sum(len(s) for s in STAGES) == 16
    print("ok")
