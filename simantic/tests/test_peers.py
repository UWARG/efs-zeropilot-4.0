"""Basic peer checks: boot the plane image against the scripted devices in
../scenario.yaml and assert each one is being talked to.

    pip install pytest
    pytest simantic/tests            # needs `sim` on PATH and simantic/zp4-h753.elf

One 12 s run is shared by every test (~30 s wall). Expected values are the
constants at the top of each *_peer.py; change a peer and update the number here.
"""
import re
import struct
import subprocess
import sys
from collections import Counter
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from mavcrc import frames  # noqa: E402

HEARTBEAT, GPS_RAW_INT, RAW_IMU, SCALED_PRESSURE, ATTITUDE = 0, 24, 27, 29, 30
RC_CHANNELS, DISTANCE_SENSOR, BATTERY_STATUS = 65, 132, 147


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    elf = HERE / "zp4-h753.elf"
    if not elf.exists():
        pytest.skip("zp4-h753.elf missing: run simantic/build.sh <checkout> plane")
    out = tmp_path_factory.mktemp("sim")
    done = subprocess.run(
        ["sim", "--scenario", "scenario.yaml", "--timeout", "12",
         "--output", out / "out.txt", "--frames", out / "frames.txt"],
        cwd=HERE, check=True, capture_output=True, text=True, timeout=300)
    bus = Counter(re.findall(r"\((\w+)\) (?:SPI|I2C|CAN)", (out / "frames.txt").read_text()))
    return list(frames(out / "out.txt")), bus, done.stdout + done.stderr


def last(msgs, mid):
    p = [p for m, p, ok, _ in msgs if m == mid and ok][-1]
    return p.ljust(64, b"\0")  # MAVLink v2 trims trailing zero bytes


def test_mavlink_stream_is_crc_clean(run):
    msgs, _, _ = run
    assert msgs and all(ok for _, _, ok, _ in msgs)
    assert {m for m, *_ in msgs} >= {HEARTBEAT, GPS_RAW_INT, RAW_IMU, SCALED_PRESSURE,
                                     ATTITUDE, RC_CHANNELS, DISTANCE_SENSOR, BATTERY_STATUS}


def test_gps_peers_ack_the_ubx_config(run):
    _, _, log = run
    for peer in ("gpspeer1", "gpspeer2"):
        assert f"({peer}) CFG-VALSET" in log
    assert "-> ACK-ACK" in log


def test_gps_fix_reaches_mavlink(run):
    msgs, _, _ = run
    assert last(msgs, GPS_RAW_INT)[28] >= 3  # fix_type


def test_both_imus_are_polled_and_attitude_follows(run):
    msgs, bus, _ = run
    assert bus["imu0"] > 100 and bus["imu1"] > 100
    pitch_deg = struct.unpack_from("<f", last(msgs, ATTITUDE), 8)[0] * 57.2958
    assert -13 < pitch_deg < -7  # icm42688_peer tilts the board 10 deg nose-up


def test_baro_is_read_over_i2c(run):
    _, bus, _ = run
    assert bus["baro"] > 1000


@pytest.mark.xfail(strict=True, reason="sim 0.6.6: the baro peer serves PRESS_DATA over DMA but SCALED_PRESSURE.press_abs "
                   "is 0 / not a pressure (an earlier note saw 1004 hPa on 0.6.4); not yet traced")
def test_baro_reaches_scaled_pressure(run):
    msgs, _, _ = run
    assert 1000 < struct.unpack_from("<f", last(msgs, SCALED_PRESSURE), 4)[0] < 1008  # hPa


def test_rangefinder_and_power_module_are_read(run):
    msgs, bus, _ = run
    assert bus["rangefinder"] > 0 and bus["power"] > 0
    assert struct.unpack_from("<H", last(msgs, DISTANCE_SENSOR), 8)[0] == 1250  # cm


def test_dronecan_node_talks_both_ways(run):
    _, bus, _ = run
    assert bus["fdcan1"] > 0 and bus["canpeer"] > 0
