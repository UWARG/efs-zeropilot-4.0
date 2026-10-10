# Minimum-viable closed-loop plant for ZeroPilot 4.0 on the H753 (W4,
# simantic-dev/simantic-core#352). One instance (PLANT, below) lives for the
# whole sim process: `tim_actuator_peer.py` writes commanded servo/throttle
# pulse widths into it from live TIM register reads, and `icm42688_peer.py`
# reads its attitude/rate state back out for the IMU packets it builds. Both
# scripts run as CPython peripherals inside the same `sim` process (see
# CPythonEngine.cs), so a plain `import zp_plant` gives every peripheral the
# same module object -- no socket, no external process, no GPLv3 code (this
# is our own physics, not JSBSim).
#
# MODELED SIMPLIFICATION: 3-axis attitude kinematics with a first-order
# rate response to control-surface deflection, not a real 6DOF aero model
# (no lift/drag/mass/inertia coefficients). Good enough to show attitude
# responding to commanded outputs; a real aircraft model is follow-up work.
#
# Channel assignment matches ZeroPilot's actual MOTOR_MAP (plane build,
# boardfiles/model/src/drivers.cpp): tim_actuator_peer.py hands us channel 0
# = aileron (TIM1 CH1/PE9), 1 = elevator (TIM1 CH2/PE11), 2 = rudder (TIM1
# CH4/PE14), 3 = throttle (TIM1 CH3/PE13). TIM2's flap/ground-steering
# channels are read by tim_actuator_peer.py but not fed into this attitude
# model -- MODELED SIMPLIFICATION / remaining gap (no lift/drag state here
# for flap deflection to act on, and ground steering doesn't affect an
# in-flight attitude model).
import math
import os

PITCH_TRIM_DEG = 10.0  # matches the fixture's original static nose-up default
NEUTRAL_PULSE_US = 1500.0
PULSE_RANGE_US = 500.0  # 1000-2000us convention -> +-1.0 normalized
THROTTLE_MIN_PULSE_US = 1000.0
THROTTLE_RANGE_US = 1000.0  # 1000-2000us convention -> 0.0-1.0 normalized (unidirectional)

MAX_ROLL_RATE_DPS = 90.0
MAX_PITCH_RATE_DPS = 60.0
MAX_YAW_RATE_DPS = 45.0
RATE_TIME_CONSTANT_S = 0.3  # first-order lag from surface to body rate

# MODELED SIMPLIFICATION: control-surface authority scales with a throttle
# proxy for airspeed/dynamic pressure (a real airframe's control surfaces
# are mushy at idle and firm under power) -- no real airspeed/lift state,
# just a cheap linear term on the same normalized throttle command. 0.3 is
# "idle still has some prop-wash authority", not a fitted response constant.
THROTTLE_MIN_EFFECTIVENESS = 0.3

# The no-clamp resting state this replaces (ATTITUDE_LIMIT_DEG=80): the old
# clamp existed because a startup PWM floor (MotorControl::init() writes
# minCCR, i.e. hard-over, and starts the timer before the firmware's control
# task ever runs) would otherwise integrate into a runaway angle. The real
# fix is to not integrate that floor at all -- see tim_actuator_peer.py's
# control_running gate, which holds the plant still until the firmware's own
# control task is observed overwriting that floor with a live command.


def _normalize(pulse_us):
    if pulse_us is None:
        return 0.0
    v = (pulse_us - NEUTRAL_PULSE_US) / PULSE_RANGE_US
    return max(-1.0, min(1.0, v))


def _normalize_throttle(pulse_us):
    if pulse_us is None:
        return 0.0
    v = (pulse_us - THROTTLE_MIN_PULSE_US) / THROTTLE_RANGE_US
    return max(0.0, min(1.0, v))


class Plant:
    """Deterministic rigid-body-ish attitude model, stepped in virtual time."""

    def __init__(self, initial_roll_deg=0.0, initial_pitch_deg=PITCH_TRIM_DEG):
        self.channel_us = [None] * 8
        self.roll_deg = initial_roll_deg
        self.pitch_deg = initial_pitch_deg
        self.yaw_deg = 0.0
        self.p_dps = 0.0  # roll rate
        self.q_dps = 0.0  # pitch rate
        self.r_dps = 0.0  # yaw rate

    def set_channel(self, ch, pulse_us):
        if 0 <= ch < len(self.channel_us):
            self.channel_us[ch] = pulse_us

    def step(self, dt_s):
        if dt_s <= 0:
            return
        aileron = _normalize(self.channel_us[0])
        elevator = _normalize(self.channel_us[1])
        rudder = _normalize(self.channel_us[2])
        effectiveness = THROTTLE_MIN_EFFECTIVENESS + (1.0 - THROTTLE_MIN_EFFECTIVENESS) * _normalize_throttle(self.channel_us[3])

        target_p = aileron * MAX_ROLL_RATE_DPS * effectiveness
        target_q = elevator * MAX_PITCH_RATE_DPS * effectiveness
        target_r = rudder * MAX_YAW_RATE_DPS * effectiveness

        alpha = min(1.0, dt_s / RATE_TIME_CONSTANT_S)
        self.p_dps += (target_p - self.p_dps) * alpha
        self.q_dps += (target_q - self.q_dps) * alpha
        self.r_dps += (target_r - self.r_dps) * alpha

        self.roll_deg += self.p_dps * dt_s
        self.pitch_deg += self.q_dps * dt_s
        self.yaw_deg = (self.yaw_deg + self.r_dps * dt_s) % 360.0

    def gyro_dps(self):
        return (self.p_dps, self.q_dps, self.r_dps)

    def accel_g(self):
        # Gravity vector rotated into the body frame by roll/pitch (level
        # flight, no linear acceleration modeled): MODELED SIMPLIFICATION.
        # Matches icm42688_peer.py's pre-existing X/Z convention exactly at
        # roll=0 (X=sin(pitch), Z=cos(pitch); the driver negates both, so a
        # positive PITCH_TRIM_DEG here still reports as nose-up), extended to
        # Y for roll with the same sign convention.
        phi = math.radians(self.roll_deg)
        theta = math.radians(self.pitch_deg)
        return (math.sin(theta), math.sin(phi) * math.cos(theta), math.cos(phi) * math.cos(theta))


# ZP4_PLANT_INITIAL_ROLL_DEG: test-injected initial disturbance (e.g. a gust
# that rolled the airframe before the sim starts recording), not a hardware
# constant -- used by the closed-loop causal-chain evidence run to give FBWA
# something to correct. Unset (default 0.0) reproduces the original resting
# trim exactly.
#
# Backend selection (simantic-dev/simantic-core#352, JSBSim swap): PLANT is a
# real 6DOF flight dynamics model (zp_jsbsim_plant.JSBSimPlant) when the
# `jsbsim` pip package is importable, falling back to the 3-axis kinematic
# Plant above when it is not installed -- this module's whole model exists
# so a jsbsim-less environment (or a unit test of the gate logic below,
# which cares about tim_actuator_peer.py's control_running detection, not
# aircraft physics) still works. ZP4_PLANT_BACKEND=kinematic forces the
# fallback even when jsbsim is installed.
def _make_plant(initial_roll_deg):
    if os.environ.get("ZP4_PLANT_BACKEND") != "kinematic":
        try:
            import jsbsim  # noqa: F401
        except ImportError:
            pass
        else:
            from zp_jsbsim_plant import JSBSimPlant
            return JSBSimPlant(initial_roll_deg=initial_roll_deg)
    return Plant(initial_roll_deg=initial_roll_deg)


PLANT = _make_plant(float(os.environ.get("ZP4_PLANT_INITIAL_ROLL_DEG", "0.0")))


def demo():
    """Self-check: a sustained elevator command must move pitch in the
    expected direction and keep climbing at full throttle (no clamp), while
    idle throttle should cut the response roughly to
    THROTTLE_MIN_EFFECTIVENESS. Run: python3 zp_plant.py"""
    p = Plant()
    p.set_channel(1, 2000.0)  # full-forward elevator pulse
    p.set_channel(3, 2000.0)  # full throttle -> full control effectiveness
    for _ in range(500):
        p.step(0.01)  # 5s at 100Hz
    assert abs(p.q_dps - MAX_PITCH_RATE_DPS) < 1.0, p.q_dps
    assert p.pitch_deg > PITCH_TRIM_DEG + 200, p.pitch_deg  # no clamp: keeps climbing
    gyro = p.gyro_dps()
    assert abs(gyro[1] - p.q_dps) < 1e-9

    p_idle = Plant()
    p_idle.set_channel(1, 2000.0)
    p_idle.set_channel(3, 1000.0)  # idle throttle -> reduced authority
    for _ in range(500):
        p_idle.step(0.01)
    assert abs(p_idle.q_dps - MAX_PITCH_RATE_DPS * THROTTLE_MIN_EFFECTIVENESS) < 1.0, p_idle.q_dps

    p2 = Plant()
    assert p2.gyro_dps() == (0.0, 0.0, 0.0)
    ax, ay, az = p2.accel_g()
    assert abs(ax - math.sin(math.radians(PITCH_TRIM_DEG))) < 1e-9
    assert abs(ay - 0.0) < 1e-9
    assert abs(az - math.cos(math.radians(PITCH_TRIM_DEG))) < 1e-9

    p3 = Plant()
    assert p3.channel_us == [None] * 8
    p3.set_channel(0, 1500.0)
    assert _normalize(p3.channel_us[0]) == 0.0

    p4 = Plant(initial_roll_deg=15.0)
    assert p4.roll_deg == 15.0
    print("ok")


if __name__ == "__main__":
    demo()
