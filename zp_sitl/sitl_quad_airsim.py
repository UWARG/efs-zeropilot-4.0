import airsim
import time
import threading
import os
import pygame
import sys
import zeropilot
import math


# SITL Scheduling Rate Hz
SITL_RATE_HZ = 1000

FLTMODE_AXIS = 6
# The 6 positions of fltmode buttons on the controller connected to sitl
FLTMODE_AXIS_VALUES = [-0.8, -0.38, -0.12, 0.0, 0.29, 0.99]

PA_TO_KPA = 0.001
NS_TO_S = 1e-9
# AirSim's barometer sensor doesn't report temperature, so assume a fixed ambient.
BARO_AMBIENT_TEMP_C = 25.0

# Every AirSim call is a blocking RPC round trip, so polling all of them each step is what keeps the loop
# from reaching SITL_RATE_HZ. Sample the slow sensors at their real rates instead of once per step.
BARO_RATE_HZ = 50
RANGEFINDER_RATE_HZ = 100

RC_DEADZONE = 0.05

# Neutral RC value
RC_CENTRE = 50

class ZP_QUAD_SITL_AIRSIM:
    def __init__(self, ip="127.0.0.1", port=14550):
        # Input Setup (Joysticks)
        pygame.init()
        pygame.joystick.init()
        self.joy = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
        if self.joy: self.joy.init()

        # AirSim setup
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        # State setup 
        self.zp = zeropilot.ZeroPilot(sitl_rate_hz=SITL_RATE_HZ, ip=ip, port=port)
        self.running = True
        self.armed = False
        self.paused = True 
        self.reset_requested = False
        self.commands = {'roll': RC_CENTRE, 'pitch': RC_CENTRE, 'yaw': RC_CENTRE, 'throttle': 0}
        self.fltmode_setpoints = [16.5, 29.5, 42.5, 55.5, 68.5, 81.5]

        # Cached slow-sensor samples and the dashboard's copy of the last step's state
        self.baro = self.client.getBarometerData()
        self.alt_rangefinder = self.client.getDistanceSensorData().distance
        self.next_baro_t = 0.0
        self.next_rangefinder_t = 0.0
        self.last_state = None
        self.achieved_rate_hz = 0.0
        self._rate_window_t = time.perf_counter()
        self._rate_window_steps = 0

        # Get fltmode on startup
        if self.joy:
            pygame.event.pump()
            self.fltmode_index = self.axis_to_fltmode(self.joy.get_axis(FLTMODE_AXIS))
        else:
            self.fltmode_index = 0

        print("initialized")

    def update_joystick(self):
        while self.running:
            pygame.event.pump()
            if self.joy:
                # Mapping: 2:Roll, 3:Pitch, 0:Yaw, 1:Throttle
                # Throttle is left raw, althold applies its own THROTTLE_DEADZONE and stabilize wants the full range
                self.commands['roll'] = (self.apply_deadzone(self.joy.get_axis(0)) + 1) * 50
                self.commands['pitch'] = (self.apply_deadzone(self.joy.get_axis(1)) + 1) * 50
                self.commands['yaw'] = (self.apply_deadzone(self.joy.get_axis(3)) + 1) * 50
                self.commands['throttle'] = (self.joy.get_axis(2) + 1) * 50

                if self.joy.get_axis(4) > 0.5: 
                    self.armed = True
                elif self.joy.get_axis(4) < 0.5:
                    self.armed = False
                
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 2:
                            self.reset_requested = True
                        if event.button == 3:
                            self.paused = not self.paused
                    elif event.type == pygame.JOYAXISMOTION:
                        if event.axis == FLTMODE_AXIS:
                            self.fltmode_index = self.axis_to_fltmode(event.value)

            time.sleep(0.01)

    def axis_to_fltmode(self, value):
        return min(range(len(FLTMODE_AXIS_VALUES)), key=lambda i: abs(FLTMODE_AXIS_VALUES[i] - value))

    @staticmethod
    def apply_deadzone(v, dz=RC_DEADZONE):
        if abs(v) < dz:
            return 0.0
        return (v - math.copysign(dz, v)) / (1.0 - dz)

    @staticmethod
    def world_to_body(q, vx, vy, vz):
        """Rotate a world-frame (NED) vector into the body frame.

        AirSim's orientation quaternion maps body -> world, so its conjugate maps
        world -> body. Applies v' = v + 2w(u x v) + 2u x (u x v) with u = (x, y, z).
        """
        w, x, y, z = q.w_val, -q.x_val, -q.y_val, -q.z_val
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (vx + w * tx + (y * tz - z * ty),
                vy + w * ty + (z * tx - x * tz),
                vz + w * tz + (x * ty - y * tx))
    
    def reset_to_air(self):
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        time.sleep(1)
        # The sim clock restarts, so the sensor schedules would otherwise sit in the future forever
        self.next_baro_t = 0.0
        self.next_rangefinder_t = 0.0

    def step(self):
        state = self.client.getMultirotorState()

        # Attitude
        q = state.kinematics_estimated.orientation
        pitch, roll, yaw = airsim.to_eularian_angles(q)

        # angular velocities (p,q,r)
        ang_vel = state.kinematics_estimated.angular_velocity
        p_rad = ang_vel.x_val
        q_rad = ang_vel.y_val
        r_rad = ang_vel.z_val

        # Linear acceleration, NED and gravity-free
        la = state.kinematics_estimated.linear_acceleration
        ax_body, ay_body, az_body = self.world_to_body(q, la.x_val, la.y_val, la.z_val)

        # latitude, longtitude, altitude from gps
        lat_deg = state.gps_location.latitude
        long_deg = state.gps_location.longitude
        alt_deg = state.gps_location.altitude

        lv = state.kinematics_estimated.linear_velocity
        ground_speed = math.sqrt(lv.x_val**2 + lv.y_val**2)  # horizontal only, m/s

        heading = math.degrees(yaw) % 360

        # AirSim stamps its state with the simulation clock. Hand it to the wrapper so the firmware's dt is
        # the time the sim really advanced, rather than the 1/SITL_RATE_HZ the loop cannot actually sustain.
        sim_time_s = state.timestamp * NS_TO_S
        self.zp.set_plant_time(sim_time_s)

        # Slow sensors: poll at their own rates so their RPCs don't throttle the control loop
        if sim_time_s >= self.next_baro_t:
            self.baro = self.client.getBarometerData()
            self.next_baro_t = sim_time_s + 1.0 / BARO_RATE_HZ
        if sim_time_s >= self.next_rangefinder_t:
            self.alt_rangefinder = self.client.getDistanceSensorData().distance
            self.next_rangefinder_t = sim_time_s + 1.0 / RANGEFINDER_RATE_HZ

        baro = self.baro
        alt_rangefinder = self.alt_rangefinder
        self.last_state = state

        self.zp.update_from_plant(
            roll, pitch,
            p_rad, q_rad, r_rad,
            ax_body, ay_body, az_body,
            lat_deg, long_deg, alt_deg,
            ground_speed,
            heading,
            lv.x_val, lv.y_val, lv.z_val,
            0.0,
            0.0,
            alt_rangefinder,
            baro.pressure * PA_TO_KPA,
            BARO_AMBIENT_TEMP_C
        )

        self.zp.set_rc(self.commands['roll'], self.commands['pitch'], self.commands['yaw'],
                self.commands['throttle'], 100 if self.armed else 0, self.fltmode_setpoints[self.fltmode_index])
        self.zp.update()

        if not self.paused:
            m1, m2, m3, m4 = self.zp.get_motor_outputs()
            self.client.moveByMotorPWMsAsync(m1 / 100, m2 / 100, m3 / 100, m4 / 100, 0.01)

        self._rate_window_steps += 1
        window = time.perf_counter() - self._rate_window_t
        if window >= 0.5:
            self.achieved_rate_hz = self._rate_window_steps / window
            self._rate_window_t = time.perf_counter()
            self._rate_window_steps = 0

    def print_state(self):
        sys.stdout.write("\033[H")
        arm_s = "\033[1;32mARMED   \033[0m" if self.armed else "\033[1;31mDISARMED\033[0m"
        sim_s = "\033[1;33mPAUSED  \033[0m" if self.paused else "\033[1;32mRUNNING \033[0m"
        
        # Reuse what step() already fetched; the dashboard must not add RPCs to the control loop
        state = self.last_state
        if state is None:
            return
        alt_rangefinder = self.alt_rangefinder
        baro = self.baro
        m1, m2, m3, m4 = self.zp.get_motor_outputs()

        # A rate well below SITL_RATE_HZ means the firmware is being stepped slower than it thinks;
        # the plant clock keeps that honest, but it still costs control bandwidth
        rate_colour = "\033[1;32m" if self.achieved_rate_hz >= 0.5 * SITL_RATE_HZ else "\033[1;33m"

        dash = [
            "==============================================",
            f"   ZeroPilot SITL | {arm_s} | {sim_s}",
            "==============================================",
            f" Roll: {self.commands['roll']:>5.1f}% | Pitch: {self.commands['pitch']:>5.1f}%",
            f" Yaw:  {self.commands['yaw']:>5.1f}% | Thr:   {self.commands['throttle']:>5.1f}%",
            f" FltMode: {self.fltmode_index + 1}",
            f" Loop: {rate_colour}{self.achieved_rate_hz:>6.0f}\033[0m / {SITL_RATE_HZ} Hz",
            "----------------------------------------------",
            f" Alt GPS:  {state.gps_location.altitude:>6.1f} m",
            f" Alt Range: {alt_rangefinder:>6.1f} m",
            f" Alt Baro: {baro.altitude:>6.1f} m",
            f" Pos:  ({state.gps_location.latitude:.4f}, {state.gps_location.longitude:.4f})",
            "==============================================",
            f" M1: {m1:>7.3f} | M2: {m2:>7.3f} | M3: {m3:>7.3f} | M4: {m4:>7.3f}",
            "\033[K"
        ]
        sys.stdout.write("\n".join(dash) + "\n")
        sys.stdout.flush()


if __name__ == '__main__':
    TARGET_IP = "127.0.0.1"
    TARGET_PORT = 14550
    
    sitl = ZP_QUAD_SITL_AIRSIM(ip=TARGET_IP, port=TARGET_PORT)
    
    os.system('cls' if os.name == 'nt' else 'clear')
    threading.Thread(target=sitl.update_joystick, daemon=True).start()

    target_dt, next_step, last_print = (1.0 / SITL_RATE_HZ), time.perf_counter(), 0
    try:
        while True:
            while time.perf_counter() < next_step: pass
            if sitl.reset_requested:
                sitl.reset_to_air()
                sitl.reset_requested = False
                next_step = time.perf_counter()
            sitl.step()
            if time.perf_counter() - last_print > 0.05:
                sitl.print_state(); last_print = time.perf_counter()
            next_step += target_dt
    except KeyboardInterrupt:
        print("\nExiting Sim.")
