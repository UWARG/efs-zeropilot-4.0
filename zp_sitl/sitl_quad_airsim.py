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

class ZP_QUAD_SITL_AIRSIM:
    def __init__(self):
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
        self.zp = zeropilot.ZeroPilot(sitl_rate_hz=SITL_RATE_HZ)
        self.running = True
        self.armed = False
        self.paused = True 
        self.commands = {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0}
        self.fltmode_setpoints = [16.5, 29.5, 42.5, 55.5, 68.5, 81.5]
        self.fltmode_index = 0

        print("initialized")

    def update_joystick(self):
        while self.running:
            pygame.event.pump()
            if self.joy:
                # Mapping: 2:Roll, 3:Pitch, 0:Yaw, 1:Throttle
                self.commands['roll'] = (self.joy.get_axis(0) + 1) * 50
                self.commands['pitch'] = (self.joy.get_axis(1) + 1) * 50
                self.commands['yaw'] = (self.joy.get_axis(3) + 1) * 50
                self.commands['throttle'] = (self.joy.get_axis(2) + 1) * 50

                if self.joy.get_axis(6) > 0.5: 
                    self.armed = True
                elif self.joy.get_axis(6) < 0.5:
                    self.armed = False
                
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        # if event.button == 1:
                        #     self.reset_to_air()
                        # if event.button == 2:
                        #     self.armed = True
                        if event.button == 3:
                            self.paused = not self.paused
                    elif event.type == pygame.JOYAXISMOTION:
                        if event.axis == 4 and event.value > 0.5:  # ZL button
                            self.fltmode_index = max(0, self.fltmode_index - 1)
                        elif event.axis == 5 and event.value > 0.5:  # ZR button
                            self.fltmode_index = min(len(self.fltmode_setpoints) - 1, self.fltmode_index + 1)
                    # elif event.type == pygame.JOYBUTTONUP:
                    #     if event.button == 2:
                    #          self.armed = False

            time.sleep(0.01)
    
    def reset_to_air(self):
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        time.sleep(1)

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

        # latitude, longtitude, altitude
        lat_deg = state.gps_location.latitude
        long_deg = state.gps_location.longitude
        alt_deg = state.gps_location.altitude

        lv = state.kinematics_estimated.linear_velocity
        ground_speed = math.sqrt(lv.x_val**2 + lv.y_val**2)  # horizontal only, m/s

        heading = math.degrees(yaw) % 360 

        self.zp.update_from_plant(
            roll, pitch, 
            p_rad, q_rad, r_rad,
            lat_deg, long_deg, alt_deg,
            ground_speed, 
            heading,
            0.0,
            0.0
        )

        self.zp.set_rc(self.commands['roll'], self.commands['pitch'], self.commands['yaw'],
                self.commands['throttle'], 100 if self.armed else 0, self.fltmode_setpoints[self.fltmode_index])
        self.zp.update()

        if not self.paused:
            m1, m2, m3, m4 = self.zp.get_motor_outputs()
            self.client.moveByMotorPWMsAsync(m1 / 100, m2 / 100, m3 / 100, m4 / 100, 0.01) 
        
    def print_state(self):
        sys.stdout.write("\033[H")
        arm_s = "\033[1;32mARMED   \033[0m" if self.armed else "\033[1;31mDISARMED\033[0m"
        sim_s = "\033[1;33mPAUSED  \033[0m" if self.paused else "\033[1;32mRUNNING \033[0m"
        
        state = self.client.getMultirotorState()
        m1, m2, m3, m4 = self.zp.get_motor_outputs()

        dash = [
            "==============================================",
            f"   ZeroPilot SITL | {arm_s} | {sim_s}",
            "==============================================",
            f" Roll: {self.commands['roll']:>5.1f}% | Pitch: {self.commands['pitch']:>5.1f}%",
            f" Yaw:  {self.commands['yaw']:>5.1f}% | Thr:   {self.commands['throttle']:>5.1f}%",
            f" FltMode: {self.fltmode_index + 1}",
            "----------------------------------------------",
            f" Alt:  {state.gps_location.altitude:>6.1f} m",
            f" Pos:  ({state.gps_location.latitude:.4f}, {state.gps_location.longitude:.4f})",
            "==============================================",
            f" M1: {m1:>7.3f} | M2: {m2:>7.3f} | M3: {m3:>7.3f} | M4: {m4:>7.3f}",
            " [A] Arm | [B] Disarm | [BACK] Pause | [START] Reset",
            " [L] Flaps Down | [R] Flaps Up | [ZL] Mode- | [ZR] Mode+",
            "\033[K"
        ]
        sys.stdout.write("\n".join(dash) + "\n")
        sys.stdout.flush()


if __name__ == '__main__':
    sitl = ZP_QUAD_SITL_AIRSIM()
    
    os.system('cls' if os.name == 'nt' else 'clear')
    threading.Thread(target=sitl.update_joystick, daemon=True).start()

    target_dt, next_step, last_print = (1.0 / SITL_RATE_HZ), time.perf_counter(), 0
    try:
        while True:
            while time.perf_counter() < next_step: pass
            sitl.step()
            if time.perf_counter() - last_print > 0.05:
                sitl.print_state(); last_print = time.perf_counter()
            next_step += target_dt
    except KeyboardInterrupt:
        if os.path.exists(sitl.fg_out_file): os.remove(sitl.fg_out_file)
