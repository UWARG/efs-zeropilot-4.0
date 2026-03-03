#!/usr/bin/env python3
import jsbsim
import time
import threading
import os
import tempfile
import pygame
import sys
import zeropilot

# Suppress pygame and JSBSim chatter
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

class ZP_FGFS_SITL:
    def __init__(self, fg_host="127.0.0.1", fg_port=5550):
        # 1. Input Setup (Joysticks)
        pygame.init()
        pygame.joystick.init()
        self.joy = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
        if self.joy: self.joy.init()
        
        # 2. JSBSim Setup
        self.fdm = jsbsim.FGFDMExec(None)
        self.fdm.set_debug_level(0)
        self.fg_out_file = self._create_fg_directive(fg_host, fg_port)
        self.fdm.load_model('c172p')
        self.fdm.set_output_directive(self.fg_out_file)
        self.fdm.set_dt(0.001)

        # 3. State Setup
        self.zp = zeropilot.ZeroPilot()
        self.running = True
        self.armed = False
        self.paused = True 
        self.commands = {'roll': 50, 'pitch': 50, 'yaw': 50, 'throttle': 0}

    def _create_fg_directive(self, host, port):
        xml = f"""<?xml version="1.0"?>
<output name="{host}" type="FLIGHTGEAR" port="{port}" protocol="UDP" rate="60">
    <property> /position/long-gc-deg </property>
    <property> /position/lat-geod-deg </property>
    <property> /position/h-sl-ft </property>
    
    <property> /attitude/phi-rad </property>
    <property> /attitude/theta-rad </property>
    <property> /attitude/psi-rad </property>

    <property> /velocities/vcalibrated-kts </property> 
    <property> /velocities/h-dot-fps </property>
    <property> /velocities/u-fps </property>
    <property> /velocities/v-fps </property>
    <property> /velocities/w-fps </property>
    
    <property> /propulsion/engine/propeller-rpm </property>

    <property> /fcs/elevator-pos-norm </property>
    <property> /fcs/left-aileron-pos-norm </property>
    <property> /fcs/right-aileron-pos-norm </property>
    <property> /fcs/rudder-pos-norm </property>
    <property> /fcs/throttle-pos-norm </property>
</output>"""
        tmp = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
        tmp.write(xml) 
        tmp.close()
        return tmp.name

    def reset_to_air(self):
        """Forces immediate engine start and refills all fuel tanks."""
        # Initial Position
        self.fdm['ic/lat-geod-deg'] = 37.4223
        self.fdm['ic/long-gc-deg'] = -122.0841
        self.fdm['ic/h-sl-ft'] = 2500.0
        self.fdm['ic/vc-kts'] = 110.0
        
        # --- Multi-Tank Fuel Refill ---
        # Filling multiple indices ensures the c172p fuel system sees a source
        try:
            self.fdm['propulsion/tank[0]/contents-lbs'] = 150.0
            self.fdm['propulsion/tank[1]/contents-lbs'] = 150.0
        except:
            pass

        self.fdm['propulsion/set-running'] = -1 
        self.fdm['fcs/mixture-cmd-norm'] = 1.0 
        self.fdm['fcs/mag-left-cmd'] = 1
        self.fdm['fcs/mag-right-cmd'] = 1
        
        self.fdm.run_ic()
        self.paused = True

    def update_joystick(self):
        while self.running:
            pygame.event.pump()
            if self.joy:
                # Mapping: 2:Roll, 3:Pitch, 0:Yaw, 1:Throttle
                self.commands['roll'] = (self.joy.get_axis(2) + 1) * 50
                self.commands['pitch'] = (self.joy.get_axis(3) + 1) * 50
                self.commands['yaw'] = (self.joy.get_axis(0) + 1) * 50
                self.commands['throttle'] = ((-self.joy.get_axis(1) + 1) * 50)

                if self.joy.get_button(0): self.armed = True
                if self.joy.get_button(1): self.armed = False
                if self.joy.get_button(7): self.reset_to_air()
                
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN and event.button == 6:
                        self.paused = not self.paused
            time.sleep(0.01)

    def step(self):
        # Update ZeroPilot Sensors
        self.zp.update_from_plant(
            self.fdm['attitude/phi-rad'], self.fdm['attitude/theta-rad'],
            self.fdm['velocities/p-rad_sec'], self.fdm['velocities/q-rad_sec'],
            self.fdm['velocities/r-rad_sec'], self.fdm['position/lat-geod-deg'],
            self.fdm['position/long-gc-deg'], self.fdm['position/h-sl-ft'] * 0.3048,
            self.fdm['velocities/vg-fps'] * 0.3048, self.fdm['attitude/psi-deg'],
            self.fdm['propulsion/total-fuel-lbs'], self.fdm['propulsion/engine/propeller-rpm']
        )
        self.zp.set_rc(self.commands['roll'], self.commands['pitch'], self.commands['yaw'], 
                       self.commands['throttle'], 100 if self.armed else 0)
        self.zp.update()

        if not self.paused:
            r_out, p_out, y_out, t_out = self.zp.get_motor_outputs()
            
            # Surface mapping
            self.fdm['fcs/aileron-cmd-norm'] = (r_out - 50) / 50.0
            self.fdm['fcs/elevator-cmd-norm'] = -((p_out - 50) / 50.0)
            self.fdm['fcs/rudder-cmd-norm'] = -((y_out - 50) / 50.0)
            self.fdm['fcs/throttle-cmd-norm'] = t_out / 100.0
            
            # Maintain engine running state
            self.fdm['fcs/mixture-cmd-norm'] = 1.0             
            self.fdm.run()

    def print_state(self):
        sys.stdout.write("\033[H")
        arm_s = "\033[1;32mARMED   \033[0m" if self.armed else "\033[1;31mDISARMED\033[0m"
        sim_s = "\033[1;33mPAUSED  \033[0m" if self.paused else "\033[1;32mRUNNING \033[0m"
        
        dash = [
            "==============================================",
            f"   ZeroPilot SITL | {arm_s} | {sim_s}",
            "==============================================",
            f" Roll: {self.commands['roll']:>5.1f}% | Pitch: {self.commands['pitch']:>5.1f}%",
            f" Yaw:  {self.commands['yaw']:>5.1f}% | Thr:   {self.commands['throttle']:>5.1f}%",
            "----------------------------------------------",
            f" Alt:  {self.fdm['position/h-sl-ft']:>6.0f} ft | Spd: {self.fdm['velocities/vc-kts']:>5.1f} kt",
            f" Fuel: {self.fdm['propulsion/total-fuel-lbs']:>6.1f} lbs | RPM: {self.fdm['propulsion/engine/propeller-rpm']:>5.0f}",
            "==============================================",
            " [A] Arm | [B] Disarm | [BACK] Pause | [START] Reset",
            "\033[K"
        ]
        sys.stdout.write("\n".join(dash) + "\n")
        sys.stdout.flush()

if __name__ == '__main__':
    sitl = ZP_FGFS_SITL()
    sitl.reset_to_air()
    
    os.system('cls' if os.name == 'nt' else 'clear')
    threading.Thread(target=sitl.update_joystick, daemon=True).start()

    target_dt, next_step, last_print = 0.001, time.perf_counter(), 0
    try:
        while True:
            while time.perf_counter() < next_step: pass
            sitl.step()
            if time.perf_counter() - last_print > 0.05:
                sitl.print_state(); last_print = time.perf_counter()
            next_step += target_dt
    except KeyboardInterrupt:
        if os.path.exists(sitl.fg_out_file): os.remove(sitl.fg_out_file)
