#!/usr/bin/env python3
import argparse
import jsbsim
import time
import json
import math
import threading
import os
import asyncio
from aiohttp import web
import zeropilot
from mavlink_decoder import MAVLinkDecoder

class ZP_SITL:
    def __init__(self, ip, port):
        # Initialize JSBSim
        self.fdm = jsbsim.FGFDMExec(None)
        self.fdm.load_model('c172p')
        
        # Set internal JSBSim timestep to 1ms (1kHz)
        self.dt = 0.001
        self.fdm.set_dt(self.dt)
        self.fdm.run_ic()
        
        # Control inputs (0-100 range)
        self.roll_cmd = 50
        self.pitch_cmd = 50
        self.yaw_cmd = 50
        self.throttle_cmd = 0
        self.arm_cmd = 0
        
        # ZeroPilot instance
        self.zp = zeropilot.ZeroPilot(ip=ip, port=port)
        
        # State tracking
        self.armed = False
        self.initialized = False
        self.running = True

    def initialize(self, config):
        """Sets initial flight conditions from the web UI."""
        self.fdm['ic/lat-geod-deg'] = config.get('latitude', 37.4)
        self.fdm['ic/long-gc-deg'] = config.get('longitude', -122.1)
        self.fdm['ic/h-sl-ft'] = config.get('altitude', 0)
        self.fdm['ic/vc-kts'] = config.get('speed', 0)
        self.fdm['ic/phi-deg'] = config.get('roll', 0)
        self.fdm['ic/theta-deg'] = config.get('pitch', 0)
        self.fdm['ic/psi-deg'] = config.get('heading', 0)
        
        self.fdm['fcs/aileron-cmd-norm'] = 0.0
        self.fdm['fcs/elevator-cmd-norm'] = 0.0
        self.fdm['fcs/rudder-cmd-norm'] = 0.0
        
        engine_on = config.get('engine', False)
        self.fdm['propulsion/set-running'] = -1 if engine_on else 0
        self.fdm['fcs/mixture-cmd-norm'] = 1.0 if engine_on else 0.0
        
        self.throttle_cmd = config.get('throttle', 0)
        self.fdm['fcs/throttle-cmd-norm'] = self.throttle_cmd / 100.0
        self.armed = engine_on
        self.arm_cmd = 100 if engine_on else 0
        
        self.fdm.run_ic()
        self.zp.set_max_batt_capacity(self.fdm['propulsion/total-fuel-lbs'])
        self.initialized = True

    def step(self):
        """The 1kHz hot-loop step."""
        if not self.initialized:
            return
        
        try:
            # 1. Update ZeroPilot sensors
            self.zp.update_from_plant(
                self.fdm['attitude/phi-rad'],
                self.fdm['attitude/theta-rad'],
                self.fdm['velocities/p-rad_sec'],
                self.fdm['velocities/q-rad_sec'],
                self.fdm['velocities/r-rad_sec'],
                self.fdm['position/lat-geod-deg'],
                self.fdm['position/long-gc-deg'],
                self.fdm['position/h-sl-ft'] * 0.3048,
                self.fdm['velocities/vg-fps'] * 0.3048,
                self.fdm['attitude/psi-deg'],
                self.fdm['propulsion/total-fuel-lbs'],
                self.fdm['propulsion/engine/propeller-rpm']
            )
            
            # 2. Sync RC commands and update logic
            self.zp.set_rc(
                self.roll_cmd, self.pitch_cmd, 
                self.yaw_cmd, self.throttle_cmd, self.arm_cmd
            )
            # Advances 1 ms in simulation time
            result = self.zp.update()

            # Check for watchdog timeout
            if result is False:
                raise RuntimeError("ZeroPilot Watchdog Timeout!")
            
            # 3. Apply ZeroPilot motor outputs back to JSBSim
            r_out, p_out, y_out, t_out = self.zp.get_motor_outputs()
            self.fdm['fcs/aileron-cmd-norm'] = (r_out - 50) / 50.0
            self.fdm['fcs/elevator-cmd-norm'] = -((p_out - 50) / 50.0)
            self.fdm['fcs/rudder-cmd-norm'] = -((y_out - 50) / 50.0)
            self.fdm['fcs/throttle-cmd-norm'] = t_out / 100.0
            
            # Run the physics engine for one 1ms step
            self.fdm.run()
        except Exception as e:
            print(f"Step Error: {e}")

    def get_state(self):
        """Returns the current flight state for the UI."""
        r_out, p_out, y_out, t_out = self.zp.get_motor_outputs()
        return {
            'roll': math.degrees(self.fdm['attitude/phi-rad']),
            'pitch': math.degrees(self.fdm['attitude/theta-rad']),
            'yaw': math.degrees(self.fdm['attitude/psi-rad']),
            'altitude': self.fdm['position/h-sl-ft'],
            'airspeed': self.fdm['velocities/vc-kts'],
            'rpm': self.fdm['propulsion/engine/propeller-rpm'],
            'roll_output': r_out,
            'pitch_output': p_out,
            'yaw_output': y_out,
            'throttle_output': t_out,
            'armed': self.armed,
        }

# Global SITL instance
sitl = None

# --- Web Server Logic ---

async def index(request):
    """Serves the frontend UI."""
    if os.path.exists('./index.html'):
        return web.FileResponse('./index.html')
    return web.Response(text="Error: index.html not found in current directory.", status=404)

async def websocket_handler(request):
    """Handles real-time communication with the UI."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    async for msg in ws:
        if msg.type == web.WSMsgType.TEXT:
            try:
                data = json.loads(msg.data)
                if data['type'] == 'init':
                    sitl.initialize(data['config'])
                elif data['type'] == 'control':
                    sitl.roll_cmd = data['roll']
                    sitl.pitch_cmd = data['pitch']
                    sitl.yaw_cmd = data['yaw']
                    sitl.throttle_cmd = data['throttle']
                elif data['type'] == 'arm':
                    sitl.armed = not sitl.armed
                    sitl.arm_cmd = 100 if sitl.armed else 0
                elif data['type'] == 'state':
                    await ws.send_json(sitl.get_state())
            except Exception as e:
                print(f"WebSocket Message Error: {e}")
    
    return ws

# Initialize MAVLink decoder
mavlink_decoder = MAVLinkDecoder()

async def rfd_viewer_handler(request):
    """Handles real-time RFD message streaming."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    try:
        while True:
            messages = sitl.zp.get_rfd_messages()
            for direction, message in messages:
                # Try to decode as MAVLink
                # TODO: If a message is split between two UDP messages, we currently don't handle it correctly and just stream the bytes. This is pretty rare though rn
                decoded = mavlink_decoder.decode_hex_message(message)
                
                if decoded:
                    msg_name, formatted = decoded
                    await ws.send_json({
                        "direction": direction,
                        "raw": message,
                        "decoded": formatted,
                        "type": msg_name
                    })
                else:
                    # Send raw if decoding fails
                    await ws.send_json({
                        "direction": direction,
                        "raw": message,
                        "decoded": None,
                        "type": "UNKNOWN"
                    })
            await asyncio.sleep(0.1)
    except Exception as e:
        print(f"Telemetry Viewer Error: {e}")
    finally:
        await ws.close()

def start_webserver():
    """Starts the aiohttp server in a dedicated thread."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/rfd', rfd_viewer_handler)
    
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, 'localhost', 8080)
    loop.run_until_complete(site.start())
    
    print("UI available at http://localhost:8080")
    loop.run_forever()

# --- Main Execution ---

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run ZeroPilot SITL simulation.")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="IP address for ZeroPilot UDP communication")
    parser.add_argument("--port", type=int, default=14550, help="Port for ZeroPilot UDP communication")
    args = parser.parse_args()

    global sitl
    sitl = ZP_SITL(args.ip, args.port)

    # 1. Start Web Server Thread
    server_thread = threading.Thread(target=start_webserver, daemon=True)
    server_thread.start()
    
    print("SITL Physics started. Target: 1000Hz via busy-wait.")
    print("Mavlink UDP forwarding on {}:{}".format(args.ip, args.port))

    # 2. High-Precision Timing Loop
    target_dt = 0.001 # 1ms
    next_step = time.perf_counter()

    try:
        while sitl.running:
            # Busy-wait: Polling the CPU for the exact microsecond
            while time.perf_counter() < next_step:
                pass
            
            # Execute physics and autopilot logic
            sitl.step()
            
            # Increment target time
            next_step += target_dt

            # Guard: If we fall behind by more than 100ms, sync clock
            # to avoid the simulation 'fast-forwarding' to catch up.
            if time.perf_counter() > next_step + 0.1:
                next_step = time.perf_counter()

    except KeyboardInterrupt:
        print("\nStopping SITL...")
        sitl.running = False

if __name__ == '__main__':
    main()
