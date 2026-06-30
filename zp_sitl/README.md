# ZeroPilot SITL

Software-in-the-Loop simulation for ZeroPilot autopilot using JSBSim flight dynamics.

## Structure

- `sitl_plane_jsbsim.py` - Python simulation loop integrating JSBSim with ZeroPilot for PLANE build
- `sitl_plane_fgfs.py` - Python simulation loop integrating FlightGear with ZeroPilot for PLANE build
- `sitl_quad_airsim.py` - Python simulation loop integrating AirSim with ZeroPilot for QUADCOPTER build
- `zeropilot_wrapper.cpp` - Python C extension wrapping ZeroPilot managers
- `sitl_drivers/` - Software-in-the-Loop driver implementations
- `scripts/` - Contains build automation and FlightGear launch scripts
- `ui/` - Frontend assets (HTML, CSS, JS) for the web dashboard
- `util/` - Utility modules including MAVLink decoders
- `sd_card/` - Logging folder where simulated sd card logging gets dumped (gitignored)

## Running

Before running for the first time:
```bash
# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
# On Linux / macOS:
source venv/bin/activate
# On Windows:
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Main Target (Web Dashboard / AirSim)

To build and run the simulation:
```bash
# For PLANE
./scripts/build_sitl.sh PLANE   # Build the C++ extension
python sitl_plane_jsbsim.py     # Start the simulation

# Or, for QUADCOPTER:
./scripts/build_sitl.sh QUADCOPTER  # Build the C++ extension
python sitl_quad_airsim.py          # Start the simulation
```

Open `http://localhost:8080` to control the simulation. You can use the sliders or connect a joystick. It also streams MAVLink onto UDP at `127.0.0.1:14550` so you can connect MissionPlanner alongside the UI. Optionally, set port and ip address for MAVLink through `python sitl_plane_jsbsim.py --ip <ip> --port <port>`.

### FGFS Target

If you install [FlightGear](https://www.flightgear.org/) you can visualize the simulation in real-time. The SITL script automatically generates a UDP output directive to stream flight data to FlightGear.

Run the FGFS SITL target via:
```bash
./scripts/build_sitl.sh PLANE
python sitl_fgfs.py
```

Launch FGFS via `./scripts/start_fgfs.sh` (requires having fgfs in your $PATH):
```bash
# This script runs fgfs --fdm=null --native-fdm=socket,in,60,,5550,udp --aircraft=c172p
./scripts/start_fgfs.sh
```

## SITL Drivers

Each driver in `sitl_drivers/` implements the same interface as the hardware driver but provides simulated data.

### Adding a New SITL Driver

If you just wrote a hardware driver and need to add SITL support:

1. **Create the SITL driver** in `sitl_drivers/sitl_<name>.hpp`:
   - Inherit from the same interface as your hardware driver
   - Implement all required interface methods
   - Add `update_from_plant()` or similar method to inject simulation data if needed

2. **Include in wrapper** (`zeropilot_wrapper.cpp`):
   - Add `#include "sitl_drivers/sitl_<name>.hpp"`
   - Add pointer to `ZPObject` struct
   - Instantiate in `ZP_new()`
   - Delete in `ZP_dealloc()`
   - Pass to relevant manager constructor
   - Add update call in `ZP_updatePlant()` with plant data

3. **Update from simulation** (`sitl_plane_jsbsim.py`):
   - Call `zp.update_from_plant()` with new parameters in simulation loop

Example pattern:
```cpp
class SITL_MyDriver : public IMyDriver {
private:
    DriverData_t data;
public:
    void update_from_plant(double sim_value) {
        data.field = sim_value;
    }
    DriverData_t readData() override { return data; }
};
```
