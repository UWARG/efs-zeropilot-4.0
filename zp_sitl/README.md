# ZeroPilot SITL

Software-in-the-Loop simulation for ZeroPilot autopilot using JSBSim flight dynamics.

## Structure

- `sitl_main.py` - Python simulation loop integrating JSBSim with ZeroPilot
- `zeropilot_wrapper.cpp` - Python C extension wrapping ZeroPilot managers
- `sil_drivers/` - Software-in-the-Loop driver implementations
- `index.html` - Web UI for simulation control
- `sd_card/` - Logging folder where simulated sd card logging gets dumped (gitignored)

## Running

Before running for the first time:
```
pip install -r requirements.txt
```

Then, to build and run:
```bash
./build_sitl.sh
python sitl_main.py
```

Open `http://localhost:8080` to control the simulation. You can use the sliders or connect a joystick. It also streams MAVLink onto UDP at `127.0.0.1:14550` so you can connect MissionPlanner alongside the UI.

## SIL Drivers

Each driver in `sil_drivers/` implements the same interface as the hardware driver but provides simulated data.

### Adding a New SIL Driver

If you just wrote a hardware driver and need to add SIL support:

1. **Create the SIL driver** in `sil_drivers/sil_<name>.hpp`:
   - Inherit from the same interface as your hardware driver
   - Implement all required interface methods
   - Add `update_from_plant()` or similar method to inject simulation data if needed

2. **Include in wrapper** (`zeropilot_wrapper.cpp`):
   - Add `#include "sil_drivers/sil_<name>.hpp"`
   - Add pointer to `ZPObject` struct
   - Instantiate in `ZP_new()`
   - Delete in `ZP_dealloc()`
   - Pass to relevant manager constructor
   - Add update call in `ZP_updatePlant()` with plant data

3. **Update from simulation** (`sitl_main.py`):
   - Call `zp.update_plant()` with new parameters in simulation loop

Example pattern:
```cpp
class SIL_MyDriver : public IMyDriver {
private:
    DriverData_t data;
public:
    void update_from_plant(double sim_value) {
        data.field = sim_value;
    }
    DriverData_t readData() override { return data; }
};
```
