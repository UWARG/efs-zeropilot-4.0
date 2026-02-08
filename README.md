# ZeroPilot 4.0

Custom flight controller autopilot system by WARG (Waterloo Aerial Robotics Group).

**Documentation**: https://uwarg-docs.atlassian.net/wiki/spaces/ZP/pages/2668101634/ZeroPilot4.0+Design

## Project Structure

```
zeropilot4.0/               # Core autopilot library
├── src/                    # Manager implementations
│   ├── attitude_manager/   # Flight control & stabilization
│   ├── telemetry_manager/  # MAVLink communication
│   └── system_manager/     # System-level tasks
├── include/                # Headers & driver interfaces
└── tests/                  # Unit tests (GoogleTest)

stm32h753iitx/              # STM32H753 hardware target
stm32l552xx/                # STM32L552 hardware target
zp_sitl/                    # Software-in-the-Loop simulation
external/                   # Third-party dependencies (MAVLink)
```

## Building

### Hardware Build

Build for STM32H753 or STM32L552 can be done via STM32CubeIDE.

### Unit Tests

```bash
cd zeropilot4.0/tests
./testbuild.bash            # Build tests
./testbuild.bash -c         # Clean build
./build/gtestzeropilot4.0   # Run tests
```

**Requirements**: GoogleTest, GoogleMock

### SITL Simulation ([WIP](https://github.com/UWARG/efs-zeropilot-4.0/pull/110))

Software-in-the-Loop simulation with JSBSim flight dynamics:

```bash
cd zp_sitl
pip install -r requirements.txt
./build_sitl.sh
python sitl_main.py
```

Open `http://localhost:8080` for web UI. MAVLink streams to `udp://127.0.0.1:14550` (connect MissionPlanner/QGroundControl).

See [zp_sitl/README.md](zp_sitl/README.md) for details.

## Development

### Linting

```bash
cd zeropilot4.0
./lint.bash
```

### Debugging Unit Tests (VS Code)

Create `.vscode/launch.json`:

```json
{
  "version": "0.2.0",
  "configurations": [{
    "name": "GTest Debug",
    "type": "cppdbg",
    "request": "launch",
    "program": "${workspaceFolder}/zeropilot4.0/tests/build/gtestzeropilot4.0",
    "args": ["--gtest_filter=*"],
    "cwd": "${workspaceFolder}",
    "MIMode": "gdb",
    "miDebuggerPath": "/usr/bin/gdb",
    "setupCommands": [{
      "description": "Enable pretty-printing for gdb",
      "text": "-enable-pretty-printing",
      "ignoreFailures": true
    }]
  }]
}
```

Note: Use --gtest_filter to run a specific or subset of tests: https://google.github.io/googletest/advanced.html#running-a-subset-of-the-tests

## CI/CD

GitHub Actions run on every push/PR:
- Hardware compilation (both targets, Debug/Release)
- Unit tests
- SITL build verification
- Code linting

## Hardware Targets

- **STM32H753IIT6**: Primary flight controller (480MHz, 2MB Flash)
- **STM32L552ZET6Q**: Secondary target (110MHz, 512KB Flash)

Both use FreeRTOS with custom driver interfaces for portability.
