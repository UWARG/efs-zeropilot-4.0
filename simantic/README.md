# Running ZeroPilot 4.0 in the Simantic simulator

Your `stm32h753iitx` plane build runs unmodified in the [Simantic](https://simantic.dev)
simulator, with a scripted device on every bus it talks to. No board, no ESP32 HIL rig.

| bus | device | script |
| --- | --- | --- |
| USART6 / USART3 | u-blox GPS x2 (UBX, NAV-PVT) | `gps_peer.py` |
| SPI1 (CS PC4 / PC5) | ICM-42688-P x2 | `icm42688_peer.py` |
| I2C1 0x45 | INA228 power monitor | `ina228_peer.py` |
| I2C2 0x63 | ICP-20100 barometer | `icp20100_peer.py` |
| I2C3 0x10 | TF02-Pro rangefinder | `tf02pro_peer.py` |
| UART4 | CRSF receiver | `crsf_peer.py` |
| FDCAN1 | one DroneCAN node | `dronecan_peer.py` |

`scenario.yaml` is the wiring. Each peer is a short Python script; the values it
reports (10 deg nose-up, 100.4 kPa, 16.8 V, 12.5 m, sticks centred) are constants at the
top of the file. Edit them, or add behaviour, that is what they are for.

## Status: what is and is not checked

Peers are **sim only**: nobody has compared them to the real part on a bench.
What was matched to a NUCLEO-H753ZI on a logic analyser is the MCU model underneath
(UART clocks, I2C, SPI, TIM1 PWM, FDCAN timing). Sensor data is fixed, with no noise,
motion or injected failures. Known open item: `SCALED_PRESSURE.press_abs` is not a plausible
pressure even though the baro peer answers; `test_baro_reaches_scaled_pressure` is marked `xfail` for it.

## 1. Install `sim` (macOS arm64 or Linux x64)

```sh
V=0.6.6   # or any 0.6.x release
curl -fsSL https://drjdhqfvrttolueolzif.supabase.co/storage/v1/object/public/releases/cli/$V/osx-arm64/sim-v$V-osx-arm64.zip -o sim.zip
#   linux: .../cli/$V/linux-x64/sim-v$V-linux-x64.zip
unzip sim.zip && chmod +x sim && mv sim ~/.local/bin/
sim login      # browser sign-in; the STM32H753 model is fetched on first run
```

## 2. Build the ELF (needs `arm-none-eabi-gcc`, `cmake`, `ninja`)

```sh
simantic/build.sh              # plane build of this checkout -> simantic/zp4-h753.elf
simantic/build.sh <checkout> quad
```

The managed CubeIDE link has no command-line equivalent, so `simantic/CMakeLists.txt`
reproduces it from `.cproject`; `build.sh` copies it to `stm32h753iitx/CMakeLists.txt`
and links against the library `hwbuild.bash` produces.

## 3. Run

```sh
cd simantic
sim --scenario scenario.yaml --timeout 12 --output out.txt --frames frames.txt
python3 mavcrc.py out.txt       # MAVLink frames per message id, CRC-checked
head frames.txt                 # one line per SPI / I2C / CAN transaction
```

Expect HEARTBEAT, ATTITUDE, RAW_IMU, SCALED_PRESSURE, GPS_RAW_INT, RC_CHANNELS,
BATTERY_STATUS, DISTANCE_SENSOR and STATUSTEXT on USART1, all CRC-clean. 12 s of
flight-controller time takes about 30 s wall.

## 4. Peer tests

```sh
pip install pytest
pytest simantic/tests
```

`tests/test_peers.py` does one 12 s run and checks, per peer: MAVLink stream CRC-clean
and complete; both GPS ports ACKed and a 3D fix reaches `GPS_RAW_INT`; both IMUs polled and
`ATTITUDE` pitch follows the IMU peer; baro, rangefinder (`DISTANCE_SENSOR` = 1250 cm)
and power module read; DroneCAN traffic in both directions. If you change a peer's
constants, change the expected number in the matching test.
