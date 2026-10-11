#!/usr/bin/env bash
# Build a ZeroPilot 4.0 STM32H753IITX application ELF from a CLI/CI shell.
#
# Upstream only builds the core autopilot *static library* (hwbuild.bash); the
# final application link is an STM32CubeIDE managed build, so there is no
# headless build of the flight controller. CMakeLists.txt in this directory
# reconstructs that link from .cproject/.mxproject.
#
# Usage: simantic/build.sh [checkout (default: this repo)] [plane|quad]
set -euo pipefail

HERE=$(cd -- "$(dirname -- "$0")" && pwd)
ZP=${1:-$HERE/..}
VEHICLE=${2:-plane}

case "$VEHICLE" in
  plane) DEF=PLANE ;;
  quad)  DEF=QUADCOPTER ;;
  *) echo "vehicle must be plane|quad" >&2; exit 1 ;;
esac

# Submodules (MAVLink, CMSIS-DSP, dronecan) must be present.
( cd "$ZP" && git submodule update --init --recursive --depth 1 )

# 1. Core autopilot library, via the project's own script.
( cd "$ZP/zeropilot4.0" && ./hwbuild.bash -v "$VEHICLE" -b h753iit )

# 2. Application link (the part CubeIDE normally does).
cp "$HERE/CMakeLists.txt" "$ZP/stm32h753iitx/CMakeLists.txt"
cmake -G Ninja -S "$ZP/stm32h753iitx" -B "$ZP/stm32h753iitx/build-h753" \
      -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE="$ZP/stm32h753iitx/stm32h753iitx.cmake" \
      -DVEHICLE="$DEF"
cmake --build "$ZP/stm32h753iitx/build-h753"

# 3. Copy the ELF next to test.yaml.
cp "$ZP/stm32h753iitx/build-h753/zp4-h753.elf" "$HERE/zp4-h753.elf"

echo
arm-none-eabi-size "$HERE/zp4-h753.elf"
