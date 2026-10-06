#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/.."

VEHICLE=${1:-PLANE}

VEHICLE=$(echo "$VEHICLE" | tr '[:lower:]' '[:upper:]')

if [[ "$VEHICLE" != "PLANE" && "$VEHICLE" != "QUADCOPTER" ]]; then
    echo "Error: Vehicle flag must be PLANE or QUADCOPTER. Got: $VEHICLE"
    echo "Usage: ./build_sitl.sh [PLANE|QUADCOPTER]"
    exit 1
fi

rm -rf build/
rm -f *.so *.pyd

echo "Building ZeroPilot C++ extension for $VEHICLE..."

V=$VEHICLE python setup.py build_ext --inplace

echo "Build complete!"
if [[ "$VEHICLE" == "PLANE" ]]; then
    echo "Run with: python sitl_plane_jsbsim.py"
else
    echo "Run with: python sitl_quad_airsim.py"
fi
