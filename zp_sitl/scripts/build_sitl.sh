#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/.."

rm -rf build/
rm -f *.so *.pyd

echo "Building ZeroPilot C++ extension in root..."
python setup.py build_ext --inplace

echo "Build complete! Run with: python sitl_main.py"
