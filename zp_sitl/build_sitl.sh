#!/bin/bash
set -e

echo "Building ZeroPilot C++ extension..."
python3 setup.py build_ext --inplace

echo "Build complete! Run with: python3 sitl_main.py"
