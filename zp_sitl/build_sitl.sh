#!/bin/bash
set -e

echo "Building ZeroPilot C++ extension..."
python setup.py build_ext --inplace

echo "Build complete! Run with: python sitl_main.py"
