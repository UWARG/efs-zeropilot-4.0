#!/bin/bash
set -e

rm -rf build/
rm -f *.so *.pyd

echo "Building ZeroPilot C++ extension..."
python setup.py build_ext --inplace

echo "Build complete! Run with: python sitl_main.py"
