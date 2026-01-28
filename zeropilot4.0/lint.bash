#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

build_dir="${script_dir}/build-host"

echo "==> Setting up host build directory: $build_dir"

# Create/clean host build dir
if [[ -d "$build_dir" ]]; then
  rm -rf "$build_dir"
fi
mkdir -p "$build_dir"

# Run native cmake config with compile_commands export
cd "$build_dir"
echo "==> Running native CMake configure..."
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$script_dir"

echo "==> Building native host build..."
cmake --build .

# Now run clang-tidy on host build directory
echo "==> Running clang-tidy..."

# Find source files (adjust if needed)
src_files=$(find "$script_dir/src" "$script_dir/include" -name '*.cpp' -o -name '*.hpp')

# Run clang-tidy with compile_commands from host build
clang-tidy $src_files -p "$build_dir" \
  --checks='readability-identifier-naming*' \
  --warnings-as-errors='readability-identifier-naming*' \
  --system-headers=false

echo "==> Done linting."