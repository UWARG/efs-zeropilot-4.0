#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

vehicle_type="plane"

usage() {
    echo "Usage: $0 [-v <plane|quad>]"
    exit 1
}

while getopts "v:" opt; do
    case "${opt}" in
        v)
            if [[ "$OPTARG" == "plane" || "$OPTARG" == "quad" ]]; then
                vehicle_type="$OPTARG"
            else
                usage
            fi
            ;;
        *)
            usage
            ;;
    esac
done

build_dir="${script_dir}/build-host-${vehicle_type}"

echo "==> Setting up host build directory: $build_dir"

# Create/clean host build dir
if [[ -d "$build_dir" ]]; then
  rm -rf "$build_dir"
fi
mkdir -p "$build_dir"

# Run native cmake config with compile_commands export
cd "$build_dir"
echo "==> Running native CMake configure..."
if [[ "$vehicle_type" == "plane" ]]; then
    cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPLANE_BUILD=ON "$script_dir"
else
    cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DQUADCOPTER_BUILD=ON "$script_dir"
fi

echo "==> Building native host build..."
cmake --build .

# Now run clang-tidy on host build directory
echo "==> Running clang-tidy..."

# Find source files.
find "$script_dir/src" "$script_dir/include" \( -name '*.cpp' -o -name '*.hpp' \) -print0 |
  xargs -0 clang-tidy -p "$build_dir" \
    --checks='readability-identifier-naming*' \
    --warnings-as-errors='readability-identifier-naming*' \
    --system-headers=false

echo "==> Done linting."