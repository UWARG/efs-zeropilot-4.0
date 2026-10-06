#!/usr/bin/env bash

set -e

script_dir=$( cd -- "$( dirname -- "$0" )" &> /dev/null && pwd )
clean="false"
vehicle_type="plane"

usage() {
    echo "Usage: $0 [-c] [-v] <vehicle_type>"
    exit 1
}

# parse args
while getopts "cv:" opt; do
    case "${opt}" in
        c)
            clean="true"
            ;;
        v)
            if [[ "$OPTARG" == "quad" || "$OPTARG" == "plane" ]]; then
                vehicle_type="$OPTARG"
            fi
            ;;
        *)
            usage
            ;;
    esac
done

# clean if requested and setup
if [[ -d "${script_dir}/build" ]]; then
    if [[ "$clean" == "true" ]]; then
        rm -rf "${script_dir}/build"
        mkdir "${script_dir}/build"
    fi
else
    mkdir "${script_dir}/build"
fi

# create cmake system
cd "${script_dir}/build"
if [[ ! -f "CMakeCache.txt" ]]; then
    # find cmake generator
    if command -v ninja >/dev/null 2>&1; then
        generator="Ninja"
    elif command -v make >/dev/null 2>&1; then
        generator="Unix Makefiles"
    elif command -v mingw32-make >/dev/null 2>&1; then
        generator="MinGW Makefiles"
    else
        echo "error: No cmake generator found."
        exit 1
    fi

    echo "generating cmake..."
    echo "generator: $generator"
    if [[ "$vehicle_type" == "plane" ]]; then
        cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G "$generator" -DPLANE_BUILD=ON ..
    elif [[ "$vehicle_type" == "quad" ]]; then
        cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G "$generator" -DQUADCOPTER_BUILD=ON ..
    fi
fi

echo && echo "building..."
cmake --build .
