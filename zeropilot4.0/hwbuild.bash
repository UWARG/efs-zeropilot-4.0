#!/usr/bin/env bash

set -e

script_dir=$( cd -- "$( dirname -- "$0" )" &> /dev/null && pwd )
vehicle_type="fw"
clean="false"
board="h753iit"

usage() {
    echo "Usage: $0 [-c] [-v] <vehicle_type> [-b] <board>"
    exit 1
}

# parse args
while getopts "b:v:c" opt; do
    case "${opt}" in
        b)
            if [[ "$OPTARG" == "l552" || "$OPTARG" == "h753iit" ]]; then
                board="$OPTARG"
            fi
            ;;
        v)
            if [[ "$OPTARG" == "quad" || "$OPTARG" == "fw" ]]; then
                vehicle_type="$OPTARG"
            fi
            ;;
        c)
            clean="true"
            ;;
        *)
            usage
            ;;
    esac
done

# set build dir
if [[ "$board" == "h753iit" && "$vehicle_type" == "fw" ]]; then
    build_dir="${script_dir}/build/h753iit-fw"
elif [[ "$board" == "h753iit" && "$vehicle_type" == "quad" ]]; then
    build_dir="${script_dir}/build/h753iit-quad"
elif [[  "$board" == "l552" && "$vehicle_type" == "fw" ]]; then
    build_dir="${script_dir}/build/l552-fw"
elif [[ "$board" == "l552"  && "$vehicle_type" == "quad" ]]; then
    build_dir="${script_dir}/build/l552-quad"
fi

# clean if requested and setup
if [[ -d "$build_dir" ]]; then
    if [[ "$clean" == "true" ]]; then
        rm -rf "$build_dir"
        mkdir -p "$build_dir"
    fi
else
    mkdir -p "$build_dir"
fi

# if [[ "$vehicle_type" == "fw" ]]; then
#     vehicle_type=1
# elif [[ "$vehicle_type" == "quad" ]]; then
#     vehicle_type=2
# fi

# create cmake system
cd "$build_dir"
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

    # find toolchain file 
    if [[ "$board" == "l552" ]]; then
        tc_file="${script_dir}/../stm32l552xx/stm32l552xx.cmake"
    elif [[ "$board" == "h753iit" ]]; then
        tc_file="${script_dir}/../stm32h753iitx/stm32h753iitx.cmake"
    fi

    echo "generating cmake..."
    echo "generator: $generator"
    echo "toolchain: $tc_file"
    if [[ "$vehicle_type" == "fw" ]]; then
        cmake -G "$generator" -Werror -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE="Debug" -DCMAKE_TOOLCHAIN_FILE="$tc_file" -DFIXED_WING_BUILD=ON "$script_dir"
    elif [[ "$vehicle_type" == "quad" ]]; then
        cmake -G "$generator" -Werror -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE="Debug" -DCMAKE_TOOLCHAIN_FILE="$tc_file" -DQUADCOPTER_BUILD=ON "$script_dir"
    fi
fi

echo && echo "building..."
cmake --build .

echo && echo "copying headers..."
mkdir -p "${build_dir}/include"
find "${script_dir}/include" \( -name "*.hpp" -o -name "*.h" \) -exec echo "{}" \; -exec cp {} "${build_dir}/include" \;
