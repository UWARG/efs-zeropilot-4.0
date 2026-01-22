#!/bin/bash
set -e

BUILD_TYPE="Debug"

if [[ "$1" == "--release" || "$1" == "-r" ]]; then
    BUILD_TYPE="Release"
elif [[ "$1" == "--debug" || "$1" == "-d" ]]; then
    BUILD_TYPE="Debug"
fi

IMAGE_NAME="zeropilot-ci"
CONTAINER_NAME="zeropilot-build-l552-$$"

echo "Building Docker image..."
docker build -t $IMAGE_NAME .

echo "Running build for stm32l552xx ($BUILD_TYPE)..."
docker run --rm --name $CONTAINER_NAME $IMAGE_NAME \
    stm32cubeide --launcher.suppressErrors -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data /workspace -import /workspace/stm32l552xx \
    -cleanBuild stm32l552xx/$BUILD_TYPE

echo "Build complete!"
