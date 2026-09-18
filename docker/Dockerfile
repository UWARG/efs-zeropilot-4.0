FROM xanderhendriks/stm32cubeide:16.0

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install common build tools, ARM toolchain, and testing dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    ninja-build \
    git \
    make \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    g++ \
    build-essential \
    libgtest-dev \
    libgmock-dev \
    clang-tidy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
