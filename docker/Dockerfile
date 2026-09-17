FROM xanderhendriks/stm32cubeide:16.0

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install common build tools, ARM toolchain, and Python dependencies used across CI jobs
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    ninja-build \
    git \
    make \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    g++ \
    build-essential \
    libgtest-dev \
    libgmock-dev \
    clang-tidy \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
