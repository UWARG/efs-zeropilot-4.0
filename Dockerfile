FROM xanderhendriks/stm32cubeide:16.0

RUN apt-get update && apt-get install -y cmake ninja-build git && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .

RUN git submodule update --init --recursive || true

RUN chmod +x zeropilot4.0/hwbuild.bash
