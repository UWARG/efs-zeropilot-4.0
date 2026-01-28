set (CMAKE_SYSTEM_PROCESSOR "arm" CACHE STRING "")
set (CMAKE_SYSTEM_NAME "Generic" CACHE STRING "")

# Skip link step during toolchain validation.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX   "arm-none-eabi-")
set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
set(CMAKE_AR           "${TOOLCHAIN_PREFIX}ar")
set(CMAKE_LINKER       "${TOOLCHAIN_PREFIX}ld")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_RANLIB       "${TOOLCHAIN_PREFIX}ranlib")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size")
set(CMAKE_STRIP        "${TOOLCHAIN_PREFIX}ld")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(C_FLAGS            "-std=gnu11")
set(CPP_FLAGS          "-std=gnu++14 -fno-exceptions -fno-rtti -fno-use-cxa-atexit")
set(MCU_FLAGS          "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb --specs=nano.specs")
set(OPTIMIZATION_FLAGS "-O0 -ffunction-sections -fdata-sections")
set(WARNING_FLAGS      "-Wall")
set(DEBUG_FLAGS        "-fstack-usage -g3")

set(CMAKE_C_FLAGS_DEBUG   "${C_FLAGS} ${MCU_FLAGS} ${OPTIMIZATION_FLAGS} ${WARNING_FLAGS} ${DEBUG_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CPP_FLAGS} ${MCU_FLAGS} ${OPTIMIZATION_FLAGS} ${WARNING_FLAGS} ${DEBUG_FLAGS}")

set(CMAKE_C_FLAGS_RELEASE   "${C_FLAGS} ${MCU_FLAGS} ${OPTIMIZATION_FLAGS} ${WARNING_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CPP_FLAGS} ${MCU_FLAGS} ${OPTIMIZATION_FLAGS} ${WARNING_FLAGS}")