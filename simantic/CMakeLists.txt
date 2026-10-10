# CLI/CI build of the ZeroPilot 4.0 STM32H753IITX application image.
# Mirrors the STM32CubeIDE managed build described in .cproject/.mxproject.
cmake_minimum_required(VERSION 3.16)
project(zp4-h753 C CXX ASM)

set(ZP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(VEHICLE "PLANE" CACHE STRING "PLANE or QUADCOPTER")
string(TOLOWER "${VEHICLE}" VEHICLE_LC)
set(ZP_LIB_DIR ${ZP_ROOT}/zeropilot4.0/build/h753iit-${VEHICLE_LC}-debug)

file(GLOB_RECURSE HAL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER HAL_SRC EXCLUDE REGEX "_template\\.c$")

file(GLOB CORE_C   ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)
file(GLOB CORE_CPP ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.cpp)
file(GLOB_RECURSE BOARD_SRC
     ${CMAKE_CURRENT_SOURCE_DIR}/boardfiles/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/boardfiles/*.cpp)

set(FR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
file(GLOB FREERTOS_SRC ${FR}/*.c ${FR}/CMSIS_RTOS_V2/*.c)
list(APPEND FREERTOS_SRC ${FR}/portable/GCC/ARM_CM4F/port.c ${FR}/portable/MemMang/heap_4.c)

set(FATFS ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FatFs/src)
set(FATFS_SRC ${FATFS}/ff.c ${FATFS}/diskio.c ${FATFS}/ff_gen_drv.c ${FATFS}/option/syscall.c
              ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/App/fatfs.c
              ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target/user_diskio.c)

file(GLOB DRONECAN_SRC ${ZP_ROOT}/external/dronecan/generated/src/*.c)
list(APPEND DRONECAN_SRC ${ZP_ROOT}/external/dronecan/libcanard/canard.c)


# CMSIS-DSP: CubeIDE build pulled these in via the IDE; build the aggregate
# translation units for the function groups ZeroPilot actually references.
set(DSP ${ZP_ROOT}/external/CMSIS-DSP/Source)
set(DSP_SRC
    ${DSP}/BasicMathFunctions/BasicMathFunctions.c
    ${DSP}/CommonTables/CommonTables.c
    ${DSP}/ComplexMathFunctions/ComplexMathFunctions.c
    ${DSP}/FastMathFunctions/FastMathFunctions.c
    ${DSP}/FilteringFunctions/FilteringFunctions.c
    ${DSP}/MatrixFunctions/MatrixFunctions.c
    ${DSP}/QuaternionMathFunctions/QuaternionMathFunctions.c
    ${DSP}/StatisticsFunctions/StatisticsFunctions.c
    ${DSP}/SupportFunctions/SupportFunctions.c
    ${DSP}/TransformFunctions/TransformFunctions.c)

add_executable(zp4-h753
    ${CORE_C} ${CORE_CPP} ${HAL_SRC} ${BOARD_SRC}
    ${FREERTOS_SRC} ${FATFS_SRC} ${DRONECAN_SRC} ${DSP_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_stm32h753iitx.s)

target_compile_definitions(zp4-h753 PRIVATE
    USE_HAL_DRIVER STM32H753xx USE_PWR_LDO_SUPPLY DEBUG ${VEHICLE})

target_include_directories(zp4-h753 PRIVATE
    Core/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include
    ${FR}/include ${FR}/CMSIS_RTOS_V2 ${FR}/portable/GCC/ARM_CM4F
    ${FATFS} FATFS/App FATFS/Target
    boardfiles/utils boardfiles/model/inc boardfiles/rtos/museq boardfiles/rtos/threads/inc
    boardfiles/drivers/barometer boardfiles/drivers/can_controller boardfiles/drivers/fft
    boardfiles/drivers/gps boardfiles/drivers/imu boardfiles/drivers/iwdg
    boardfiles/drivers/logger boardfiles/drivers/mathutils boardfiles/drivers/motor
    boardfiles/drivers/power_module boardfiles/drivers/queue boardfiles/drivers/rangefinder
    boardfiles/drivers/rc boardfiles/drivers/safety_switch boardfiles/drivers/systemutils
    boardfiles/drivers/telemlink
    ${ZP_ROOT}/zeropilot4.0 ${ZP_LIB_DIR} ${ZP_LIB_DIR}/include
    ${ZP_ROOT}/external/dronecan/generated/include ${ZP_ROOT}/external/dronecan/libcanard)

target_include_directories(zp4-h753 SYSTEM PRIVATE
    ${ZP_ROOT}/external/c_library_v2/all
    ${ZP_ROOT}/external/CMSIS-DSP/Include ${ZP_ROOT}/external/CMSIS-DSP/PrivateInclude)

target_link_directories(zp4-h753 PRIVATE ${ZP_LIB_DIR})
target_link_libraries(zp4-h753 PRIVATE zeropilot4.0 c m nosys)
target_link_options(zp4-h753 PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32H753IITX_FLASH.ld
    -Wl,-Map=zp4-h753.map,--cref,--gc-sections
    -static -Wl,--start-group -Wl,--end-group)
set_target_properties(zp4-h753 PROPERTIES SUFFIX ".elf")
