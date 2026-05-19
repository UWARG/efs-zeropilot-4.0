################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../boardfiles/model/src/drivers.cpp \
../boardfiles/model/src/managers.cpp \
../boardfiles/model/src/model.cpp 

OBJS += \
./boardfiles/model/src/drivers.o \
./boardfiles/model/src/managers.o \
./boardfiles/model/src/model.o 

CPP_DEPS += \
./boardfiles/model/src/drivers.d \
./boardfiles/model/src/managers.d \
./boardfiles/model/src/model.d 


# Each subdirectory must supply rules for building sources it contributes
boardfiles/model/src/%.o boardfiles/model/src/%.su boardfiles/model/src/%.cyclo: ../boardfiles/model/src/%.cpp boardfiles/model/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DFIXED_WING -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../zeropilot4.0/build/l552-fw/include -I../boardfiles/drivers/iwdg -I../boardfiles/drivers/power_module -I../boardfiles/drivers/logger -I../boardfiles/drivers/motor -I../boardfiles/drivers/queue -I../boardfiles/drivers/rc -I../boardfiles/drivers/telemlink -I../boardfiles/drivers/gps -I../boardfiles/drivers/systemutils -I../boardfiles/model/inc -I../boardfiles/rtos/museq -I../boardfiles/rtos/threads/inc -I../boardfiles/utils -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/krisko/efs-zeropilot-4.0/stm32l552xx/boardfiles/drivers/imu" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -isystem "../../external/c_library_v2/all" -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-boardfiles-2f-model-2f-src

clean-boardfiles-2f-model-2f-src:
	-$(RM) ./boardfiles/model/src/drivers.cyclo ./boardfiles/model/src/drivers.d ./boardfiles/model/src/drivers.o ./boardfiles/model/src/drivers.su ./boardfiles/model/src/managers.cyclo ./boardfiles/model/src/managers.d ./boardfiles/model/src/managers.o ./boardfiles/model/src/managers.su ./boardfiles/model/src/model.cyclo ./boardfiles/model/src/model.d ./boardfiles/model/src/model.o ./boardfiles/model/src/model.su

.PHONY: clean-boardfiles-2f-model-2f-src

