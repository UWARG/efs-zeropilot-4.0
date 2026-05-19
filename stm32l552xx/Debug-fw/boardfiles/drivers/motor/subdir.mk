################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../boardfiles/drivers/motor/dshot.cpp \
../boardfiles/drivers/motor/motor.cpp 

OBJS += \
./boardfiles/drivers/motor/dshot.o \
./boardfiles/drivers/motor/motor.o 

CPP_DEPS += \
./boardfiles/drivers/motor/dshot.d \
./boardfiles/drivers/motor/motor.d 


# Each subdirectory must supply rules for building sources it contributes
boardfiles/drivers/motor/%.o boardfiles/drivers/motor/%.su boardfiles/drivers/motor/%.cyclo: ../boardfiles/drivers/motor/%.cpp boardfiles/drivers/motor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DFIXED_WING -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../zeropilot4.0/build/l552-fw/include -I../boardfiles/drivers/iwdg -I../boardfiles/drivers/power_module -I../boardfiles/drivers/logger -I../boardfiles/drivers/motor -I../boardfiles/drivers/queue -I../boardfiles/drivers/rc -I../boardfiles/drivers/telemlink -I../boardfiles/drivers/gps -I../boardfiles/drivers/systemutils -I../boardfiles/model/inc -I../boardfiles/rtos/museq -I../boardfiles/rtos/threads/inc -I../boardfiles/utils -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/krisko/efs-zeropilot-4.0/stm32l552xx/boardfiles/drivers/imu" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -isystem "../../external/c_library_v2/all" -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-boardfiles-2f-drivers-2f-motor

clean-boardfiles-2f-drivers-2f-motor:
	-$(RM) ./boardfiles/drivers/motor/dshot.cyclo ./boardfiles/drivers/motor/dshot.d ./boardfiles/drivers/motor/dshot.o ./boardfiles/drivers/motor/dshot.su ./boardfiles/drivers/motor/motor.cyclo ./boardfiles/drivers/motor/motor.d ./boardfiles/drivers/motor/motor.o ./boardfiles/drivers/motor/motor.su

.PHONY: clean-boardfiles-2f-drivers-2f-motor

