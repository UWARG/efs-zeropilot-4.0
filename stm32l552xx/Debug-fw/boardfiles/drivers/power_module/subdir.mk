################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../boardfiles/drivers/power_module/power_module.cpp 

OBJS += \
./boardfiles/drivers/power_module/power_module.o 

CPP_DEPS += \
./boardfiles/drivers/power_module/power_module.d 


# Each subdirectory must supply rules for building sources it contributes
boardfiles/drivers/power_module/%.o boardfiles/drivers/power_module/%.su boardfiles/drivers/power_module/%.cyclo: ../boardfiles/drivers/power_module/%.cpp boardfiles/drivers/power_module/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DFIXED_WING -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../zeropilot4.0/build/l552-fw/include -I../boardfiles/drivers/iwdg -I../boardfiles/drivers/power_module -I../boardfiles/drivers/logger -I../boardfiles/drivers/motor -I../boardfiles/drivers/queue -I../boardfiles/drivers/rc -I../boardfiles/drivers/telemlink -I../boardfiles/drivers/gps -I../boardfiles/drivers/systemutils -I../boardfiles/model/inc -I../boardfiles/rtos/museq -I../boardfiles/rtos/threads/inc -I../boardfiles/utils -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/krisko/efs-zeropilot-4.0/stm32l552xx/boardfiles/drivers/imu" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -isystem "../../external/c_library_v2/all" -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-boardfiles-2f-drivers-2f-power_module

clean-boardfiles-2f-drivers-2f-power_module:
	-$(RM) ./boardfiles/drivers/power_module/power_module.cyclo ./boardfiles/drivers/power_module/power_module.d ./boardfiles/drivers/power_module/power_module.o ./boardfiles/drivers/power_module/power_module.su

.PHONY: clean-boardfiles-2f-drivers-2f-power_module

