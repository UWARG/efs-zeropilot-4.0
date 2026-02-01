################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l552zetxq.s 

S_DEPS += \
./Core/Startup/startup_stm32l552zetxq.d 

OBJS += \
./Core/Startup/startup_stm32l552zetxq.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../boardfiles/drivers/logger -I../boardfiles/utils -I../Core/Inc/dsdlc_generated/include -I../boardfiles/drivers/can -I../boardfiles/drivers/gps -I../boardfiles/drivers/iwdg -I../boardfiles/drivers/motor -I../boardfiles/drivers/rc -I../boardfiles/drivers/rfd -I../boardfiles/model/inc -I../boardfiles/rtos/museq -I../boardfiles/rtos/threads/inc -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l552zetxq.d ./Core/Startup/startup_stm32l552zetxq.o

.PHONY: clean-Core-2f-Startup

