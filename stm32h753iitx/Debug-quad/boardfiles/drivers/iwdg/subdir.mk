################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../boardfiles/drivers/iwdg/iwdg.cpp 

OBJS += \
./boardfiles/drivers/iwdg/iwdg.o 

CPP_DEPS += \
./boardfiles/drivers/iwdg/iwdg.d 


# Each subdirectory must supply rules for building sources it contributes
boardfiles/drivers/iwdg/%.o boardfiles/drivers/iwdg/%.su boardfiles/drivers/iwdg/%.cyclo: ../boardfiles/drivers/iwdg/%.cpp boardfiles/drivers/iwdg/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../boardfiles/utils -I../boardfiles/drivers/logger -I../../zeropilot4.0/build/h753iit-quad/include -I../boardfiles/drivers/iwdg -I../boardfiles/drivers/motor -I../boardfiles/drivers/queue -I../boardfiles/drivers/rc -I../boardfiles/drivers/telemlink -I../boardfiles/drivers/gps -I../boardfiles/drivers/systemutils -I../boardfiles/model/inc -I../boardfiles/rtos/threads/inc -I../boardfiles/rtos/museq -I../boardfiles/drivers/imu -I../boardfiles/drivers/power_module -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -isystem "../../external/c_library_v2/all" -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-boardfiles-2f-drivers-2f-iwdg

clean-boardfiles-2f-drivers-2f-iwdg:
	-$(RM) ./boardfiles/drivers/iwdg/iwdg.cyclo ./boardfiles/drivers/iwdg/iwdg.d ./boardfiles/drivers/iwdg/iwdg.o ./boardfiles/drivers/iwdg/iwdg.su

.PHONY: clean-boardfiles-2f-drivers-2f-iwdg

