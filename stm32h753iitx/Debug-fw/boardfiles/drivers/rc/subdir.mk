################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../boardfiles/drivers/rc/rc_crsf.cpp \
../boardfiles/drivers/rc/rc_sbus.cpp 

OBJS += \
./boardfiles/drivers/rc/rc_crsf.o \
./boardfiles/drivers/rc/rc_sbus.o 

CPP_DEPS += \
./boardfiles/drivers/rc/rc_crsf.d \
./boardfiles/drivers/rc/rc_sbus.d 


# Each subdirectory must supply rules for building sources it contributes
boardfiles/drivers/rc/%.o boardfiles/drivers/rc/%.su boardfiles/drivers/rc/%.cyclo: ../boardfiles/drivers/rc/%.cpp boardfiles/drivers/rc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DFIXED_WING -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../boardfiles/utils -I../boardfiles/drivers/logger -I../../zeropilot4.0/build/h753iit-fw/include -I../boardfiles/drivers/iwdg -I../boardfiles/drivers/motor -I../boardfiles/drivers/queue -I../boardfiles/drivers/rc -I../boardfiles/drivers/telemlink -I../boardfiles/drivers/gps -I../boardfiles/drivers/systemutils -I../boardfiles/model/inc -I../boardfiles/rtos/threads/inc -I../boardfiles/rtos/museq -I../boardfiles/drivers/imu -I../boardfiles/drivers/power_module -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -isystem "../../external/c_library_v2/all" -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-boardfiles-2f-drivers-2f-rc

clean-boardfiles-2f-drivers-2f-rc:
	-$(RM) ./boardfiles/drivers/rc/rc_crsf.cyclo ./boardfiles/drivers/rc/rc_crsf.d ./boardfiles/drivers/rc/rc_crsf.o ./boardfiles/drivers/rc/rc_crsf.su ./boardfiles/drivers/rc/rc_sbus.cyclo ./boardfiles/drivers/rc/rc_sbus.d ./boardfiles/drivers/rc/rc_sbus.o ./boardfiles/drivers/rc/rc_sbus.su

.PHONY: clean-boardfiles-2f-drivers-2f-rc

