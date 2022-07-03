################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../flash_fucntion/flash.c 

OBJS += \
./flash_fucntion/flash.o 

C_DEPS += \
./flash_fucntion/flash.d 


# Each subdirectory must supply rules for building sources it contributes
flash_fucntion/%.o flash_fucntion/%.su: ../flash_fucntion/%.c flash_fucntion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/computer/STM32CubeIDE/workspace_1.9.0/project_final/flash_fucntion" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-flash_fucntion

clean-flash_fucntion:
	-$(RM) ./flash_fucntion/flash.d ./flash_fucntion/flash.o ./flash_fucntion/flash.su

.PHONY: clean-flash_fucntion

