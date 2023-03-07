################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/buffer-c/src/buffer.c 

OBJS += \
./Lib/buffer-c/src/buffer.o 

C_DEPS += \
./Lib/buffer-c/src/buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/buffer-c/src/%.o Lib/buffer-c/src/%.su: ../Lib/buffer-c/src/%.c Lib/buffer-c/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/buffer-c/src" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/rs485-bus-communication/src/inc" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/smt32-dma-streamer/src/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lib-2f-buffer-2d-c-2f-src

clean-Lib-2f-buffer-2d-c-2f-src:
	-$(RM) ./Lib/buffer-c/src/buffer.d ./Lib/buffer-c/src/buffer.o ./Lib/buffer-c/src/buffer.su

.PHONY: clean-Lib-2f-buffer-2d-c-2f-src

