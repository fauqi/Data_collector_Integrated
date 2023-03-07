################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/smt32-dma-streamer/src/dma_streamer.c 

OBJS += \
./Lib/smt32-dma-streamer/src/dma_streamer.o 

C_DEPS += \
./Lib/smt32-dma-streamer/src/dma_streamer.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/smt32-dma-streamer/src/%.o Lib/smt32-dma-streamer/src/%.su: ../Lib/smt32-dma-streamer/src/%.c Lib/smt32-dma-streamer/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/buffer-c/src" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/rs485-bus-communication/src/inc" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/smt32-dma-streamer/src/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lib-2f-smt32-2d-dma-2d-streamer-2f-src

clean-Lib-2f-smt32-2d-dma-2d-streamer-2f-src:
	-$(RM) ./Lib/smt32-dma-streamer/src/dma_streamer.d ./Lib/smt32-dma-streamer/src/dma_streamer.o ./Lib/smt32-dma-streamer/src/dma_streamer.su

.PHONY: clean-Lib-2f-smt32-2d-dma-2d-streamer-2f-src

