################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/rs485-bus-communication/src/event_handler.c \
../Lib/rs485-bus-communication/src/master.c \
../Lib/rs485-bus-communication/src/package.c \
../Lib/rs485-bus-communication/src/rs485-bus.c \
../Lib/rs485-bus-communication/src/slave.c 

OBJS += \
./Lib/rs485-bus-communication/src/event_handler.o \
./Lib/rs485-bus-communication/src/master.o \
./Lib/rs485-bus-communication/src/package.o \
./Lib/rs485-bus-communication/src/rs485-bus.o \
./Lib/rs485-bus-communication/src/slave.o 

C_DEPS += \
./Lib/rs485-bus-communication/src/event_handler.d \
./Lib/rs485-bus-communication/src/master.d \
./Lib/rs485-bus-communication/src/package.d \
./Lib/rs485-bus-communication/src/rs485-bus.d \
./Lib/rs485-bus-communication/src/slave.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/rs485-bus-communication/src/%.o Lib/rs485-bus-communication/src/%.su: ../Lib/rs485-bus-communication/src/%.c Lib/rs485-bus-communication/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/buffer-c/src" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/rs485-bus-communication/src/inc" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/smt32-dma-streamer/src/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lib-2f-rs485-2d-bus-2d-communication-2f-src

clean-Lib-2f-rs485-2d-bus-2d-communication-2f-src:
	-$(RM) ./Lib/rs485-bus-communication/src/event_handler.d ./Lib/rs485-bus-communication/src/event_handler.o ./Lib/rs485-bus-communication/src/event_handler.su ./Lib/rs485-bus-communication/src/master.d ./Lib/rs485-bus-communication/src/master.o ./Lib/rs485-bus-communication/src/master.su ./Lib/rs485-bus-communication/src/package.d ./Lib/rs485-bus-communication/src/package.o ./Lib/rs485-bus-communication/src/package.su ./Lib/rs485-bus-communication/src/rs485-bus.d ./Lib/rs485-bus-communication/src/rs485-bus.o ./Lib/rs485-bus-communication/src/rs485-bus.su ./Lib/rs485-bus-communication/src/slave.d ./Lib/rs485-bus-communication/src/slave.o ./Lib/rs485-bus-communication/src/slave.su

.PHONY: clean-Lib-2f-rs485-2d-bus-2d-communication-2f-src

