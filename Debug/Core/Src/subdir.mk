################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CANbus.c \
../Core/Src/main.c \
../Core/Src/mode.c \
../Core/Src/report.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/ws2812.c 

OBJS += \
./Core/Src/CANbus.o \
./Core/Src/main.o \
./Core/Src/mode.o \
./Core/Src/report.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/ws2812.o 

C_DEPS += \
./Core/Src/CANbus.d \
./Core/Src/main.d \
./Core/Src/mode.d \
./Core/Src/report.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/ws2812.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/buffer-c/src" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/rs485-bus-communication/src/inc" -I"D:/Data Folder Fauqi/Data_collector_Integrated/Lib/smt32-dma-streamer/src/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CANbus.d ./Core/Src/CANbus.o ./Core/Src/CANbus.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mode.d ./Core/Src/mode.o ./Core/Src/mode.su ./Core/Src/report.d ./Core/Src/report.o ./Core/Src/report.su ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/ws2812.d ./Core/Src/ws2812.o ./Core/Src/ws2812.su

.PHONY: clean-Core-2f-Src

