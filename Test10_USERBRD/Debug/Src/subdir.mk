################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../Src/_exit.o \
../Src/main.o \
../Src/stm32f3xx_hal_msp.o \
../Src/stm32f3xx_it.o \
../Src/syscall.o \
../Src/system_stm32f3xx.o 

C_SRCS += \
../Src/_exit.c \
../Src/main.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/syscall.c \
../Src/system_stm32f3xx.c 

OBJS += \
./Src/_exit.o \
./Src/main.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/syscall.o \
./Src/system_stm32f3xx.o 

C_DEPS += \
./Src/_exit.d \
./Src/main.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/syscall.d \
./Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F303x8 -I"/home/wako/workspace/f3/Test10_USERBRD/Inc" -I"/home/wako/workspace/f3/Test10_USERBRD/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/wako/workspace/f3/Test10_USERBRD/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/wako/workspace/f3/Test10_USERBRD/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/wako/workspace/f3/Test10_USERBRD/Drivers/CMSIS/Include" -I"/home/wako/workspace/f3/Test10_USERBRD/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


