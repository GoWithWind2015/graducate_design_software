################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/GPIO.c \
../src/main.c \
../src/stm32f10x_it.c \
../src/syscalls.c \
../src/system_stm32f10x.c 

OBJS += \
./src/GPIO.o \
./src/main.o \
./src/stm32f10x_it.o \
./src/syscalls.o \
./src/system_stm32f10x.o 

C_DEPS += \
./src/GPIO.d \
./src/main.d \
./src/stm32f10x_it.d \
./src/syscalls.d \
./src/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F107VCTx -DDEBUG -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -I"/home/hu/Pro_self/stm32/stm32f107vct6/StdPeriph_Driver/inc" -I"/home/hu/Pro_self/stm32/stm32f107vct6/inc" -I"/home/hu/Pro_self/stm32/stm32f107vct6/CMSIS/device" -I"/home/hu/Pro_self/stm32/stm32f107vct6/CMSIS/core" -I"/home/hu/Pro_self/stm32/stm32f107vct6/decadriver" -I"/home/hu/Pro_self/stm32/stm32f107vct6/platform" -I"/home/hu/Pro_self/stm32/stm32f107vct6/src" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


