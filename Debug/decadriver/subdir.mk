################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../decadriver/deca_device.c \
../decadriver/deca_params_init.c \
../decadriver/deca_range_tables.c 

OBJS += \
./decadriver/deca_device.o \
./decadriver/deca_params_init.o \
./decadriver/deca_range_tables.o 

C_DEPS += \
./decadriver/deca_device.d \
./decadriver/deca_params_init.d \
./decadriver/deca_range_tables.d 


# Each subdirectory must supply rules for building sources it contributes
decadriver/%.o: ../decadriver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F107VCTx -DDEBUG -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -I"/home/hu/Pro_self/stm32/stm32f107vct6/StdPeriph_Driver/inc" -I"/home/hu/Pro_self/stm32/stm32f107vct6/inc" -I"/home/hu/Pro_self/stm32/stm32f107vct6/CMSIS/device" -I"/home/hu/Pro_self/stm32/stm32f107vct6/CMSIS/core" -I"/home/hu/Pro_self/stm32/stm32f107vct6/decadriver" -I"/home/hu/Pro_self/stm32/stm32f107vct6/platform" -I"/home/hu/Pro_self/stm32/stm32f107vct6/src" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


