################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/001LEDToggle.c \
../src/syscalls.c \
../src/sysmem.c 

OBJS += \
./src/001LEDToggle.o \
./src/syscalls.o \
./src/sysmem.o 

C_DEPS += \
./src/001LEDToggle.d \
./src/syscalls.d \
./src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../inc -I"/home/jack/Documents/mcu1course/stm32f4xx_drivers/drivers/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
