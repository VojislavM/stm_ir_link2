################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ir_common.c \
../src/main.c \
../src/rc5_encode.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/ir_common.o \
./src/main.o \
./src/rc5_encode.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/ir_common.d \
./src/main.d \
./src/rc5_encode.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -DSTM32 -DSTM32F411RETx -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -I"C:/Users/vojis/Documents/stm_projects/stm_ir_link2/inc" -I"C:/Users/vojis/Documents/stm_projects/stm_ir_link2/CMSIS/core" -I"C:/Users/vojis/Documents/stm_projects/stm_ir_link2/CMSIS/device" -I"C:/Users/vojis/Documents/stm_projects/stm_ir_link2/HAL_Driver/Inc/Legacy" -I"C:/Users/vojis/Documents/stm_projects/stm_ir_link2/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


