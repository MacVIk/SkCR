################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32f4/core/arm_common_tables.c \
../stm32f4/core/arm_cos_f32.c \
../stm32f4/core/arm_sin_f32.c \
../stm32f4/core/stm32f4xx_it.c 

OBJS += \
./stm32f4/core/arm_common_tables.o \
./stm32f4/core/arm_cos_f32.o \
./stm32f4/core/arm_sin_f32.o \
./stm32f4/core/stm32f4xx_it.o 

C_DEPS += \
./stm32f4/core/arm_common_tables.d \
./stm32f4/core/arm_cos_f32.d \
./stm32f4/core/arm_sin_f32.d \
./stm32f4/core/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f4/core/%.o: ../stm32f4/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin  -g -DARM_MATH_CM4 -D__FPU_USED -D__FPU_PRESENT -DHSE_VALUE=8000000 -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\Application" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\wrapper" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\include" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\CMSIS" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\core" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\SPL\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


