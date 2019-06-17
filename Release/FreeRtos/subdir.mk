################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRtos/croutine.c \
../FreeRtos/event_groups.c \
../FreeRtos/heap_3.c \
../FreeRtos/list.c \
../FreeRtos/port.c \
../FreeRtos/queue.c \
../FreeRtos/tasks.c \
../FreeRtos/timers.c 

OBJS += \
./FreeRtos/croutine.o \
./FreeRtos/event_groups.o \
./FreeRtos/heap_3.o \
./FreeRtos/list.o \
./FreeRtos/port.o \
./FreeRtos/queue.o \
./FreeRtos/tasks.o \
./FreeRtos/timers.o 

C_DEPS += \
./FreeRtos/croutine.d \
./FreeRtos/event_groups.d \
./FreeRtos/heap_3.d \
./FreeRtos/list.d \
./FreeRtos/port.d \
./FreeRtos/queue.d \
./FreeRtos/tasks.d \
./FreeRtos/timers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRtos/%.o: ../FreeRtos/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin  -g -DARM_MATH_CM4 -D__FPU_USED -D__FPU_PRESENT -DHSE_VALUE=8000000 -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\Application" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\wrapper" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\include" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\CMSIS" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\core" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\SPL\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


