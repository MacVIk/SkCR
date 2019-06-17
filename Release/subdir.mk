################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LibraryHacks.cpp \
../main.cpp 

S_UPPER_SRCS += \
../Startup.S 

OBJS += \
./LibraryHacks.o \
./Startup.o \
./main.o 

S_UPPER_DEPS += \
./Startup.d 

CPP_DEPS += \
./LibraryHacks.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin  -g -DARM_MATH_CM4 -D__FPU_USED -D__FPU_PRESENT -DHSE_VALUE=8000000 -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\Application" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\wrapper" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\include" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\CMSIS" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\core" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\SPL\inc" -std=gnu++11 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin  -g -x assembler-with-cpp -DSTM32F407xx -D__FPU_USED -D__FPU_PRESENT -DSTM32F4XX -DUSE_STM32F4_DISCOVERY -DARM_MATH_CM4 -D__HEAP_SIZE=0x1000 -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\CMSIS" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\core" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\SPL" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


