################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Application/BatteryChargeAsker.cpp \
../Application/CollisionAvoidance.cpp \
../Application/CollisionHandler.cpp \
../Application/HyroMotor.cpp \
../Application/InitialisationList.cpp \
../Application/LEDStrip.cpp \
../Application/MovConvert.cpp \
../Application/MovementControl.cpp \
../Application/ReadEncoders.cpp \
../Application/UARTtoRS485.cpp \
../Application/UARTuserInit.cpp \
../Application/USBUserInterface.cpp 

OBJS += \
./Application/BatteryChargeAsker.o \
./Application/CollisionAvoidance.o \
./Application/CollisionHandler.o \
./Application/HyroMotor.o \
./Application/InitialisationList.o \
./Application/LEDStrip.o \
./Application/MovConvert.o \
./Application/MovementControl.o \
./Application/ReadEncoders.o \
./Application/UARTtoRS485.o \
./Application/UARTuserInit.o \
./Application/USBUserInterface.o 

CPP_DEPS += \
./Application/BatteryChargeAsker.d \
./Application/CollisionAvoidance.d \
./Application/CollisionHandler.d \
./Application/HyroMotor.d \
./Application/InitialisationList.d \
./Application/LEDStrip.d \
./Application/MovConvert.d \
./Application/MovementControl.d \
./Application/ReadEncoders.d \
./Application/UARTtoRS485.d \
./Application/UARTuserInit.d \
./Application/USBUserInterface.d 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o: ../Application/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin  -g -DARM_MATH_CM4 -D__FPU_USED -D__FPU_PRESENT -DHSE_VALUE=8000000 -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\Application" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\wrapper" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\FreeRtos\include" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\CMSIS" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\core" -I"C:\Users\Taras.Melnik\Documents\eclipse-workspace\SkCR\stm32f4\SPL\inc" -std=gnu++11 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


