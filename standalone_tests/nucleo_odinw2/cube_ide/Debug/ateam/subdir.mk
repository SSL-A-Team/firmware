################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ateam/OdinW2Radio.cpp \
../ateam/Robot.cpp \
../ateam/ateam_entry.cpp 

OBJS += \
./ateam/OdinW2Radio.o \
./ateam/Robot.o \
./ateam/ateam_entry.o 

CPP_DEPS += \
./ateam/OdinW2Radio.d \
./ateam/Robot.d \
./ateam/ateam_entry.d 


# Each subdirectory must supply rules for building sources it contributes
ateam/%.o: ../ateam/%.cpp ateam/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -D__cplusplus -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../USB_DEVICE/App -I../ateam -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

