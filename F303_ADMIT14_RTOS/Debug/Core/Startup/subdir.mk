################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f303k8tx.s 

OBJS += \
./Core/Startup/startup_stm32f303k8tx.o 

S_DEPS += \
./Core/Startup/startup_stm32f303k8tx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I../Core/Inc -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Drivers/STM32F3xx_HAL_Driver/Inc -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Drivers/CMSIS/Device/ST/STM32F3xx/Include -IC:/Users/Minh/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.5/Drivers/CMSIS/Include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f303k8tx.d ./Core/Startup/startup_stm32f303k8tx.o

.PHONY: clean-Core-2f-Startup

