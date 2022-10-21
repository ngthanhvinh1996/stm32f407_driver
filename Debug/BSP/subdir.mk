################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/ds1307.c \
../BSP/lcd.c 

OBJS += \
./BSP/ds1307.o \
./BSP/lcd.o 

C_DEPS += \
./BSP/ds1307.d \
./BSP/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o: ../BSP/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"/home/vinhkuto/Desktop/Project STM32/Pro_CreateProject/stm32f407xx_driver/Drivers/Inc" -I"/home/vinhkuto/Desktop/Project STM32/Pro_CreateProject/stm32f407xx_driver/BSP" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


