################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Mongoose/mongoose.c 

OBJS += \
./Drivers/Mongoose/mongoose.o 

C_DEPS += \
./Drivers/Mongoose/mongoose.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Mongoose/%.o Drivers/Mongoose/%.su Drivers/Mongoose/%.cyclo: ../Drivers/Mongoose/%.c Drivers/Mongoose/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/ISM43362_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Drivers/Mongoose/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Mongoose

clean-Drivers-2f-Mongoose:
	-$(RM) ./Drivers/Mongoose/mongoose.cyclo ./Drivers/Mongoose/mongoose.d ./Drivers/Mongoose/mongoose.o ./Drivers/Mongoose/mongoose.su

.PHONY: clean-Drivers-2f-Mongoose

