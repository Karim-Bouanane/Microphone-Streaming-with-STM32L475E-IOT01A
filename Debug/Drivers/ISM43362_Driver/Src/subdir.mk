################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ISM43362_Driver/Src/es_wifi.c \
../Drivers/ISM43362_Driver/Src/es_wifi_io.c \
../Drivers/ISM43362_Driver/Src/wifi.c 

OBJS += \
./Drivers/ISM43362_Driver/Src/es_wifi.o \
./Drivers/ISM43362_Driver/Src/es_wifi_io.o \
./Drivers/ISM43362_Driver/Src/wifi.o 

C_DEPS += \
./Drivers/ISM43362_Driver/Src/es_wifi.d \
./Drivers/ISM43362_Driver/Src/es_wifi_io.d \
./Drivers/ISM43362_Driver/Src/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ISM43362_Driver/Src/%.o Drivers/ISM43362_Driver/Src/%.su Drivers/ISM43362_Driver/Src/%.cyclo: ../Drivers/ISM43362_Driver/Src/%.c Drivers/ISM43362_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/ISM43362_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Drivers/Mongoose/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ISM43362_Driver-2f-Src

clean-Drivers-2f-ISM43362_Driver-2f-Src:
	-$(RM) ./Drivers/ISM43362_Driver/Src/es_wifi.cyclo ./Drivers/ISM43362_Driver/Src/es_wifi.d ./Drivers/ISM43362_Driver/Src/es_wifi.o ./Drivers/ISM43362_Driver/Src/es_wifi.su ./Drivers/ISM43362_Driver/Src/es_wifi_io.cyclo ./Drivers/ISM43362_Driver/Src/es_wifi_io.d ./Drivers/ISM43362_Driver/Src/es_wifi_io.o ./Drivers/ISM43362_Driver/Src/es_wifi_io.su ./Drivers/ISM43362_Driver/Src/wifi.cyclo ./Drivers/ISM43362_Driver/Src/wifi.d ./Drivers/ISM43362_Driver/Src/wifi.o ./Drivers/ISM43362_Driver/Src/wifi.su

.PHONY: clean-Drivers-2f-ISM43362_Driver-2f-Src

