################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f051xx_gpio_driver.c \
../Drivers/Src/stm32f051xx_spi_driver.c 

OBJS += \
./Drivers/Src/stm32f051xx_gpio_driver.o \
./Drivers/Src/stm32f051xx_spi_driver.o 

C_DEPS += \
./Drivers/Src/stm32f051xx_gpio_driver.d \
./Drivers/Src/stm32f051xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f051xx_gpio_driver.o: ../Drivers/Src/stm32f051xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F0DISCOVERY -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"E:/Electronics/Discovery_projects/stm32f0xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f051xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f051xx_spi_driver.o: ../Drivers/Src/stm32f051xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F0DISCOVERY -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"E:/Electronics/Discovery_projects/stm32f0xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f051xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

