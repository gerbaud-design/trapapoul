################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/Wire/utility/twi.c 

C_DEPS += \
./libraries/Wire/utility/twi.d 

OBJS += \
./libraries/Wire/utility/twi.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/Wire/utility/%.o: ../libraries/Wire/utility/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\core" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Time" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\TimerOne" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\EnableInterrupt" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SPI" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SD" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Wire" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\LiquidCrystal_I2C" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\DS1337RTC" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


