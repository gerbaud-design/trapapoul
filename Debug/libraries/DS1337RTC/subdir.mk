################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../libraries/DS1337RTC/DS1337RTC.cpp 

OBJS += \
./libraries/DS1337RTC/DS1337RTC.o 

CPP_DEPS += \
./libraries/DS1337RTC/DS1337RTC.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/DS1337RTC/%.o: ../libraries/DS1337RTC/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\core" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Time" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\TimerOne" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\EnableInterrupt" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SPI" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SD" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Wire" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\LiquidCrystal_I2C" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\DS1337RTC" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


