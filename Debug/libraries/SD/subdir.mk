################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../libraries/SD/File.cpp \
../libraries/SD/SD.cpp 

OBJS += \
./libraries/SD/File.o \
./libraries/SD/SD.o 

CPP_DEPS += \
./libraries/SD/File.d \
./libraries/SD/SD.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/SD/%.o: ../libraries/SD/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\core" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SPI" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Wire" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\RTC-MCP7940" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SD" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\LiquidCrystal" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\EnableInterrupt" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


