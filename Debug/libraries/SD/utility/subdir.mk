################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../libraries/SD/utility/Sd2Card.cpp \
../libraries/SD/utility/SdFile.cpp \
../libraries/SD/utility/SdVolume.cpp 

OBJS += \
./libraries/SD/utility/Sd2Card.o \
./libraries/SD/utility/SdFile.o \
./libraries/SD/utility/SdVolume.o 

CPP_DEPS += \
./libraries/SD/utility/Sd2Card.d \
./libraries/SD/utility/SdFile.d \
./libraries/SD/utility/SdVolume.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/SD/utility/%.o: ../libraries/SD/utility/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\core" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SPI" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Wire" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\RTC-MCP7940" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SD" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\LiquidCrystal" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\EnableInterrupt" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


