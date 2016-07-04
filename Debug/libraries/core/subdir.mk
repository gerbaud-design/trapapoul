################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/core/WInterrupts.c \
../libraries/core/hooks.c \
../libraries/core/wiring.c \
../libraries/core/wiring_analog.c \
../libraries/core/wiring_digital.c \
../libraries/core/wiring_pulse.c \
../libraries/core/wiring_shift.c 

CPP_SRCS += \
../libraries/core/CDC.cpp \
../libraries/core/HID.cpp \
../libraries/core/HardwareSerial.cpp \
../libraries/core/HardwareSerial0.cpp \
../libraries/core/HardwareSerial1.cpp \
../libraries/core/HardwareSerial2.cpp \
../libraries/core/HardwareSerial3.cpp \
../libraries/core/IPAddress.cpp \
../libraries/core/Print.cpp \
../libraries/core/Stream.cpp \
../libraries/core/Tone.cpp \
../libraries/core/USBCore.cpp \
../libraries/core/WMath.cpp \
../libraries/core/WString.cpp \
../libraries/core/abi.cpp \
../libraries/core/main.cpp \
../libraries/core/new.cpp 

S_UPPER_SRCS += \
../libraries/core/wiring_pulse.S 

C_DEPS += \
./libraries/core/WInterrupts.d \
./libraries/core/hooks.d \
./libraries/core/wiring.d \
./libraries/core/wiring_analog.d \
./libraries/core/wiring_digital.d \
./libraries/core/wiring_pulse.d \
./libraries/core/wiring_shift.d 

OBJS += \
./libraries/core/CDC.o \
./libraries/core/HID.o \
./libraries/core/HardwareSerial.o \
./libraries/core/HardwareSerial0.o \
./libraries/core/HardwareSerial1.o \
./libraries/core/HardwareSerial2.o \
./libraries/core/HardwareSerial3.o \
./libraries/core/IPAddress.o \
./libraries/core/Print.o \
./libraries/core/Stream.o \
./libraries/core/Tone.o \
./libraries/core/USBCore.o \
./libraries/core/WInterrupts.o \
./libraries/core/WMath.o \
./libraries/core/WString.o \
./libraries/core/abi.o \
./libraries/core/hooks.o \
./libraries/core/main.o \
./libraries/core/new.o \
./libraries/core/wiring.o \
./libraries/core/wiring_analog.o \
./libraries/core/wiring_digital.o \
./libraries/core/wiring_pulse.o \
./libraries/core/wiring_shift.o 

S_UPPER_DEPS += \
./libraries/core/wiring_pulse.d 

CPP_DEPS += \
./libraries/core/CDC.d \
./libraries/core/HID.d \
./libraries/core/HardwareSerial.d \
./libraries/core/HardwareSerial0.d \
./libraries/core/HardwareSerial1.d \
./libraries/core/HardwareSerial2.d \
./libraries/core/HardwareSerial3.d \
./libraries/core/IPAddress.d \
./libraries/core/Print.d \
./libraries/core/Stream.d \
./libraries/core/Tone.d \
./libraries/core/USBCore.d \
./libraries/core/WMath.d \
./libraries/core/WString.d \
./libraries/core/abi.d \
./libraries/core/main.d \
./libraries/core/new.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/core/%.o: ../libraries/core/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\core" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Time" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\TimerOne" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\EnableInterrupt" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SPI" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SD" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Wire" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\LiquidCrystal_I2C" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\DS1337RTC" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/core/%.o: ../libraries/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\core" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Time" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\TimerOne" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\EnableInterrupt" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SPI" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\SD" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\Wire" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\LiquidCrystal_I2C" -I"F:\Eclipse workspaces\arduino workspace\trapapoul\libraries\DS1337RTC" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/core/%.o: ../libraries/core/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -g2 -gstabs -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


