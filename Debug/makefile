################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include libraries/core/subdir.mk
-include libraries/Wire/utility/subdir.mk
-include libraries/Wire/subdir.mk
-include libraries/TimerOne/subdir.mk
-include libraries/Time/subdir.mk
-include libraries/SPI/subdir.mk
-include libraries/SD/utility/subdir.mk
-include libraries/SD/subdir.mk
-include libraries/RTC-MCP7940/subdir.mk
-include libraries/LiquidCrystal/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
trapapoul.lss \

FLASH_IMAGE += \
trapapoul.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: trapapoul.elf secondary-outputs

# Tool invocations
trapapoul.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,trapapoul.map,--cref -mrelax -Wl,--gc-sections -mmcu=atmega328p -o "trapapoul.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

trapapoul.lss: trapapoul.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S trapapoul.elf  >"trapapoul.lss"
	@echo 'Finished building: $@'
	@echo ' '

trapapoul.hex: trapapoul.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex trapapoul.elf  "trapapoul.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: trapapoul.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p trapapoul.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(S_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) trapapoul.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
