################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
F767_unit_1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F767_unit_1.list \

OBJCOPY_BIN += \
F767_unit_1.bin \


# All Target
all: F767_unit_1.elf secondary-outputs

# Tool invocations
F767_unit_1.elf: $(OBJS) $(USER_OBJS) C:\Users\fukuj\STM32CubeIDE\workspace_1.2.0\F767_unit_1\STM32F767ZITX_FLASH.ld
	arm-none-eabi-gcc -o "F767_unit_1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\fukuj\STM32CubeIDE\workspace_1.2.0\F767_unit_1\STM32F767ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F767_unit_1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F767_unit_1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F767_unit_1.list"
	@echo 'Finished building: $@'
	@echo ' '

F767_unit_1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "F767_unit_1.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
