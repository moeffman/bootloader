# ================================
# Project Configuration
# ================================
TARGET     = bootloader
LINKER     = ../libs/linkers/stm32g071rb_bootloader.ld
SRC_DIR    = src
INC_DIR    = inc
BUILD_DIR  = build

STARTUP    = ../libs/stm32g071rb/src/startup_stm32g071rb.c
SYSCALLS   = ../libs/stm32g071rb/src/syscalls.c

# ================================
# Toolchain
# ================================
CC       = arm-none-eabi-gcc
OBJCOPY  = arm-none-eabi-objcopy

# ================================
# Compilation Flags
# ================================

# Architecture / Target
MCUFLAGS   = -mcpu=cortex-m0plus -mthumb

# Required to avoid linking newlib/libc
CORE_CFLAGS = -nostdlib -nostartfiles

# Optional (can toggle for release builds)
DEBUGFLAGS  = -g -Wall -Wpedantic -fstack-usage

# Include directories
INCLUDES    = -I$(INC_DIR)

# Spec files for nano-lib and no syscalls
SPECS       = -specs=nosys.specs -specs=nano.specs

# Final C compiler flags
CFLAGS      = $(MCUFLAGS) $(CORE_CFLAGS) $(DEBUGFLAGS) $(INCLUDES) $(SPECS)

# Linker script
LDFLAGS     = -T $(LINKER)

# ================================
# Source Files and Objects
# ================================
SRCS = $(wildcard $(SRC_DIR)/*.c)

OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS)) \
       $(BUILD_DIR)/startup_stm32g071rb.o \
       $(BUILD_DIR)/syscalls.o

# ================================
# Build Rules
# ================================

# Default target
all: $(BUILD_DIR) $(TARGET).elf $(TARGET).bin

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo Compiling $<...

# Compile startup file
$(BUILD_DIR)/startup_stm32g071rb.o: $(STARTUP)
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo Compiling startup file $<...

# Compile syscalls file
$(BUILD_DIR)/syscalls.o: $(SYSCALLS)
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo Compiling syscalls file $<...

# Link everything into the ELF
$(TARGET).elf: $(OBJS)
	@$(CC) $(MCUFLAGS) $(DEBUGFLAGS) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
	@echo Linking complete!

# Convert ELF to binary and hex formats
$(TARGET).bin: $(TARGET).elf
	@$(OBJCOPY) -O binary $< $(TARGET).bin
	@$(OBJCOPY) -O ihex $< $(TARGET).hex
	@echo Success!
	@echo Created: $(TARGET).bin, $(TARGET).hex, $(TARGET).elf

# Display binary size
size: $(TARGET).elf
	@arm-none-eabi-size $<

# Clean build artifacts
clean:
	@rm -rf $(BUILD_DIR) $(TARGET).elf $(TARGET).bin $(TARGET).hex
	@echo Cleaned up build files.

# Mark phony targets
.PHONY: all clean size
