# ================================
# Project Configuration
# ================================
TARGET     = bootloader
LINKER     = bootloader.ld
SRC_DIR    = src
INC_DIR    = inc
BUILD_DIR  = build

# ================================
# Toolchain
# ================================
CC       = arm-none-eabi-gcc
OBJCOPY  = arm-none-eabi-objcopy

# ================================
# Compilation Flags
# ================================
MCUFLAGS     = -mcpu=cortex-m0plus -mthumb
CORE_CFLAGS  = -nostdlib -nostartfiles
DEBUGFLAGS   = -g -Wall -Wpedantic -Werror -fstack-usage
INCLUDES     = -I$(INC_DIR)
OPTIMIZATION = -Os
SPECS        = -specs=nosys.specs -specs=nano.specs
CFLAGS       = $(MCUFLAGS) $(CORE_CFLAGS) $(DEBUGFLAGS) $(INCLUDES) $(SPECS) $(OPTIMIZATION)
LDFLAGS      = -T $(LINKER)

# ================================
# Source Files and Objects
# ================================
SRCS = $(wildcard $(SRC_DIR)/*.c) $(STARTUP) $(SYSCALLS)
OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(SRCS)))

# ================================
# Build Rules
# ================================
all: $(BUILD_DIR) $(TARGET).elf $(TARGET).bin

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo Compiling $<...

$(BUILD_DIR)/%.o: ../../libs/stm32g071rb/src/%.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo Compiling $<...

# Link everything
$(TARGET).elf: $(OBJS)
	@$(CC) $(MCUFLAGS) $(DEBUGFLAGS) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
	@echo Linking complete!

# Convert to binary and hex
$(TARGET).bin: $(TARGET).elf
	@$(OBJCOPY) -O binary $< $(TARGET).bin
	@$(OBJCOPY) -O ihex $< $(TARGET).hex
	@echo Success!
	@echo Created: $(TARGET).bin, $(TARGET).hex, $(TARGET).elf

# Size reporting
size: $(TARGET).elf
	@arm-none-eabi-size $<

# Clean all
clean:
	@rm -rf $(BUILD_DIR) $(TARGET).elf $(TARGET).bin $(TARGET).hex
	@echo Cleaned up build files.

# Mark phony targets
.PHONY: all clean size
