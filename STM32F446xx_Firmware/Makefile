CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
MACH = cortex-m4
MCU = stm32f446xx

CFLAGS = -c -g -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0 -I $(INC)
LDFLAGS = -mcpu=$(MACH) -mthumb  --specs=nano.specs -T $(LD_SCRIPT)
DEPFLAGS = -MMD -MP

# List of directories
LD_SCRIPT = startup/$(MCU).ld
INC = drivers/inc
SRC_DIR = src
DRIVERS_DIR = drivers/src
STARTUP_DIR = startup
OBJ_DIR = .build
BINDIR = .bin
ELF = $(BINDIR)/final.elf
HEX = $(BINDIR)/final.hex

# List of source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
DRIVERS_SRCS := $(wildcard $(DRIVERS_DIR)/*.c)
STARTUP_SRCS := $(wildcard $(STARTUP_DIR)/*.c)

# List of object files
OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS)) \
       $(patsubst $(DRIVERS_DIR)/%.c,$(OBJ_DIR)/%.o,$(DRIVERS_SRCS)) \
       $(patsubst $(STARTUP_DIR)/%.c,$(OBJ_DIR)/%.o,$(STARTUP_SRCS))

# List of dependency files
DEPS = $(OBJS:.o=.d)

all: $(HEX)

# Rule to generate Hex file from ELF file
$(HEX): $(ELF)
	@echo Generating Hex.....
	@mkdir -p $(BINDIR)
	$(OBJCOPY) -O ihex $< $@
	@echo Build complete.

# Rule to generate ELF file from object file
$(ELF): $(OBJS)
	@echo Linking......
	@mkdir -p $(BINDIR)
	$(CC) $(LDFLAGS) -o $@ $^
	@echo Linking complete.

# Rule to compile source files in src directory
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo Compiling $<......
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) $< -o $@

# Rule to compile source files in drivers/src directory
$(OBJ_DIR)/%.o: $(DRIVERS_DIR)/%.c
	@echo Compiling $<......
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) $< -o $@

# Rule to compile source files in startup directory
$(OBJ_DIR)/%.o: $(STARTUP_DIR)/%.c
	@echo Compiling $<......
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) $< -o $@



# Include the dependency files
-include $(DEPS)

flash:
	openocd -f openocd.cfg -c "program $(HEX) verify reset exit"

.PHONY: all clean flash

clean:
	@rm -rf $(OBJ_DIR) $(BINDIR)
