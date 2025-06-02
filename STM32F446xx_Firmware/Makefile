CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
ELFSIZE = arm-none-eabi-size

MACH = cortex-m4
MCU = stm32f446xx

CFLAGS = -c -g -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu99 -Wall -O2 -I $(INC)
LDFLAGS = -mcpu=$(MACH) -mthumb  --specs=nosys.specs --specs=nano.specs -T $(LD_SCRIPT) -Wl,-Map=$(MAP)
DEPFLAGS = -MD -MP

# List of directories
LD_SCRIPT = startup/$(MCU).ld
INC = drivers/inc
SRC_DIR = src
DRIVERS_DIR = drivers/src
STARTUP_DIR = startup
OBJ_DIR = .build
BIN_DIR = .bin
DEBUG_DIR = .debug
ELF = $(BIN_DIR)/final.elf
HEX = $(BIN_DIR)/final.hex
MAP = $(OBJ_DIR)/final.map

# List of source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
DRIVERS_SRCS := $(wildcard $(DRIVERS_DIR)/*.c)
STARTUP_SRCS := $(wildcard $(STARTUP_DIR)/*.c)

# List of object files
OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS)) \
       $(patsubst $(DRIVERS_DIR)/%.c,$(OBJ_DIR)/%.o,$(DRIVERS_SRCS)) \
       $(patsubst $(STARTUP_DIR)/%.c,$(OBJ_DIR)/%.o,$(STARTUP_SRCS))

-include $(OBJS:.o=.d)

all: $(HEX)

# Rule to generate Hex file from ELF file
$(HEX): $(ELF)
	@echo Generating Hex.....
	@mkdir -p $(BIN_DIR)
	$(OBJCOPY) -O ihex $< $@
	@echo Build complete.

# Rule to generate ELF file from object file
$(ELF): $(OBJS)
	@echo Linking......
	@mkdir -p $(BIN_DIR)
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

# Debug output target: ELF + individual object disassembly
debug: $(ELF)
	@echo Generating debug output...
	@mkdir -p $(DEBUG_DIR)

    # Full ELF disassembly with source
	$(OBJDUMP) -S $(ELF) > $(DEBUG_DIR)/final_elf.s

    # Per-object disassembly
	@for obj in $(OBJS); do \
		name=$$(basename $$obj .o); \
		$(OBJDUMP) -d $$obj > $(DEBUG_DIR)/$$name.s; \
	done
    # Map file (memory layout)
	@mkdir -p $(BIN_DIR)
	@cp $(MAP) $(DEBUG_DIR)/final.map

	@echo Debug files written to $(DEBUG_DIR)


size: $(ELF)
	@echo "Size information:"
	$(ELFSIZE) -B $(ELF)


flash:
	openocd -f openocd.cfg -c "program $(HEX) verify reset exit"

clean:
	@rm -rf $(OBJ_DIR) $(BIN_DIR) $(DEBUG_DIR)

.PHONY: all clean flash debug size