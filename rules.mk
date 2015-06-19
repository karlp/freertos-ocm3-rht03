
BUILD_DIR   ?= bin

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q               := @
NULL            := 2>/dev/null
endif

PREFIX	    ?= arm-none-eabi-
CC	    = $(PREFIX)gcc
LD	    = $(PREFIX)gcc
OBJCOPY	    = $(PREFIX)objcopy
OBJDUMP	    = $(PREFIX)objdump

OPENCM3_DIR = ../../../libs/libopencm3
OPENCM3_INC = $(OPENCM3_DIR)/include
OPENCM3_DEFS = -DSTM32F1

# Inclusion of header files
INCLUDES += $(patsubst %,-I%, . $(OPENCM3_INC) )

CFLAGS += -Os -ggdb3 -std=c99
CFLAGS += -mcpu=cortex-m3 -msoft-float -mthumb
CFLAGS +=  $(INCLUDES) -fno-common -MD
CFLAGS += -Wall -Wextra -Wshadow -Wno-unused-variable -Wimplicit-function-declaration
CFLAGS += -Wredundant-decls -Wstrict-prototypes
#CFLAGS += -Wmissing-prototypes
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += $(OPENCM3_DEFS)
#CFLAGS += -DNDEBUG # squelch asserts in freemodbus lib

LDSCRIPT = $(OPENCM3_DIR)/lib/stm32/f1/stm32f100x8.ld

LDFLAGS += -I . -lc -T$(LDSCRIPT) -L$(OPENCM3_DIR)/lib -nostartfiles -Wl,--gc-sections
LDFLAGS += -lopencm3_stm32f1
LDFLAGS += -specs=nano.specs
# nosys is only in newer gcc-arm-embedded...
#LDFLAGS += -specs=nosys.specs
LDFLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LDFLAGS += -Wl,--undefined=uxTopUsedPriority
# OPTIONAL
#LDFLAGS += -Wl,-Map=$(PROJECT).map

OBJS = $(CFILES:%.c=$(BUILD_DIR)/%.o)

all: $(PROJECT).elf $(PROJECT).bin
flash: $(PROJECT).flash

# Need a special rule to have a bin dir
$(BUILD_DIR)/%.o: %.c
	@printf "  CC\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

$(PROJECT).elf: $(OBJS)
	@printf "  LD\t$@\n"
	$(Q)$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

%.bin: %.elf
	@printf "  OBJCOPY\t$@\n"
	$(Q)$(OBJCOPY) -O binary  $< $@

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

%.list: %.elf
	$(OBJDUMP) -S $< > $@

%.flash: %.elf
	@printf "  FLASH\t$<\n"
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		-f target/$(OOCD_TARGET).cfg \
		-c "program $(*).elf verify reset exit" \
		$(NULL)

clean:
	rm -rf $(BUILD_DIR) $(PROJECT).{elf,bin} $(PROJECT).{list,lss,map}

.PHONY: all clean
-include $(OBJS:.o=.d)

OOCD            ?= openocd
OOCD_INTERFACE  ?= stlink-v2-1
OOCD_TARGET     ?= stm32f1x

