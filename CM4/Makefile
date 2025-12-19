#### Target and Hardware Details ####

BUILD := bin
TARGET := CivicAlert_CM4
EXECUTABLE := $(BUILD)/$(TARGET).elf
MAP_FILE := $(BUILD)/$(TARGET).map
SIZE_OUTPUT := default.size.stdout
OBJDUMP_LIST := $(BUILD)/$(TARGET).list
LINKER_FILE := STM32H745ZITX_FLASH.ld

ifndef BOARD_REV
$(error You must specify a BOARD_REV (A, B, etc) to continue)
endif

CPU := cortex-m4
FPU := fpv4-sp-d16
FABI := hard

CC := arm-none-eabi-gcc
SIZE := arm-none-eabi-size
OBJDUMP := arm-none-eabi-objdump
MKDIR := mkdir -p
RM := rm -rf
FLASH := STM32_Programmer_CLI

DEFINES := -D_DATETIME=$(shell date +%s) -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_PWR_LDO_SUPPLY -DHAVE_CONFIG_H -DOPUS_ARM_INLINE_ASM -DUSBD_MAX_POWER=250 -DBOARD_REV=$(BOARD_REV)

VPATH := ../Common/Src Core/Src Core/Startup ../Drivers/STM32H7xx_HAL_Driver/Src ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src ../Middlewares/ST/STM32_USB_Device_Library/Core/Src ../Middlewares/Third_Party/Opus/celt ../Middlewares/Third_Party/Opus/silk ../Middlewares/Third_Party/Opus/silk/float ../Middlewares/Third_Party/Opus/src
INCLUDES := -I../Common/Inc -ICore/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/Opus -I../Middlewares/Third_Party/Opus/include -I../Middlewares/Third_Party/Opus/celt -I../Middlewares/Third_Party/Opus/silk -I../Middlewares/Third_Party/Opus/silk/float -I../Middlewares/Third_Party/Opus/src
SOURCES := $(foreach SOURCE,$(foreach DIR,$(VPATH),$(wildcard $(DIR)/*.c)),$(patsubst %.c,%.c,$(notdir $(SOURCE))))
OBJECTS := $(SOURCES:%.c=$(BUILD)/%.o) $(BUILD)/startup_stm32h745zitx.o

CFLAGS  = -mcpu=$(CPU) -std=gnu11 $(DEFINES) $(INCLUDES) -ffunction-sections -fdata-sections -Wall -Wno-packed-bitfield-compat
CFLAGS += -fstack-usage -MMD -MP --specs=nano.specs -mfpu=$(FPU) -mfloat-abi=$(FABI) -mthumb
LFLAGS  = -mcpu=$(CPU) --specs=nosys.specs -Wl,-Map=$(MAP_FILE) -Wl,--gc-sections -static
LFLAGS += --specs=nano.specs -mfpu=$(FPU) -mfloat-abi=$(FABI) -mthumb -Wl,--start-group -lc -lm -Wl,--end-group


#### Build Targets ####

all: CFLAGS += -Ofast
all: directories $(EXECUTABLE) secondary-outputs

debug: CFLAGS += -g3 -DDEBUG -O0
debug: directories $(EXECUTABLE) secondary-outputs

directories: $(BUILD)

$(BUILD):
	-$(MKDIR) $@
	
secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

$(EXECUTABLE) $(MAP_FILE): $(OBJECTS)
	$(CC) -o $@ -T$(LINKER_FILE) $(OBJECTS) $(LFLAGS)

$(SIZE_OUTPUT): $(EXECUTABLE)
	$(SIZE) $(EXECUTABLE)

$(OBJDUMP_LIST): $(EXECUTABLE)
	$(OBJDUMP) -h -S $(EXECUTABLE) > $@

$(BUILD)/startup_stm32h745zitx.o: Core/Startup/startup_stm32h745zitx.s
	$(CC) -mcpu=$(CPU) -c -x assembler-with-cpp -MMD -MP --specs=nano.specs -mfpu=$(FPU) -mfloat-abi=$(FABI) -mthumb $< -o $@

$(BUILD)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	-$(RM) $(BUILD)

flash: all
	$(FLASH) -c port=SWD mode=UR -d $(EXECUTABLE) 0x08100000 -v

.PHONY: all directories secondary-outputs clean flash
