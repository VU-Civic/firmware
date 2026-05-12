#### Target and Hardware Details ####

BUILD := bin
TARGET := CivicAlert_CM4
EXECUTABLE := $(BUILD)/$(TARGET).elf
MAP_FILE := $(BUILD)/$(TARGET).map
SIZE_OUTPUT := default.size.stdout
OBJDUMP_LIST := $(BUILD)/$(TARGET).list
LINKER_FILE := STM32H745ZITX_FLASH.ld

SD_TARGET := SD_$(TARGET)
SD_EXECUTABLE := $(BUILD)/$(SD_TARGET).elf

TEST_TARGET := Test_$(TARGET)
TEST_EXECUTABLE := $(BUILD)/$(TEST_TARGET).elf

TEST_TRANSMIT_TARGET := Test_Transmit_$(TARGET)
TEST_TRANSMIT_EXECUTABLE := $(BUILD)/$(TEST_TRANSMIT_TARGET).elf

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

DEFINES := -D_DATETIME=$(shell date +%s) -D_FW_REVISION=$(shell cd .. && git log --pretty=format:'%H' -n 1 | cut -c 1-8) -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_PWR_LDO_SUPPLY -DUSBD_MAX_POWER=250 -DBOARD_REV=$(BOARD_REV)

VPATH := ../Common/Src Core/Src Core/Startup ../Drivers/STM32H7xx_HAL_Driver/Src ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src ../Middlewares/ST/STM32_USB_Device_Library/Core/Src
INCLUDES := -I../Common/Inc -ICore/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS_DSP/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
SOURCES := $(filter-out main.c, $(foreach SOURCE,$(foreach DIR,$(VPATH),$(wildcard $(DIR)/*.c)),$(patsubst %.c,%.c,$(notdir $(SOURCE)))))
CMSIS_DSP_SOURCES := arm_copy_q7.c
VPATH += Testing ../Drivers/CMSIS_DSP/Source/BasicMathFunctions ../Drivers/CMSIS_DSP/Source/ComplexMathFunctions ../Drivers/CMSIS_DSP/Source/CommonTables ../Drivers/CMSIS_DSP/Source/FastMathFunctions ../Drivers/CMSIS_DSP/Source/SupportFunctions ../Drivers/CMSIS_DSP/Source/TransformFunctions
OBJECTS := $(SOURCES:%.c=$(BUILD)/%.o) $(CMSIS_DSP_SOURCES:%.c=$(BUILD)/%.o) $(BUILD)/startup_stm32h745zitx.o

CFLAGS  = -mcpu=$(CPU) -std=gnu11 $(DEFINES) $(INCLUDES) -ffunction-sections -fdata-sections -Wall -Wno-packed-bitfield-compat
CFLAGS += -fstack-usage -MMD -MP --specs=nano.specs -mfpu=$(FPU) -mfloat-abi=$(FABI) -mthumb
LFLAGS  = -mcpu=$(CPU) --specs=nosys.specs -Wl,-Map=$(MAP_FILE) -Wl,--no-warn-rwx-segments -Wl,--gc-sections -static
LFLAGS += --specs=nano.specs -mfpu=$(FPU) -mfloat-abi=$(FABI) -mthumb -Wl,--start-group -lc -lm -Wl,--end-group


#### Build Targets ####

all: CFLAGS += -Ofast
all: DEFINES += -DSIM_REQUIRED
all: directories $(EXECUTABLE) secondary-outputs

debug: CFLAGS += -g3 -DDEBUG -O0
debug: directories $(EXECUTABLE) secondary-outputs

sdsim: CFLAGS += -Ofast
sdsim: DEFINES += -DSIM_REQUIRED -DDISABLE_EVIDENCE_XMIT
sdsim: directories $(SD_EXECUTABLE)

sdsim4: CFLAGS += -Ofast
sdsim4: DEFINES += -DSIM_REQUIRED -DDISABLE_EVIDENCE_XMIT -DPACKET_FULL_AUDIO
sdsim4: directories $(SD_EXECUTABLE)

sd1: CFLAGS += -Ofast
sd1: DEFINES += -DDISABLE_EVIDENCE_XMIT
sd1: directories $(SD_EXECUTABLE)

sd4: CFLAGS += -Ofast
sd4: DEFINES += -DDISABLE_EVIDENCE_XMIT -DPACKET_FULL_AUDIO
sd4: directories $(SD_EXECUTABLE)

transmit: CFLAGS += -Ofast
transmit: DEFINES += -DSIM_REQUIRED
transmit: directories $(TEST_TRANSMIT_EXECUTABLE)

test: CFLAGS += -Ofast
test: DEFINES += -DPERIPHERAL_TESTS
test: directories $(TEST_EXECUTABLE)

directories: $(BUILD)

$(BUILD):
	-$(MKDIR) $@
	
secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

$(EXECUTABLE) $(MAP_FILE): $(OBJECTS) $(BUILD)/main.o
	$(CC) -o $@ -T$(LINKER_FILE) $^ $(LFLAGS)

$(SD_EXECUTABLE): $(OBJECTS) $(BUILD)/sdcard.o
	$(CC) -o $@ -T$(LINKER_FILE) $^ $(LFLAGS)

$(TEST_TRANSMIT_EXECUTABLE): $(OBJECTS) $(BUILD)/cell_transmit.o
	$(CC) -o $@ -T$(LINKER_FILE) $^ $(LFLAGS)

$(TEST_EXECUTABLE): $(OBJECTS) $(BUILD)/manufacturing.o
	$(CC) -o $@ -T$(LINKER_FILE) $^ $(LFLAGS)

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

flashsd:
	$(FLASH) -c port=SWD mode=UR -d $(SD_EXECUTABLE) 0x08100000 -v

flashtransmit: transmit
	$(FLASH) -c port=SWD mode=UR -d $(TEST_TRANSMIT_EXECUTABLE) 0x08100000 -v

flashtest: test
	$(FLASH) -c port=SWD mode=UR -d $(TEST_EXECUTABLE) 0x08100000 -v

.PHONY: all debug sdsim sdsim4 sd1 sd4 transmit test directories secondary-outputs clean flash flashsd flashtransmit flashtest
