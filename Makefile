FLASH := STM32_Programmer_CLI
OBJCOPY := arm-none-eabi-objcopy
PRINT := @echo
COPY := @cp -a
RM := @rm -rf

ifndef STM32_PRG_PATH
ifeq ($(OS),Windows_NT)
STM32_PRG_PATH := $(dir $(shell where $(FLASH)))
else
STM32_PRG_PATH := $(dir $(shell which $(FLASH)))
endif
endif
ifndef STM32_PRG_PATH
$(error You must add $(FLASH) to the PATH or specify the STM32_PRG_PATH to continue)
endif

.PHONY: all CM4 CM7 flash clean

all: CM4 CM7

CM4:
	$(MAKE) -C CM4 all

CM7:
	$(MAKE) -C CM7 all

flash:
	$(MAKE) -C CM4 flash
	$(MAKE) -C CM7 flash

clean:
	$(MAKE) -C CM4 clean
	$(MAKE) -C CM7 clean
