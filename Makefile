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

.PHONY: all sdsim sd1 sd4 test CM4 CM7 sd1CM4 sd4CM4 fullCM7 testCM4 testCM7 flash clean

all: CM4 CM7

sdsim: sdsimCM4 CM7
	$(MAKE) -C CM4 flashsdsim
	$(MAKE) -C CM7 flash

sd1: sd1CM4 CM7
	$(MAKE) -C CM4 flashsd1
	$(MAKE) -C CM7 flash

sd4: sd4CM4 fullCM7
	$(MAKE) -C CM4 flashsd4
	$(MAKE) -C CM7 flash

test: testCM4 testCM7
	$(MAKE) -C CM4 flashtest
	$(MAKE) -C CM7 flashtest

CM4:
	$(MAKE) -C CM4 all

CM7:
	$(MAKE) -C CM7 all

sdsimCM4:
	$(MAKE) -C CM4 sdsim

sd1CM4:
	$(MAKE) -C CM4 sd1

sd4CM4:
	$(MAKE) -C CM4 sd4

fullCM7:
	$(MAKE) -C CM7 full

testCM4:
	$(MAKE) -C CM4 test

testCM7:
	$(MAKE) -C CM7 test

flash:
	$(MAKE) -C CM4 flash
	$(MAKE) -C CM7 flash

clean:
	$(MAKE) -C CM4 clean
	$(MAKE) -C CM7 clean
