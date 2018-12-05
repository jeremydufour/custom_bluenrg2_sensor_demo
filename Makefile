GCCDIR := ../../../../../Program\ Files\ \(x86\)/GNU\ Tools\ ARM\ Embedded/7\ 2017-q4-major/bin/

SDK_ROOT := ./BlueNRG_DK_2.6.0
PROJ_DIR := .

# MODULE_BLE_DIR := ./module-ble_management

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf \"$(1)\""
endif

OBJDIR := _build
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ..

# $(OUTPUT_DIRECTORY)/steval-idb800vx.out: \

PROJECT := custom_ble_sensor_demo

# Source files common to all targets
OBJECTS += \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_adc.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_dma.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_flash.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_gpio.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_i2c.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_mft.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_pka.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_radio.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_rng.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_rtc.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_spi.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_sysCtrl.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_uart.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_wdg.o \
  $(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src/misc.o \
  $(SDK_ROOT)/Library/Bluetooth_LE/OTA/src/OTA_btl.o \
  $(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/profiles/ble_profiles/master_events_CB.o \
  $(SDK_ROOT)/Library/Bluetooth_LE/src/ble_utils.o \
  $(SDK_ROOT)/Library/CMSIS/Device/ST/BlueNRG1/Source/system_bluenrg1.o \
  $(SDK_ROOT)/Library/hal/src/clock.o \
  $(SDK_ROOT)/Library/hal/src/context_switch.o \
  $(SDK_ROOT)/Library/hal/src/fifo.o \
  $(SDK_ROOT)/Library/hal/src/gp_timer.o \
  $(SDK_ROOT)/Library/hal/src/hal_radio.o \
  $(SDK_ROOT)/Library/hal/src/miscutil.o \
  $(SDK_ROOT)/Library/hal/src/osal.o \
  $(SDK_ROOT)/Library/hal/src/sleep.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/HTS221_DRIVER.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/HTS221_HAL.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/LPS25HB.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/LSM6DS3.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/SDK_EVAL_Button.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/SDK_EVAL_Com.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/SDK_EVAL_Config.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/SDK_EVAL_I2C.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/SDK_EVAL_Led.o \
  $(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src/SDK_EVAL_SPI.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/src/SDK_EVAL_GPIO.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/src/SDK_EVAL_Spi.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/src/SDK_EVAL_Uart.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/src/SDK_Eval_Com.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/src/SDK_Eval_Config.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/src/SDK_Eval_Led.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/src/SDK_Eval_Timer.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_comp.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_crc.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cryp_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cryp.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_dac_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_dac.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_dma.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_flash_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_flash_ramfunc.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_flash.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_i2c.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_irda.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_iwdg.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_lcd.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_msp_template.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_nor.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_opamp_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_opamp.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pcd_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pcd.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_sd.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_smartcard.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_sram.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_tim_ex.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_tim.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_uart.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_wwdg.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_ll_fsmc.o \
  $(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_ll_sdmmc.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src/clock.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src/gp_timer.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src/list.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src/low_power.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src/osal.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller/bluenrg1_gap_aci.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller/bluenrg1_gatt_aci.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller/bluenrg1_hal_aci.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller/bluenrg1_hci_le.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller/bluenrg1_l2cap_aci.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/bluenrg1_events.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/bluenrg1_hci_le.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/hci.o \
  $(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/STM32L1xx_HAL_BlueNRG1_Drivers/src/stm32l1xx_hal_bluenrg1_spi.o \
  $(PROJ_DIR)/BlueNRG1_it.o \
  $(PROJ_DIR)/gatt_db.o \
  $(PROJ_DIR)/sensor.o \
  $(PROJ_DIR)/SensorDemo_main.o



# Include folders common to all targets
INC_FOLDERS += -I$(SDK_ROOT)
INC_FOLDERS += -I$(SDK_ROOT)/Library
INC_FOLDERS += -I$(SDK_ROOT)/Library/BlueNRG1_Periph_Driver
INC_FOLDERS += -I$(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/inc
# INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/library
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/OTA
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/OTA/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/OTA/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/includes
# INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/library
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/profiles/ble_profiles
INC_FOLDERS += -I$(SDK_ROOT)/Library/Bluetooth_LE/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/CMSIS/Device/ST/BlueNRG1/Include
INC_FOLDERS += -I$(SDK_ROOT)/Library/CMSIS/Device/ST/BlueNRG1/Source
INC_FOLDERS += -I$(SDK_ROOT)/Library/CMSIS/Include
INC_FOLDERS += -I$(SDK_ROOT)/Library/hal/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/hal/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/CMSIS/Device/ST/STM32L1xx/Include
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/includes
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/STM32L1xx_HAL_BlueNRG1_Drivers/inc
INC_FOLDERS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/STM32L1xx_HAL_BlueNRG1_Drivers/src
INC_FOLDERS += -I$(PROJ_DIR)/
INC_FOLDERS += -I$(PROJ_DIR)/includes

# $(SDK_ROOT)/Library/Bluetooth_LE/library \
# $(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/library \

# Libraries common to all targets
LIB_FILES += \
  $(SDK_ROOT)/Library/Bluetooth_LE/library/libbluenrg1_stack.a \
  $(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/library/libmaster_library_bluenrg1.a

LIBRARY_PATHS += \
  $(SDK_ROOT)/Library/Bluetooth_LE/library \
  $(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/library

LIBRARIES += -llibbluenrg1_stack -llibmaster_library_bluenrg1

LINKER_SCRIPT := BlueNRG2.ld

AS  = arm-none-eabi-gcc
CC  = arm-none-eabi-gcc
CPP = arm-none-eabi-g++
AR  = arm-none-eabi-ar
ELF2BIN = arm-none-eabi-objcopy
LD      = arm-none-eabi-gcc
PREPROC = arm-none-eabi-cpp

# Optimization flags
OPT = -O2 -g
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += -std=c99
CFLAGS += -DUSE_FULL_ASSERT
CFLAGS += $(OPT)
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb
CFLAGS += -Wall -Werror
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fstack-usage
CFLAGS += -include

# C++ flags common to all targets
CXXFLAGS += $(OPT)
CXXFLAGS += -include

# Assembler flags common to all targets
ASMFLAGS += -x
ASMFLAGS += -g
ASMFLAGS += -mcpu=cortex-m0
ASMFLAGS += -mthumb
ASMFLAGS += assembler-with-cpp
ASMFLAGS += -I$(SDK_ROOT)
ASMFLAGS += -I$(SDK_ROOT)/Library
ASMFLAGS += -I$(SDK_ROOT)/Library/BlueNRG1_Periph_Driver
ASMFLAGS += -I$(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/BlueNRG1_Periph_Driver/src
ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/inc
# ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/library
ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/OTA/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/OTA/src
ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/includes
# ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/library
ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/Profile_Framework_Central/profiles/ble_profiles
ASMFLAGS += -I$(SDK_ROOT)/Library/Bluetooth_LE/src
ASMFLAGS += -I$(SDK_ROOT)/Library/CMSIS/Device/ST/BlueNRG1/Include
ASMFLAGS += -I$(SDK_ROOT)/Library/CMSIS/Device/ST/BlueNRG1/Source
ASMFLAGS += -I$(SDK_ROOT)/Library/CMSIS/Include
ASMFLAGS += -I$(SDK_ROOT)/Library/hal/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/hal/src
ASMFLAGS += -I$(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/SDK_Eval_BlueNRG1/src
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_BlueNRG1/src
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/BSP/STM32L1xx_Nucleo/src
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/CMSIS/Device/ST/STM32L1xx/Include
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Inc
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Drivers/STM32L1xx_HAL_Driver/Src
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/HAL/src
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci/controller
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/hci
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/SimpleBlueNRG1_HCI/includes
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/STM32L1xx_HAL_BlueNRG1_Drivers/inc
ASMFLAGS += -I$(SDK_ROOT)/Library/STM32L/Middlewares/ST/STM32_BlueNRG1/STM32L1xx_HAL_BlueNRG1_Drivers/src
ASMFLAGS += -I$(PROJ_DIR)/
ASMFLAGS += -I$(PROJ_DIR)/includes


# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb
LDFLAGS += -mcpu=cortex-m0
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs


# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LD_LIB += -lc -lnosys -lm -llibmaster_library_bluenrg1 -llibbluenrg1_stack


# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"

	@$(AS) -c $(ASMFLAGS) -o $@ $<


# .S.o:
# 	+@$(call MAKEDIR,$(dir $@))
# 	+@echo "Assemble: $(notdir $<)"

# 	@$(AS) -c $(ASMFLAGS) -o $@ $<


.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(CFLAGS) $(INC_FOLDERS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXXFLAGS) $(INC_FOLDERS) -o $@ $<


$(PROJECT).link_script.ld: $(LINKER_SCRIPT)
	@$(PREPROC) $< -o $@



$(PROJECT).elf: $(OBJECTS) $(PROJECT).link_script.ld
	+@echo "link: $(notdir $@)"
	@$(LD) $(LDFLAGS) -T $(filter-out %.o, $^) $(LIBRARY_PATHS) --output $@ $(filter %.o, $^) $(LIBRARIES) $(LD_LIB)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ ====="

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@


# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
