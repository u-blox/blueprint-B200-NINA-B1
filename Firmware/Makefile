MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

SDK_ROOT := C:/sw/nRF5_SDK_11.0.0
PROJ_DIR := src
TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

GNU_PREFIX := arm-none-eabi

MK := mkdir
RM := rm -rf

VERBOSE := 1

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

fw_version := 1.2
fw_version_wo_dot := $(subst .,_,$(fw_version))
 
# Toolchain commands
CC              := $(GNU_PREFIX)-gcc
AS              := $(GNU_PREFIX)-as
AR              := $(GNU_PREFIX)-ar -r
LD              := $(GNU_PREFIX)-ld
NM              := $(GNU_PREFIX)-nm
OBJDUMP         := $(GNU_PREFIX)-objdump
OBJCOPY         := $(GNU_PREFIX)-objcopy
SIZE            := $(GNU_PREFIX)-size

UNAME := $(shell uname -a)
ifeq (MinGW, $(findstring MinGW, $(UNAME)))
	# Variables specific for building in MinGW (Eclipse)
	SREC_CAT = tools/srec_cat.exe
	NRF_UTIL = tools/nordic/nrfutil.exe
endif

ifeq (Linux, $(findstring Linux, $(UNAME)))
	# Variables specific for building on the gitlab CI machine
	SREC_CAT = srec_cat
	NRF_UTIL = nrfutil
endif

ifeq (, $(UNAME))
	$(error Unsupported platform!)
endif

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

SDK_TOOLCHAIN_PATH := $(SDK_ROOT)/components/toolchain/gcc/

#source common to all targets
C_SOURCE_FILES += \
$(PROJ_DIR)/main.c \
$(PROJ_DIR)/drivers/battery.c \
$(PROJ_DIR)/drivers/bmi160.c \
$(PROJ_DIR)/drivers/leds.c \
$(PROJ_DIR)/services/acc_srv.c \
$(PROJ_DIR)/services/gyro_srv.c \
$(PROJ_DIR)/services/led_srv.c \
$(PROJ_DIR)/services/temp_srv.c \
$(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
$(SDK_ROOT)/components/ble/ble_services/ble_bas/ble_bas.c \
$(SDK_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
$(SDK_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
$(SDK_ROOT)/components/ble/common/ble_advdata.c \
$(SDK_ROOT)/components/ble/common/ble_conn_params.c \
$(SDK_ROOT)/components/ble/common/ble_srv_common.c \
$(SDK_ROOT)/components/ble/device_manager/device_manager_peripheral.c \
$(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
$(SDK_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
$(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(SDK_ROOT)/components/drivers_nrf/hal/nrf_saadc.c \
$(SDK_ROOT)/components/drivers_nrf/pstorage/pstorage.c \
$(SDK_ROOT)/components/drivers_nrf/pwm/nrf_drv_pwm.c \
$(SDK_ROOT)/components/drivers_nrf/saadc/nrf_drv_saadc.c \
$(SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
$(SDK_ROOT)/components/libraries/bootloader_dfu/bootloader_util.c \
$(SDK_ROOT)/components/libraries/bootloader_dfu/dfu_app_handler.c \
$(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
$(SDK_ROOT)/components/libraries/led_softblink/led_softblink.c \
$(SDK_ROOT)/components/libraries/low_power_pwm/low_power_pwm.c \
$(SDK_ROOT)/components/libraries/timer/app_timer.c \
$(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
$(SDK_ROOT)/components/libraries/twi/app_twi.c \
$(SDK_ROOT)/components/libraries/util/app_error.c \
$(SDK_ROOT)/components/libraries/util/app_util_platform.c \
$(SDK_ROOT)/components/libraries/util/nrf_log.c \
$(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \
$(SDK_ROOT)/components/toolchain/system_nrf52.c \
$(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.s

#includes common to all targets
INC_PATHS += -I$(PROJ_DIR)
INC_PATHS += -I$(PROJ_DIR)/drivers
INC_PATHS += -I$(PROJ_DIR)/services
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_dis
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK_ROOT)/components/ble/common
INC_PATHS += -I$(SDK_ROOT)/components/ble/device_manager
INC_PATHS += -I$(SDK_ROOT)/components/device
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/config
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/gpiote
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pwm
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/saadc
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/twi_master
INC_PATHS += -I$(SDK_ROOT)/components/libraries/bootloader_dfu
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_section_vars
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fstorage
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fstorage/config
INC_PATHS += -I$(SDK_ROOT)/components/libraries/led_softblink
INC_PATHS += -I$(SDK_ROOT)/components/libraries/low_power_pwm
INC_PATHS += -I$(SDK_ROOT)/components/libraries/scheduler
INC_PATHS += -I$(SDK_ROOT)/components/libraries/timer
INC_PATHS += -I$(SDK_ROOT)/components/libraries/trace
INC_PATHS += -I$(SDK_ROOT)/components/libraries/twi
INC_PATHS += -I$(SDK_ROOT)/components/libraries/util
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s132/headers
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s132/headers/nrf52
INC_PATHS += -I$(SDK_ROOT)/components/toolchain
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(SDK_ROOT)/external/segger_rtt

OBJECT_DIRECTORY_DEMO = obj_demo
LISTING_DIRECTORY = bin
OUTPUT_BINARY_DIRECTORY = bin

DEMO_OUTPUT_NAME = appl

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY_DEMO) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#CFLAGS = -DBOARD_PCA10040
CFLAGS = -DBOARD_NINA_B1

#flags common to all targets
CFLAGS += -DNRF52
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DS132
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O0 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
CFLAGS += -DFW_VERSION=\"$(fw_version)\"
CFLAGS += -D__HEAP_SIZE=0

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T nina_b1.ld
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_30
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DNRF52_PAN_53
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DS132
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63
ASMFLAGS += -D__HEAP_SIZE=0

#building all targets
all: appl finalize_all

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS_DEMO = $(addprefix $(OBJECT_DIRECTORY_DEMO)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS_DEMO = $(addprefix $(OBJECT_DIRECTORY_DEMO)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

appl: OUTPUT_FILENAME = $(DEMO_OUTPUT_NAME)
appl: $(BUILD_DIRECTORIES) $(C_OBJECTS_DEMO) $(ASM_OBJECTS_DEMO)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(C_OBJECTS_DEMO) $(ASM_OBJECTS_DEMO) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Explicit target for bootloader_util.c. 
# ARM ABI uses r7 as frame pointer for thumb mode, this to make it easier to 
# debug the code. Omitting the frame pointer globally on the code is possible, 
# but it will be harder to debug. So lets just do it on this file.
$(OBJECT_DIRECTORY_DEMO)/bootloader_util.o: $(SDK_ROOT)/components/libraries/bootloader_dfu/bootloader_util.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) -fomit-frame-pointer $(INC_PATHS) -c -o $@ $<

# Create objects from C SRC files
$(OBJECT_DIRECTORY_DEMO)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY_DEMO)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize_all: genbin genapphex echosize dfu_package

genbin:
	@echo Preparing: $(DEMO_OUTPUT_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(DEMO_OUTPUT_NAME).out $(OUTPUT_BINARY_DIRECTORY)/$(DEMO_OUTPUT_NAME).bin

## Create binary .hex file from the .out file
genapphex: 
	@echo Preparing: $(DEMO_OUTPUT_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(DEMO_OUTPUT_NAME).out $(OUTPUT_BINARY_DIRECTORY)/$(DEMO_OUTPUT_NAME).hex
	
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(DEMO_OUTPUT_NAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)
	
cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf52  --sectorerase
	nrfjprog --reset -f nrf52

## Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_2.0.0_softdevice.hex
	nrfjprog --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_2.0.0_softdevice.hex -f nrf52 --chiperase
	nrfjprog --reset -f nrf52

dfu_package:
	@echo Creating DFU packages
	$(NRF_UTIL) dfu genpkg --application bin/$(DEMO_OUTPUT_NAME).hex --dev-revision 0x0100 --dev-type 0x0001 bin/dfu_$(DEMO_OUTPUT_NAME)_$(fw_version_wo_dot).zip
    
create_demo_product:
	@echo Preparing: $(DEMO_OUTPUT_NAME).hex
	$(NO_ECHO)$(SREC_CAT) $(OUTPUT_BINARY_DIRECTORY)/$(DEMO_OUTPUT_NAME).hex -intel bootloader/_build/nrf52832_xxaa_s132.hex -intel tools/app_valid_setting_apply.hex -intel $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_2.0.0_softdevice.hex -intel -o $(OUTPUT_BINARY_DIRECTORY)/B200-3D_Demo.hex -intel
