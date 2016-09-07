PROJECT_NAME := TwinRotors

export OUTPUT_FILENAME

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = sdk/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath sdk/components/libraries/button/app_button.c) \
$(abspath sdk/components/libraries/util/app_error.c) \
$(abspath sdk/components/libraries/util/app_error_weak.c) \
$(abspath sdk/components/libraries/timer/app_timer_freertos.c) \
$(abspath sdk/components/libraries/trace/app_trace.c) \
$(abspath sdk/components/libraries/util/app_util_platform.c) \
$(abspath sdk/components/libraries/fstorage/fstorage.c) \
$(abspath sdk/components/libraries/util/nrf_assert.c) \
$(abspath sdk/components/libraries/util/nrf_log.c) \
$(abspath sdk/components/libraries/sensorsim/sensorsim.c) \
$(abspath sdk/external/segger_rtt/RTT_Syscalls_GCC.c) \
$(abspath sdk/external/segger_rtt/SEGGER_RTT.c) \
$(abspath sdk/external/segger_rtt/SEGGER_RTT_printf.c) \
$(abspath sdk/external/freertos/source/croutine.c) \
$(abspath sdk/external/freertos/source/event_groups.c) \
$(abspath sdk/external/freertos/source/portable/MemMang/heap_1.c) \
$(abspath sdk/external/freertos/source/list.c) \
$(abspath sdk/external/freertos/portable/GCC/nrf52/port.c) \
$(abspath sdk/external/freertos/portable/CMSIS/nrf52/port_cmsis.c) \
$(abspath sdk/external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c) \
$(abspath sdk/external/freertos/source/queue.c) \
$(abspath sdk/external/freertos/source/tasks.c) \
$(abspath sdk/external/freertos/source/timers.c) \
$(abspath sdk/components/libraries/fifo/app_fifo.c) \
$(abspath sdk/components/drivers_nrf/delay/nrf_delay.c) \
$(abspath sdk/components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath sdk/components/drivers_nrf/pstorage/pstorage.c) \
$(abspath sdk/components/drivers_nrf/clock/nrf_drv_clock.c) \
$(abspath sdk/components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath sdk/examples/bsp/bsp.c) \
$(abspath sdk/examplesbsp/bsp_btn_ble.c) \
$(abspath ../common/main.c) \
$(abspath src/ble_main.c) \
$(abspath src/hal.c) \
$(abspath src/ble_rs.c) \
$(abspath sdk/components/ble/common/ble_advdata.c) \
$(abspath sdk/components/ble/ble_advertising/ble_advertising.c) \
$(abspath sdk/components/ble/common/ble_conn_params.c) \
$(abspath sdk/components/ble/common/ble_srv_common.c) \
$(abspath sdk/components/ble/device_manager/device_manager_peripheral.c) \
$(abspath sdk/components/toolchain/system_nrf52.c) \
$(abspath sdk/components/softdevice/common/softdevice_handler/softdevice_handler.c) \

# $(abspath sdk/components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath sdk/components/toolchain/gcc/gcc_startup_nrf52.s)

#includes common to all targets
INC_PATHS  = -I$(abspath config/ble_app_template_s132_pca10040)
INC_PATHS += -I$(abspath ../common)
INC_PATHS += -I$(abspath config)
INC_PATHS += -I$(abspath src)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/config)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/delay)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/pstorage)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/hal)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/common)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/clock)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath sdk/components/libraries/timer)
INC_PATHS += -I$(abspath sdk/components/libraries/fstorage/config)
INC_PATHS += -I$(abspath sdk/components/libraries/sensorsim)
INC_PATHS += -I$(abspath sdk/components/libraries/fifo)
INC_PATHS += -I$(abspath sdk/components/libraries/util)
INC_PATHS += -I$(abspath sdk/components/libraries/button)
INC_PATHS += -I$(abspath sdk/components/libraries/fstorage)
INC_PATHS += -I$(abspath sdk/components/libraries/experimental_section_vars)
INC_PATHS += -I$(abspath sdk/components/libraries/trace)
INC_PATHS += -I$(abspath sdk/components/softdevice/s132/headers/nrf52)
INC_PATHS += -I$(abspath sdk/components/softdevice/s132/headers)
INC_PATHS += -I$(abspath sdk/components/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath sdk/components/ble/ble_advertising)
INC_PATHS += -I$(abspath sdk/components/ble/device_manager)
INC_PATHS += -I$(abspath sdk/components/ble/common)
INC_PATHS += -I$(abspath sdk/components/device)
INC_PATHS += -I$(abspath sdk/external/freertos/portable/GCC/nrf52)
INC_PATHS += -I$(abspath sdk/components/drivers_nrf/delay)
INC_PATHS += -I$(abspath sdk/external/freertos/source/include)
INC_PATHS += -I$(abspath sdk/external/freertos/config)
INC_PATHS += -I$(abspath sdk/external/freertos/portable/CMSIS/nrf52)
INC_PATHS += -I$(abspath sdk/external/segger_rtt)
INC_PATHS += -I$(abspath sdk/examples/bsp)
INC_PATHS += -I$(abspath sdk/components/toolchain/CMSIS/Include)
INC_PATHS += -I$(abspath sdk/components/toolchain/gcc)
INC_PATHS += -I$(abspath sdk/components/toolchain)


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF52
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DBOARD_PCA10040
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
CFLAGS += -DNRF_LOG_USES_UART=0
CFLAGS += -DS132
CFLAGS += -DFREERTOS
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
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
ASMFLAGS += -DNRF_LOG_USES_UART=0
ASMFLAGS += -DS132
ASMFLAGS += -DFREERTOS
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63

#default target - first one defined
default: clean nrf52832_xxaa_s132

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf52832_xxaa_s132

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf52832_xxaa_s132
	@echo 	flash_softdevice

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf52832_xxaa_s132: OUTPUT_FILENAME := nrf52832_xxaa_s132
nrf52832_xxaa_s132: LINKER_SCRIPT=linker/ble_app_template_gcc_nrf52.ld

nrf52832_xxaa_s132: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
flash: nrf52832_xxaa_s132
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf52  --sectorerase
	nrfjprog --reset -f nrf52

## Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_2.0.0_softdevice.hex
	nrfjprog --program sdk/components/softdevice/s132/hex/s132_nrf52_2.0.0_softdevice.hex -f nrf52 --chiperase
	nrfjprog --reset -f nrf52