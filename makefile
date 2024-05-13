NAME := ros2

BUILD_DIR := build
DIR_GUARD = @mkdir -p "$(@D)"

ST_CORE := Core
ST_DRIVERS := Drivers
ST_MW := Middlewares/Third_Party

MICROROS_DIR := micro_ros_stm32cubemx_utils/microros_static_library/libmicroros
MICROROS_LIB := -L$(MICROROS_DIR) -lmicroros

LIBS := lib

INCL_PATHS := \
			  -I$(CURDIR) \
			  -I$(LIBS) \
			  -I$(ST_CORE)/Inc \
			  -I$(ST_DRIVERS)/STM32F7xx_HAL_Driver/Inc \
			  -I$(ST_DRIVERS)/CMSIS/Device/ST/STM32F7xx/Include \
			  -I$(ST_DRIVERS)/CMSIS/Include \
			  -I$(ST_MW)/FreeRTOS/Source/include \
			  -I$(ST_MW)/FreeRTOS/Source/CMSIS_RTOS_V2 \
			  -I$(ST_MW)/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
			  -I$(ST_MW)/LwIP/src/include \
			  -I$(ST_MW)/LwIP/system \
			  -I$(ST_MW)/LwIP/src/include/netif/ppp \
			  -I$(ST_MW)/LwIP/src/include/lwip \
			  -I$(ST_MW)/LwIP/src/include/lwip/apps \
			  -I$(ST_MW)/LwIP/src/include/lwip/priv \
			  -I$(ST_MW)/LwIP/src/include/lwip/prot \
			  -I$(ST_MW)/LwIP/src/include/netif \
			  -I$(ST_MW)/LwIP/src/include/compat/posix \
			  -I$(ST_MW)/LwIP/src/include/compat/posix/arpa \
			  -I$(ST_MW)/LwIP/src/include/compat/posix/net \
			  -I$(ST_MW)/LwIP/src/include/compat/posix/sys \
			  -I$(ST_MW)/LwIP/src/include/compat/stdc \
			  -I$(ST_MW)/LwIP/system/arch \
			  -I$(MICROROS_DIR)/microros_include \
			  -ILWIP/App \
			  -ILWIP/Target

TGT_FLAGS = \
			 -mcpu=cortex-m7 \
			 -g3 \
			 -DDEBUG \
			 -DUSE_HAL_DRIVER \
			 -DSTM32F767xx \
			 -c \
			 -O0 \
			 -ffunction-sections \
			 -fdata-sections \
			 -u_printf_float \
			 -Wall \
			 -fstack-usage \
			 -MMD \
			 -MP \
			 -MF"$(@:%.o=%.d)" \
			 -MT"$@" \
			 --specs=nano.specs \
			 -mfpu=fpv5-d16 \
			 -mfloat-abi=hard \
			 -mthumb \
			 $(INCL_PATHS)

	TGT_CPPFLAGS = $(TGT_FLAGS) \
				   -std=gnu++17 \
				   -fexceptions \
				   -fno-rtti \
				   -fno-use-cxa-atexit

TGT_CFLAGS = $(TGT_FLAGS) \
			 -std=gnu11

C_SRCS := $(filter-out \
		  ./micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c \
		  ./micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c \
		  ./micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c \
		  ./micro_ros_stm32cubemx_utils/sample_main.c \
		  ./$(ST_CORE)/Src/freertos.c \
		  ./$(ST_CORE)/Src/syscalls.c \
		  ./$(ST_CORE)/Src/sysmem.c \
		  , $(shell find . -type f -name "*.c"))
CC_SRCS := $(shell find . -type f -name "*.cc")
S_SRC := $(ST_CORE)/Startup/startup_stm32f767zitx.s

OBJS := $(addprefix $(BUILD_DIR)/, $(C_SRCS:.c=.o)) \
		$(addprefix $(BUILD_DIR)/, $(CC_SRCS:.cc=.o)) \
		$(addprefix $(BUILD_DIR)/, $(S_SRC:.s=.o))

BIN := $(BUILD_DIR)/$(NAME).bin
EXECUTABLES := $(BUILD_DIR)/$(NAME).elf
MAP_FILES := $(BUILD_DIR)/$(NAME).map
OBJDUMP_LIST := $(BUILD_DIR)/$(NAME).list
SIZE_OUTPUT := $(BUILD_DIR)/default.size.stdout

.PHONY: all clean

all: $(BIN) $(SIZE_OUTPUT) $(OBJDUMP_LIST)

$(BUILD_DIR)/%.o $(BUILD_DIR)/%.su: %.c
	$(DIR_GUARD)
	arm-none-eabi-gcc "$<" $(TGT_CFLAGS) -o "$@"

$(BUILD_DIR)/%.o $(BUILD_DIR)/%.su: %.cc
	$(DIR_GUARD)
	arm-none-eabi-g++ "$<" $(TGT_CPPFLAGS) -o "$@"

$(BUILD_DIR)/%.o: %.s
	$(DIR_GUARD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

flash: $(BIN)
	st-flash --reset write $(EXECUTABLES:.elf=.bin) 0x8000000

$(BIN): $(EXECUTABLES)
	arm-none-eabi-objcopy -O binary $(EXECUTABLES) $(EXECUTABLES:.elf=.bin)

$(EXECUTABLES) $(MAP_FILES): $(OBJS) STM32F767ZITX_FLASH.ld
	arm-none-eabi-g++ -o $(EXECUTABLES) $(OBJS) $(MICROROS_LIB) -mcpu=cortex-m7 -TSTM32F767ZITX_FLASH.ld --specs=nosys.specs -Wl,-Map=$(MAP_FILES) -Wl,--gc-sections -static $(MICROROS_LIB) -u_printf_float --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'

$(SIZE_OUTPUT): $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'

$(OBJDUMP_LIST): $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "$(OBJDUMP_LIST)"
	@echo 'Finished building: $@'

clean:
	$(RM) $(OBJS)

fclean:
	$(RM) $(shell find $(BUILD_DIR) -type f -name '*.su' -or -name '*.d' -or -name '*.o')
	$(RM) $(EXECUTABLES)
	$(RM) $(BIN)
	$(RM) $(MAP_FILES)
	$(RM) $(OBJDUMP_LIST)

# test
print_c_src:
	@echo $(C_SRCS)

print_cc_src:
	@echo $(CC_SRCS)

print_objs:
	@echo $(OBJS)

backup: $(BIN)
	@cp $(BIN) $(BUILD_DIR)/$(shell git log -1 --pretty='%h').bin
	@rm -f $(BUILD_DIR)/$(shell git log -2 --pretty='%h').bin

diff:
	@diff $(BUILD_DIR)/*.bin
