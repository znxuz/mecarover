NAME := mecarover

BUILD_DIR := build
DIR_GUARD = @mkdir -p "$(@D)"

ST_DIR_CORE := Core
ST_DIR_DRIVERS := Drivers
ST_DIR_MW := Middlewares/Third_Party
ST_DIR_LWIP := LWIP

MICRO_ROS_DIR := micro_ros_stm32cubemx_utils
MICRO_ROS_LIB_DIR := micro_ros_stm32cubemx_utils/microros_static_library/libmicroros
MICRO_ROS_LIB := -L$(MICRO_ROS_LIB_DIR) -lmicroros

EIGEN_DIR := eigen

BIN := $(BUILD_DIR)/$(NAME).bin
EXECUTABLE := $(BUILD_DIR)/$(NAME).elf
MAP_FILES := $(BUILD_DIR)/$(NAME).map
OBJDUMP_LIST := $(BUILD_DIR)/$(NAME).list
SIZE_OUTPUT := $(BUILD_DIR)/default.size.stdout

INCL_PATHS := \
			  -I$(CURDIR) \
			  -I$(EIGEN_DIR) \
			  -I$(ST_DIR_CORE)/Inc \
			  -I$(ST_DIR_DRIVERS)/STM32F7xx_HAL_Driver/Inc \
			  -I$(ST_DIR_DRIVERS)/CMSIS/Device/ST/STM32F7xx/Include \
			  -I$(ST_DIR_DRIVERS)/CMSIS/Include \
			  -I$(ST_DIR_MW)/FreeRTOS/Source/include \
			  -I$(ST_DIR_MW)/FreeRTOS/Source/CMSIS_RTOS_V2 \
			  -I$(ST_DIR_MW)/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
			  -I$(ST_DIR_MW)/LwIP/src/include \
			  -I$(ST_DIR_MW)/LwIP/system \
			  -I$(ST_DIR_MW)/LwIP/src/include/netif/ppp \
			  -I$(ST_DIR_MW)/LwIP/src/include/lwip \
			  -I$(ST_DIR_MW)/LwIP/src/include/lwip/apps \
			  -I$(ST_DIR_MW)/LwIP/src/include/lwip/priv \
			  -I$(ST_DIR_MW)/LwIP/src/include/lwip/prot \
			  -I$(ST_DIR_MW)/LwIP/src/include/netif \
			  -I$(ST_DIR_MW)/LwIP/src/include/compat/posix \
			  -I$(ST_DIR_MW)/LwIP/src/include/compat/posix/arpa \
			  -I$(ST_DIR_MW)/LwIP/src/include/compat/posix/net \
			  -I$(ST_DIR_MW)/LwIP/src/include/compat/posix/sys \
			  -I$(ST_DIR_MW)/LwIP/src/include/compat/stdc \
			  -I$(ST_DIR_MW)/LwIP/system/arch \
			  -I$(MICRO_ROS_LIB_DIR)/microros_include \
			  -I$(ST_DIR_LWIP)/App \
			  -I$(ST_DIR_LWIP)/Target

FLAGS = \
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

CFLAGS = \
		 $(FLAGS) \
		 -std=gnu11

CPPFLAGS = \
		   $(FLAGS) \
		   -std=gnu++20 \
		   -fexceptions \
		   -fno-rtti \
		   -fno-use-cxa-atexit

LD_FLAGS = \
		   $(MICRO_ROS_LIB) \
		   -mcpu=cortex-m7 \
		   -TSTM32F767ZITX_FLASH.ld \
		   --specs=nosys.specs \
		   -Wl,-Map=$(MAP_FILES) \
		   -Wl,--gc-sections \
		   -static \
		   -u_printf_float \
		   --specs=nano.specs \
		   -mfpu=fpv5-d16 \
		   -mfloat-abi=hard \
		   -mthumb \
		   -Wl,--start-group \
		   -lc \
		   -lm \
		   -lstdc++ \
		   -lsupc++ \
		   -Wl,--end-group

STARTUP_FLAGS = \
				-mcpu=cortex-m7 \
				-g3 \
				-DDEBUG \
				-c \
				-x \
				assembler-with-cpp \
				-MMD \
				-MP \
				-MF"$(@:%.o=%.d)" \
				-MT"$@" \
				--specs=nano.specs \
				-mfpu=fpv5-d16 \
				-mfloat-abi=hard \
				-mthumb

SRCS_PATHS := $(ST_DIR_CORE) $(ST_DIR_DRIVERS) $(ST_DIR_MW) $(ST_DIR_LWIP) $(MICRO_ROS_DIR) mecarover

# TODO replace the blacklist with a whitelist
C_SRCS := $(filter-out \
		  $(MICRO_ROS_DIR)/extra_sources/microros_transports/dma_transport.c \
		  $(MICRO_ROS_DIR)/extra_sources/microros_transports/it_transport.c \
		  $(MICRO_ROS_DIR)/extra_sources/microros_transports/udp_transport.c \
		  $(MICRO_ROS_DIR)/extra_sources/microros_transports/usb_cdc_transport.c \
		  $(MICRO_ROS_DIR)/sample_main.c \
		  $(MICRO_ROS_DIR)/sample_main_embeddedrtps.c \
		  $(MICRO_ROS_DIR)/sample_main_udp.c \
		  $(ST_DIR_CORE)/Src/freertos.c \
		  $(ST_DIR_CORE)/Src/syscalls.c \
		  $(ST_DIR_CORE)/Src/sysmem.c \
		  , $(shell find $(SRCS_PATHS) -type f -name "*.c"))
CPP_SRCS := $(shell find $(SRCS_PATHS) -type f -name "*.cpp")
S_SRC := $(ST_DIR_CORE)/Startup/startup_stm32f767zitx.s
OBJS := $(addprefix $(BUILD_DIR)/, $(C_SRCS:.c=.o)) \
		$(addprefix $(BUILD_DIR)/, $(CPP_SRCS:.cpp=.o)) \
		$(addprefix $(BUILD_DIR)/, $(S_SRC:.s=.o))

.PHONY: all clean fclean flash

all: $(BIN) $(SIZE_OUTPUT) $(OBJDUMP_LIST)

$(BUILD_DIR)/%.o $(BUILD_DIR)/%.su: %.c
	$(DIR_GUARD)
	arm-none-eabi-gcc "$<" $(CFLAGS) -o "$@"

$(BUILD_DIR)/%.o $(BUILD_DIR)/%.su: %.cpp
	$(DIR_GUARD)
	arm-none-eabi-g++ "$<" $(CPPFLAGS) -o "$@"

$(BUILD_DIR)/%.o: %.s
	$(DIR_GUARD)
	arm-none-eabi-gcc $(STARTUP_FLAGS) "$<" -o "$@"

flash: $(BIN)
	st-flash --reset write $(EXECUTABLE:.elf=.bin) 0x8000000

$(BIN): $(EXECUTABLE)
	arm-none-eabi-objcopy -O binary $(EXECUTABLE) $(EXECUTABLE:.elf=.bin)

$(EXECUTABLE) $(MAP_FILES): $(OBJS) STM32F767ZITX_FLASH.ld
	arm-none-eabi-g++ $(OBJS) $(LD_FLAGS) -o $(EXECUTABLE)
	@echo "Finished building target: $@"

$(SIZE_OUTPUT): $(EXECUTABLE)
	arm-none-eabi-size  $(EXECUTABLE)
	@echo "Finished building: $@"

$(OBJDUMP_LIST): $(EXECUTABLE)
	arm-none-eabi-objdump -h -S $(EXECUTABLE) > "$(OBJDUMP_LIST)"
	@echo "Finished building: $@"

clean:
	$(RM) $(OBJS)

fclean:
	$(RM) $(shell find $(BUILD_DIR) -type f -name '*.su' -or -name '*.d' -or -name '*.o')
	$(RM) $(EXECUTABLE)
	$(RM) $(BIN)
	$(RM) $(MAP_FILES)
	$(RM) $(OBJDUMP_LIST)

# required by micro_ros_stm32cubemx_utils
print_cflags:
	@echo $(CFLAGS)

clangd_db: clean
	@bear --output build/compile_commands.json -- make -j

# test
print_c_srcs:
	@echo $(C_SRCS)

print_cpp_srcs:
	@echo $(CPP_SRCS)

print_objs:
	@echo $(OBJS)

backup: $(BIN)
	@cp $(BIN) $(BUILD_DIR)/$(shell git log -1 --pretty='%h').bin
	@rm -f $(BUILD_DIR)/$(shell git log -2 --pretty='%h' | tail -n1).bin

diff: $(BIN)
	diff $(BUILD_DIR)/*.bin
