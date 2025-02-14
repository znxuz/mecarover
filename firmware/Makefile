NAME := application

APP_DIR := $(NAME)
BUILD_DIR := build
DIR_GUARD = @mkdir -p "$(@D)"

ST_CORE_DIR := Core
ST_DRIVERS_DIR := Drivers
ST_MW_DIR := Middlewares/Third_Party

EIGEN_DIR := third_party/eigen
ULOG_DIR := third_party/ulog

DEBUG := -DDEBUG -g3
ULOG_ENABLED := -DULOG_ENABLED

OPT := -Os

BIN := $(BUILD_DIR)/$(NAME).bin
EXECUTABLE := $(BUILD_DIR)/$(NAME).elf
MAP_FILES := $(BUILD_DIR)/$(NAME).map
OBJDUMP_LIST := $(BUILD_DIR)/$(NAME).list
SIZE_OUTPUT := $(BUILD_DIR)/default.size.stdout

INCL_PATHS := \
			  -I$(CURDIR) \
			  -I$(ST_CORE_DIR)/Inc \
			  -I$(ST_DRIVERS_DIR)/STM32F7xx_HAL_Driver/Inc \
			  -I$(ST_DRIVERS_DIR)/CMSIS/Device/ST/STM32F7xx/Include \
			  -I$(ST_DRIVERS_DIR)/CMSIS/Include \
			  -I$(ST_MW_DIR)/FreeRTOS/Source/include \
			  -I$(ST_MW_DIR)/FreeRTOS/Source/CMSIS_RTOS_V2 \
			  -I$(ST_MW_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
			  -I$(EIGEN_DIR) \
			  -I$(ULOG_DIR)/src

FLAGS = \
		-mcpu=cortex-m7 \
		--specs=nano.specs \
		--specs=nosys.specs \
		-mfpu=fpv5-d16 \
		-mfloat-abi=hard \
		-mthumb \
		-DUSE_HAL_DRIVER \
		-DSTM32F767xx \
		$(DEBUG) \
		$(OPT) \
		-c \
		-ffunction-sections \
		-fdata-sections \
		-Wall \
		-fstack-usage \
		-MMD \
		-MP \
		-MF"$(@:%.o=%.d)" \
		-MT"$@" \
		$(INCL_PATHS) \
		$(ULOG_ENABLED)

CFLAGS = \
		 $(FLAGS) \
		 -std=gnu11

CPPFLAGS = \
		   $(FLAGS) \
		   -std=gnu++20 \
		   -fno-rtti \
		   -fno-use-cxa-atexit

LDFLAGS = \
		  $(DEBUG) \
		  -mcpu=cortex-m7 \
		  -mfpu=fpv5-d16 \
		  -mfloat-abi=hard \
		  --specs=nano.specs \
		  --specs=nosys.specs \
		  -mthumb \
		  -T$(LD_FILE) \
		  -Wl,-Map=$(MAP_FILES) \
		  -Wl,--gc-sections \
		  -static \
		  -u_printf_float \
		  -Wl,--start-group \
		  -lc \
		  -lm \
		  -lstdc++ \
		  -Wl,--end-group

STARTUP_FLAGS = \
				$(DEBUG) \
				-mcpu=cortex-m7 \
				--specs=nano.specs \
				--specs=nosys.specs \
				-mfpu=fpv5-d16 \
				-mfloat-abi=hard \
				-mthumb \
				-c \
				-x assembler-with-cpp \
				-MMD \
				-MP \
				-MF"$(@:%.o=%.d)" \
				-MT"$@"

SRCS_PATHS := \
			  $(APP_DIR) \
			  $(ST_CORE_DIR) \
			  $(ST_DRIVERS_DIR) \
			  $(ST_MW_DIR) \
			  $(ULOG_DIR)/src
C_SRCS_EXCLS := $(shell find application/micro_ros -type f -name '*.c')
C_SRCS := $(filter-out $(C_SRCS_EXCLS), \
		  $(shell find $(SRCS_PATHS) -type f -name "*.c"))
CPP_SRCS_EXCLS := $(shell find application/micro_ros -type f -name '*.cpp')
CPP_SRCS := $(filter-out $(CPP_SRCS_EXCLS), \
			$(shell find $(SRCS_PATHS) -type f -name "*.cpp"))
S_SRC := startup_stm32f767xx.s
LD_FILE := stm32f767zitx_flash.ld
OBJS := $(addprefix $(BUILD_DIR)/, $(C_SRCS:.c=.o)) \
		$(addprefix $(BUILD_DIR)/, $(CPP_SRCS:.cpp=.o)) \
		$(addprefix $(BUILD_DIR)/, $(S_SRC:.s=.o))

.PHONY: all clean flash reflash

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

$(BIN): $(EXECUTABLE)
	arm-none-eabi-objcopy -O binary $(EXECUTABLE) $(EXECUTABLE:.elf=.bin)

$(EXECUTABLE) $(MAP_FILES): $(OBJS) $(LD_FILE)
	arm-none-eabi-g++ $(OBJS) $(LDFLAGS) -o $(EXECUTABLE)
	@echo "Finished building target: $@"

$(SIZE_OUTPUT): $(EXECUTABLE)
	arm-none-eabi-size  $(EXECUTABLE)
	@echo "Finished building: $@"

$(OBJDUMP_LIST): $(EXECUTABLE)
	arm-none-eabi-objdump -h -S $(EXECUTABLE) > "$(OBJDUMP_LIST)"
	@echo "Finished building: $@"

clean:
	$(RM) $(shell find $(BUILD_DIR) -type f -name '*.su' -or -name '*.d' -or -name '*.o')
	$(RM) $(EXECUTABLE)
	$(RM) $(BIN)
	$(RM) $(MAP_FILES)
	$(RM) $(OBJDUMP_LIST)

clangdb: clean
	@mkdir -p build
	@bear --output $(BUILD_DIR)/compile_commands.json -- $(MAKE) -j

# misc

print_c_srcs:
	@echo $(C_SRCS)

print_cpp_srcs:
	@echo $(CPP_SRCS)

print_objs:
	@echo $(OBJS)

flash: all
	@while ! st-flash --reset write $(EXECUTABLE:.elf=.bin) 0x8000000; do \
		echo "st-flash failed, retrying..."; \
		sleep 1; \
		done

reflash:
	$(MAKE) clean
	$(MAKE) flash
