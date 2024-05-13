NAME := ros2
BUILD_DIR := build
DIR_GUARD = @mkdir -p "$(@D)"

TGT_CPPFLAGS = \
			   -mcpu=cortex-m7 \
			   -std=gnu++17 \
			   -g3 \
			   -DDEBUG \
			   -DUSE_HAL_DRIVER \
			   -DSTM32F767xx \
			   -c \
			   -I"components/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include" \
			   -IDrivers/STM32F7xx_HAL_Driver/Inc \
			   -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
			   -IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
			   -IDrivers/CMSIS/Include \
			   -IMiddlewares/Third_Party/FreeRTOS/Source/include \
			   -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
			   -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
			   -I"config" \
			   -I"components/controller" \
			   -I"components/HAL" \
			   -I"components/mrlogger" \
			   -I"components/ros_interface" \
			   -ICore/Inc \
			   -I"components/Eigen" \
			   -I"components/Eigen/Eigen" \
			   -I"Core/Inc" \
			   -ILWIP/App \
			   -ILWIP/Target \
			   -IMiddlewares/Third_Party/LwIP/src/include \
			   -IMiddlewares/Third_Party/LwIP/system \
			   -IDrivers/BSP/Components/lan8742 \
			   -IMiddlewares/Third_Party/LwIP/src/include/netif/ppp \
			   -IMiddlewares/Third_Party/LwIP/src/include/lwip \
			   -IMiddlewares/Third_Party/LwIP/src/include/lwip/apps \
			   -IMiddlewares/Third_Party/LwIP/src/include/lwip/priv \
			   -IMiddlewares/Third_Party/LwIP/src/include/lwip/prot \
			   -IMiddlewares/Third_Party/LwIP/src/include/netif \
			   -IMiddlewares/Third_Party/LwIP/src/include/compat/posix \
			   -IMiddlewares/Third_Party/LwIP/src/include/compat/posix/arpa \
			   -IMiddlewares/Third_Party/LwIP/src/include/compat/posix/net \
			   -IMiddlewares/Third_Party/LwIP/src/include/compat/posix/sys \
			   -IMiddlewares/Third_Party/LwIP/src/include/compat/stdc \
			   -IMiddlewares/Third_Party/LwIP/system/arch \
			   -I"components/micro_ros_stm32cubemx_utils" \
			   -I"components/LaserScanner" \
			   -O0 \
			   -ffunction-sections \
			   -fdata-sections \
			   -fno-exceptions \
			   -fno-rtti \
			   -fno-use-cxa-atexit \
			   -Wall \
			   -fexceptions \
			   -u_printf_float \
			   -fstack-usage \
			   -MMD \
			   -MP \
			   -MF"$(@:%.o=%.d)" \
			   -MT"$@" \
			   --specs=nano.specs \
			   -mfpu=fpv5-d16 \
			   -mfloat-abi=hard \
			   -mthumb

TGT_CFLAGS = \
			 -mcpu=cortex-m7 \
			 -std=gnu11 \
			 -g3 \
			 -DDEBUG \
			 -DUSE_HAL_DRIVER \
			 -DSTM32F767xx \
			 -c \
			 -I"components/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include" \
			 -IDrivers/STM32F7xx_HAL_Driver/Inc \
			 -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
			 -IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
			 -IDrivers/CMSIS/Include \
			 -IMiddlewares/Third_Party/FreeRTOS/Source/include \
			 -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
			 -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
			 -I"components/controller" \
			 -I"config" \
			 -I"components/mrlogger" \
			 -I"components/HAL" \
			 -ICore/Inc \
			 -I"components/Eigen/Eigen" \
			 -I"Core/Inc" \
			 -ILWIP/App \
			 -ILWIP/Target \
			 -IMiddlewares/Third_Party/LwIP/src/include \
			 -IMiddlewares/Third_Party/LwIP/system \
			 -IDrivers/BSP/Components/lan8742 \
			 -IMiddlewares/Third_Party/LwIP/src/include/netif/ppp \
			 -IMiddlewares/Third_Party/LwIP/src/include/lwip \
			 -IMiddlewares/Third_Party/LwIP/src/include/lwip/apps \
			 -IMiddlewares/Third_Party/LwIP/src/include/lwip/priv \
			 -IMiddlewares/Third_Party/LwIP/src/include/lwip/prot \
			 -IMiddlewares/Third_Party/LwIP/src/include/netif \
			 -IMiddlewares/Third_Party/LwIP/src/include/compat/posix \
			 -IMiddlewares/Third_Party/LwIP/src/include/compat/posix/arpa \
			 -IMiddlewares/Third_Party/LwIP/src/include/compat/posix/net \
			 -IMiddlewares/Third_Party/LwIP/src/include/compat/posix/sys \
			 -IMiddlewares/Third_Party/LwIP/src/include/compat/stdc \
			 -IMiddlewares/Third_Party/LwIP/system/arch \
			 -I"components/LaserScanner" \
			 -O0 \
			 -ffunction-sections \
			 -fdata-sections \
			 -Wall \
			 -fexceptions \
			 -u_printf_float \
			 -fstack-usage \
			 -MMD \
			 -MP \
			 -MF"$(@:%.o=%.d)" \
			 -MT"$@" \
			 --specs=nano.specs \
			 -mfpu=fpv5-d16 \
			 -mfloat-abi=hard \
			 -mthumb

C_SRCS := $(filter-out \
		  ./components/micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c \
		  ./components/micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c \
		  ./components/micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c \
		  ./components/micro_ros_stm32cubemx_utils/sample_main.c \
		  ./Core/Src/freertos.c \
		  ./Core/Src/syscalls.c \
		  ./Core/Src/sysmem.c \
		  , $(shell find . -type f -name "*.c"))
CC_SRCS := $(shell find . -type f -name "*.cc")
S_SRC := Core/Startup/startup_stm32f767zitx.s
LIBS := -lmicroros

OBJS := $(addprefix $(BUILD_DIR)/, $(C_SRCS:.c=.o)) \
		$(addprefix $(BUILD_DIR)/, $(CC_SRCS:.cc=.o)) \
		$(addprefix $(BUILD_DIR)/, $(S_SRC:.s=.o))

BIN := $(BUILD_DIR)/$(NAME).bin
EXECUTABLES := $(BUILD_DIR)/$(NAME).elf
MAP_FILES := $(BUILD_DIR)/$(NAME).map
SIZE_OUTPUT := $(BUILD_DIR)/default.size.stdout
OBJDUMP_LIST := $(BUILD_DIR)/$(NAME).list

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
	arm-none-eabi-g++ -o $(EXECUTABLES) $(OBJS) $(LIBS) -mcpu=cortex-m7 -TSTM32F767ZITX_FLASH.ld --specs=nosys.specs -Wl,-Map=$(MAP_FILES) -Wl,--gc-sections -static -Lcomponents/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros -u_printf_float --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'

$(SIZE_OUTPUT): $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'

$(OBJDUMP_LIST): $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "$(OBJDUMP_LIST)"
	@echo 'Finished building: $@'

clean:
	$(RM) $(OBJS)
	$(RM) $(EXECUTABLES)
	$(RM) $(BIN)
	$(RM) $(shell find $(BUILD_DIR) -type f -name '*.su' -or -name '*.d' -or -name '*.o')

fclean:
	$(RM) -r $(BUILD_DIR)

# test
print_c_src:
	@echo $(C_SRCS)

print_cc_src:
	@echo $(CC_SRCS)

print_objs:
	@echo $(OBJS)

diff:
	@diff $(BUILD_DIR)/*.bin
