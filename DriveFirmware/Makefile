TARGET := turing_CBU6
BUILD_DIR := build

TOOLCHAIN ?= $(HOME)/.local/arm-gnu-toolchain-15.2.rel1
PREFIX := $(TOOLCHAIN)/bin/arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc -x assembler-with-cpp
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size

SAFE_START ?= 1
DRIVE_ID ?= 1
CAN_FEEDBACK_SINGLE_TURN ?= 0
YYT_DISABLE_OUTPUT ?= 0
YYT_UART_DEBUG ?= 0
MOTOR_ZERO_ELECTRIC_ANGLE ?= 3.585681f
DRIVE_AUTO_ZERO_HOLD ?= 0
DRIVE_AUTO_ZERO_OUTPUT_SIGN ?= 1
DRIVE_AUTO_ZERO_HOLD_KP ?= 12.0f
DRIVE_AUTO_ZERO_HOLD_KI ?= 3.0f
DRIVE_AUTO_ZERO_HOLD_KD ?= 0.0f
DRIVE_AUTO_ZERO_HOLD_LIMIT ?= 12.0f
DRIVE_AUTO_ZERO_APPROACH_RAD ?= 0.12f
DRIVE_AUTO_ZERO_APPROACH_VELOCITY ?= 0.35f
DRIVE_AUTO_ZERO_APPROACH_VOLTAGE ?= 6.0f
DRIVE_AUTO_ZERO_DEADBAND_RAD ?= 0.0f
DRIVE_AUTO_ZERO_HOLD_DURATION_US ?= 0U
DRIVE_AUTO_ZERO_START_DELAY_US ?= 0U
MOTOR_AUTO_ELECTRIC_ZERO ?= 0
MOTOR_ALIGN_VOLTAGE ?= 3.0f
MOTOR_AUTO_ELECTRIC_ZERO_OFFSET ?= 0.0f
MOTOR_SENSOR_DIRECTION ?= 1.0f
MOTOR_POLE_PAIRS ?= 11.0f
MOTOR_SKIP_BOOT_SWEEP ?= 0
MOTOR_ALIGN_ONLY ?= 0
FOC_MODULATION_LIMIT ?= 0.577f
CAN_SPIN_TEST_VOLTAGE ?= 6.0f
CAN_SPIN_TEST_VELOCITY ?= 0.12f
CAN_PHASE_SCAN_ENABLE ?= 0
CAN_MAX_COMMAND_VOLTAGE ?= 12.0f

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS := -DUSE_HAL_DRIVER -DSTM32G431xx -DYYT_SAFE_START=$(SAFE_START) -DDRIVE_ID=$(DRIVE_ID) -DCAN_FEEDBACK_SINGLE_TURN=$(CAN_FEEDBACK_SINGLE_TURN) -DYYT_DISABLE_OUTPUT=$(YYT_DISABLE_OUTPUT) -DYYT_UART_DEBUG=$(YYT_UART_DEBUG) -DMOTOR_ZERO_ELECTRIC_ANGLE=$(MOTOR_ZERO_ELECTRIC_ANGLE) -DDRIVE_AUTO_ZERO_HOLD=$(DRIVE_AUTO_ZERO_HOLD) -DDRIVE_AUTO_ZERO_OUTPUT_SIGN=$(DRIVE_AUTO_ZERO_OUTPUT_SIGN) -DDRIVE_AUTO_ZERO_HOLD_KP=$(DRIVE_AUTO_ZERO_HOLD_KP) -DDRIVE_AUTO_ZERO_HOLD_KI=$(DRIVE_AUTO_ZERO_HOLD_KI) -DDRIVE_AUTO_ZERO_HOLD_KD=$(DRIVE_AUTO_ZERO_HOLD_KD) -DDRIVE_AUTO_ZERO_HOLD_LIMIT=$(DRIVE_AUTO_ZERO_HOLD_LIMIT) -DDRIVE_AUTO_ZERO_APPROACH_RAD=$(DRIVE_AUTO_ZERO_APPROACH_RAD) -DDRIVE_AUTO_ZERO_APPROACH_VELOCITY=$(DRIVE_AUTO_ZERO_APPROACH_VELOCITY) -DDRIVE_AUTO_ZERO_APPROACH_VOLTAGE=$(DRIVE_AUTO_ZERO_APPROACH_VOLTAGE) -DDRIVE_AUTO_ZERO_DEADBAND_RAD=$(DRIVE_AUTO_ZERO_DEADBAND_RAD) -DDRIVE_AUTO_ZERO_HOLD_DURATION_US=$(DRIVE_AUTO_ZERO_HOLD_DURATION_US) -DDRIVE_AUTO_ZERO_START_DELAY_US=$(DRIVE_AUTO_ZERO_START_DELAY_US) -DMOTOR_AUTO_ELECTRIC_ZERO=$(MOTOR_AUTO_ELECTRIC_ZERO) -DMOTOR_ALIGN_VOLTAGE=$(MOTOR_ALIGN_VOLTAGE) -DMOTOR_AUTO_ELECTRIC_ZERO_OFFSET=$(MOTOR_AUTO_ELECTRIC_ZERO_OFFSET) -DMOTOR_SENSOR_DIRECTION=$(MOTOR_SENSOR_DIRECTION) -DMOTOR_POLE_PAIRS=$(MOTOR_POLE_PAIRS) -DMOTOR_SKIP_BOOT_SWEEP=$(MOTOR_SKIP_BOOT_SWEEP) -DMOTOR_ALIGN_ONLY=$(MOTOR_ALIGN_ONLY) -DFOC_MODULATION_LIMIT=$(FOC_MODULATION_LIMIT) -DCAN_SPIN_TEST_VOLTAGE=$(CAN_SPIN_TEST_VOLTAGE) -DCAN_SPIN_TEST_VELOCITY=$(CAN_SPIN_TEST_VELOCITY) -DCAN_PHASE_SCAN_ENABLE=$(CAN_PHASE_SCAN_ENABLE) -DCAN_MAX_COMMAND_VOLTAGE=$(CAN_MAX_COMMAND_VOLTAGE)

INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include \
  -IFOC \
  -IHardware

C_SOURCES := \
  Core/Src/main.c \
  Core/Src/drive_uart_debug.c \
  Core/Src/gpio.c \
  Core/Src/adc.c \
  Core/Src/dma.c \
  Core/Src/fdcan.c \
  Core/Src/can_bridge.c \
  Core/Src/spi.c \
  Core/Src/tim.c \
  Core/Src/usart.c \
  Core/Src/stm32g4xx_it.c \
  Core/Src/stm32g4xx_hal_msp.c \
  Core/Src/system_stm32g4xx.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
  FOC/Data.c \
  FOC/FOC.c \
  FOC/Lowpass.c \
  FOC/Motor.c \
  FOC/Pid.c \
  FOC/axdr_pid.c \
  FOC/axdr_position_pid.c \
  FOC/axdr_zengliang_pid.c \
  Hardware/mt6816.c

ASM_SOURCES := startup_stm32g431xx_gcc.s

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

CFLAGS := $(MCU) $(DEFS) $(INCLUDES) -Og -g3 -Wall -ffunction-sections -fdata-sections
ASFLAGS := $(MCU) $(DEFS) $(INCLUDES) -g3
LDSCRIPT := STM32G431CBUx_FLASH.ld
LDFLAGS := $(MCU) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs -lc -lm

.PHONY: all clean flash flash-id erase probe ids ids-single-turn id6-zero-hold-current flash-id6-zero-hold-current id5-zero-hold-current flash-id5-zero-hold-current id5-uart-debug flash-id5-uart-debug

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SZ) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MMD -MP $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

flash: all
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

flash-id:
	$(MAKE) DRIVE_ID=$(DRIVE_ID) flash

id6-zero-hold-current:
	$(MAKE) clean
	$(MAKE) DRIVE_ID=6 DRIVE_AUTO_ZERO_HOLD=1 DRIVE_AUTO_ZERO_OUTPUT_SIGN=1 DRIVE_AUTO_ZERO_HOLD_KP=12.0f DRIVE_AUTO_ZERO_HOLD_KI=3.2f DRIVE_AUTO_ZERO_HOLD_KD=0.0f DRIVE_AUTO_ZERO_HOLD_LIMIT=12.0f DRIVE_AUTO_ZERO_APPROACH_VOLTAGE=12.0f DRIVE_AUTO_ZERO_DEADBAND_RAD=0.02f FOC_MODULATION_LIMIT=1.0f all

flash-id6-zero-hold-current: id6-zero-hold-current
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

id5-zero-hold-current:
	$(MAKE) clean
	$(MAKE) DRIVE_ID=5 DRIVE_AUTO_ZERO_HOLD=1 DRIVE_AUTO_ZERO_OUTPUT_SIGN=-1 DRIVE_AUTO_ZERO_HOLD_KP=8.0f DRIVE_AUTO_ZERO_HOLD_KI=2.0f DRIVE_AUTO_ZERO_HOLD_KD=0.25f DRIVE_AUTO_ZERO_HOLD_LIMIT=12.0f DRIVE_AUTO_ZERO_APPROACH_VOLTAGE=12.0f DRIVE_AUTO_ZERO_DEADBAND_RAD=0.02f DRIVE_AUTO_ZERO_START_DELAY_US=5000000U FOC_MODULATION_LIMIT=1.0f all

flash-id5-zero-hold-current: id5-zero-hold-current
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

id5-uart-debug:
	$(MAKE) clean
	$(MAKE) DRIVE_ID=5 DRIVE_AUTO_ZERO_HOLD=0 YYT_UART_DEBUG=1 MOTOR_SKIP_BOOT_SWEEP=1 FOC_MODULATION_LIMIT=1.0f all

flash-id5-uart-debug: id5-uart-debug
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

ids:
	@rm -rf firmware_ids
	@mkdir -p firmware_ids
	@for id in 1 2 3 5 6 7; do \
		echo "Building DRIVE_ID=$$id"; \
		$(MAKE) --no-print-directory clean >/dev/null; \
		$(MAKE) --no-print-directory DRIVE_ID=$$id >/dev/null; \
		cp $(BUILD_DIR)/$(TARGET).elf firmware_ids/$(TARGET)_id$$id.elf; \
		cp $(BUILD_DIR)/$(TARGET).hex firmware_ids/$(TARGET)_id$$id.hex; \
		cp $(BUILD_DIR)/$(TARGET).bin firmware_ids/$(TARGET)_id$$id.bin; \
	done

ids-single-turn:
	@rm -rf firmware_ids_single_turn
	@mkdir -p firmware_ids_single_turn
	@for id in 1 2 5 6; do \
		echo "Building DRIVE_ID=$$id CAN_FEEDBACK_SINGLE_TURN=1"; \
		$(MAKE) --no-print-directory clean >/dev/null; \
		$(MAKE) --no-print-directory DRIVE_ID=$$id CAN_FEEDBACK_SINGLE_TURN=1 >/dev/null; \
		cp $(BUILD_DIR)/$(TARGET).elf firmware_ids_single_turn/$(TARGET)_id$$id.elf; \
		cp $(BUILD_DIR)/$(TARGET).hex firmware_ids_single_turn/$(TARGET)_id$$id.hex; \
		cp $(BUILD_DIR)/$(TARGET).bin firmware_ids_single_turn/$(TARGET)_id$$id.bin; \
	done

probe:
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; init; shutdown"

erase:
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; init; reset halt; stm32g4x mass_erase 0; shutdown"

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
