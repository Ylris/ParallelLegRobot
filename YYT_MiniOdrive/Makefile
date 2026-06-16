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

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS := -DUSE_HAL_DRIVER -DSTM32G431xx -DYYT_SAFE_START=$(SAFE_START) -DDRIVE_ID=$(DRIVE_ID)

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

.PHONY: all clean flash flash-id erase probe ids

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

probe:
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; init; shutdown"

erase:
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; init; reset halt; stm32g4x mass_erase 0; shutdown"

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
