TARGET := turing_CBU6_open_safe_hold
BUILD_DIR := build

TOOLCHAIN ?= $(HOME)/.local/arm-gnu-toolchain-15.2.rel1
PREFIX := $(TOOLCHAIN)/bin/arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc -x assembler-with-cpp
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS := -DUSE_HAL_DRIVER -DSTM32G431xx -DOPEN_SOURCE_SAFE_HOLD=1

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

STARTUP_SOURCE := ../DriveFirmware/startup_stm32g431xx_gcc.s
STARTUP_OBJECT := $(BUILD_DIR)/startup_stm32g431xx_gcc.o

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o)) $(STARTUP_OBJECT)

CFLAGS := $(MCU) $(DEFS) $(INCLUDES) -Og -g3 -ffunction-sections -fdata-sections
ASFLAGS := $(MCU) $(DEFS) $(INCLUDES) -g3
LDSCRIPT := ../DriveFirmware/STM32G431CBUx_FLASH.ld
LDFLAGS := $(MCU) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs -lc -lm

.PHONY: all clean flash

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SZ) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MMD -MP $< -o $@

$(STARTUP_OBJECT): $(STARTUP_SOURCE)
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

flash: all
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

clean:
	rm -rf $(BUILD_DIR)
