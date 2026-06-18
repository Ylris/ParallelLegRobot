# Wheel Motor I2C Plan

This document records the current wheel-motor integration plan.

## Hardware Topology

The ESP32-C3 custom main board is the top-level controller. It uses one shared I2C bus through the power distribution board:

| Signal | ESP32-C3 pin |
| --- | --- |
| I2C SCL | GPIO3 |
| I2C SDA | GPIO4 |

All I2C devices share SDA/SCL/GND. They must have different 7-bit addresses.

| Device | Role | Default address |
| --- | --- | --- |
| MPU6050 | Body IMU | `0x68` |
| STM32F103C8T6 minimum board | Wheel PWM coprocessor | `0x12` |
| Wheel magnetic encoder | Wheel angle feedback | `0x36` |

The current firmware keeps all wheel outputs disabled at boot. Type `wheelarm` before sending PWM.

## STM32F103 Wiring

The F103 minimum board firmware in `src/f103_wheel_pwm.cpp` uses these pins:

| Signal | STM32F103 pin | Connect to |
| --- | --- | --- |
| I2C1 SCL | `PB6` | ESP32-C3 `GPIO3` / shared I2C SCL |
| I2C1 SDA | `PB7` | ESP32-C3 `GPIO4` / shared I2C SDA |
| GND | `GND` | ESP32-C3 GND, encoder GND, SimpleFOC GND |
| 3.3 V | `3V3` | 3.3 V logic supply |
| Left wheel PWM | `PA0` | Left SimpleFOC PWM/speed input |
| Right wheel PWM | `PA1` | Right SimpleFOC PWM/speed input |
| Left wheel DIR | `PA2` | Left SimpleFOC direction input, if used |
| Right wheel DIR | `PA3` | Right SimpleFOC direction input, if used |
| Status LED | `PC13` | On-board LED on many BluePill boards |

ST-Link programming wires:

| ST-Link | STM32F103 minimum board |
| --- | --- |
| `GND` | `GND` |
| `3.3V` / `VTref` | `3V3` |
| `SWDIO` / `DIO` | `PA13` / `SWDIO` |
| `SWCLK` / `CLK` | `PA14` / `SWCLK` |

Do not connect 5 V to ESP32-C3 GPIO, F103 GPIO, or I2C pull-ups. The shared I2C bus should be pulled up to 3.3 V.

## STM32F103 PWM Coprocessor Protocol

ESP32-C3 is I2C master. STM32F103 is I2C slave at address `0x12`.

Command `0x10` sets signed wheel PWM:

| Byte | Meaning |
| --- | --- |
| 0 | command register `0x10` |
| 1 | left PWM low byte |
| 2 | left PWM high byte |
| 3 | right PWM low byte |
| 4 | right PWM high byte |

PWM values are signed little-endian `int16_t`, clamped by ESP32 to `-1000..1000`.

Example payload for left `+200`, right `-200`:

```text
0x10 0xC8 0x00 0x38 0xFF
```

The STM32F103 firmware should treat missing I2C writes as a fault and stop PWM after a short timeout, for example 100 ms.

## Wheel Encoder Protocol

The current ESP32 firmware assumes an AS5600-compatible magnetic encoder:

| Address | Register | Meaning |
| --- | --- | --- |
| `0x36` | `0x0C`, `0x0D` | 12-bit raw angle |

The ESP32 converts the raw value to radians:

```text
angle_rad = raw * 2*pi / 4096
```

If the real encoder is not AS5600-compatible, only `readWheelEncoder()` in `src/main.cpp` needs to change.

## ESP32 Serial Commands

| Command | Meaning |
| --- | --- |
| `i2cscan` | Scan shared I2C bus and print detected addresses |
| `wheel` | Print wheel F103/encoder status |
| `wheelarm` | Allow wheel PWM commands, current PWM remains zero |
| `wheeldisarm` | Force wheel PWM to zero and disable wheel commands |
| `wheelpwm <left> <right>` | Send signed PWM to STM32F103, range `-1000..1000` |
| `wheelstream on` | Stream `wheel:angle,pwm_l,pwm_r,status...` lines |
| `wheelstream off` | Stop wheel stream |

Global `stop` and `disarm` also force wheel PWM to zero.

## Bring-up Order

1. Power ESP32-C3 logic only.
2. Run `i2cscan`; expect at least the F103 address and encoder address if connected.
3. Run `wheel`; confirm `f103=online` and `encoder=online`.
4. Power the SimpleFOC wheel power stage with current limit.
5. Run `wheelarm`.
6. Start with very small PWM, for example `wheelpwm 50 50`.
7. Use `wheelstream on` to check encoder angle changes.
8. Run `wheelpwm 0 0`, then `wheeldisarm` after the test.
