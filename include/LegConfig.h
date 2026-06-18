#ifndef LEG_CONFIG_H
#define LEG_CONFIG_H

#include <Arduino.h>

// ==========================================
// 机器人几何参数 (单位: mm)
// ==========================================
const float LEG_D = 20.0f; 
const float LEG_L1 = 45.0f;
const float LEG_L2 = 90.0f;

// ==========================================
// 方案一：GPIO 矩阵总线电机引脚与波特率配置
// ==========================================
const int MOTOR_COUNT = 6;
// 避开了 ESP32-C3 的启动引导引脚 (GPIO 2, GPIO 8, GPIO 9)
// 将原本的 GPIO 2 移至 GPIO 6
const int MOTOR_PINS[MOTOR_COUNT] = {0, 1, 6, 3, 4, 5};
const uint32_t MOTOR_BAUD = 115200;

// 电机索引定义
const int MOTOR_L1 = 0; // 左腿电机 1 (对应 GPIO 0)
const int MOTOR_L2 = 1; // 左腿电机 2 (对应 GPIO 1)
const int MOTOR_L3 = 2; // 左腿辅助电机 (对应 GPIO 6)
const int MOTOR_R1 = 3; // 右腿电机 1 (对应 GPIO 3)
const int MOTOR_R2 = 4; // 右腿电机 2 (对应 GPIO 4)
const int MOTOR_R3 = 5; // 右腿辅助电机 (对应 GPIO 5)

// ==========================================
// 板载 LED 呼吸灯配置
// ==========================================
// 避开了启动引脚 GPIO 8，移至安全引脚 GPIO 10
const int LED_PIN = 10;

// ==========================================
// 运动学限制 (工作空间限幅，单位: mm)
// ==========================================
const float LIMIT_X_MIN = -50.0f;
const float LIMIT_X_MAX = 50.0f;
const float LIMIT_Y_MIN = -130.0f;
const float LIMIT_Y_MAX = -60.0f;

// ==========================================
// Shared I2C bus on the custom ESP32-C3 main board.
// Devices on this bus:
//   - MPU6050 IMU, default 0x68
//   - STM32F103C8T6 wheel PWM coprocessor, default 0x12
//   - Wheel magnetic encoder, default AS5600 address 0x36
// ==========================================
const int I2C_SCL_PIN = 3;
const int I2C_SDA_PIN = 4;
const uint32_t I2C_BUS_HZ = 400000;

const int IMU_SDA_PIN = I2C_SDA_PIN;
const int IMU_SCL_PIN = I2C_SCL_PIN;
const uint8_t MPU6050_I2C_ADDR = 0x68;
const uint8_t WHEEL_PWM_COPROCESSOR_I2C_ADDR = 0x12;
const uint8_t LEFT_WHEEL_ENCODER_I2C_ADDR = 0x36;
const uint8_t RIGHT_WHEEL_ENCODER_I2C_ADDR = 0x38;

#endif // LEG_CONFIG_H

