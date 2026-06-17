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
// IMU MPU6050 I2C 引脚配置 (避开电机引脚，使用标准引脚并连接上拉电阻)
// ==========================================
const int IMU_SDA_PIN = 5;
const int IMU_SCL_PIN = 4;

#endif // LEG_CONFIG_H


