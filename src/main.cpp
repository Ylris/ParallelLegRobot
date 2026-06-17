#include <Arduino.h>
#include "driver/twai.h" // 引入 ESP32-C3 TWAI (CAN) 驱动
#include "LegConfig.h"
#include "LegKinematics.h"
#include "ImuManager.h"

// 运动学求解器与 IMU 管理器
LegKinematics kinematics;
ImuManager imu;

// 运行模式定义
enum MotionMode {
    MODE_STANDBY,     // 静态悬停在 (0, -100)
    MODE_BALANCE,     // IMU 自适应反馈平衡模式
    MODE_INTERACTIVE  // 串口控制模式
};

MotionMode currentMode = MODE_STANDBY;

// 目标点
float targetX = 0.0f;
float targetY = -100.0f;

// 打印当前状态的时间控制
unsigned long lastPrintTime = 0;

// PID 控制器结构体，用于在主控端对电机进行位置闭环控制
struct PIDController {
    float kp;
    float ki;
    float kd;
    float error_sum;
    float last_error;
    float max_output;

    void init(float p, float i, float d, float max_out) {
        kp = p;
        ki = i;
        kd = d;
        error_sum = 0.0f;
        last_error = 0.0f;
        max_output = max_out;
    }

    float update(float error, float dt, float current_velocity_rad_s, bool use_velocity_feedback) {
        error_sum += error * dt;
        
        // 积分限幅 (防止积分饱和，限制积分项输出不超过最大输出的50%)
        float max_i = max_output * 0.5f;
        if (ki > 0.0f) {
            if (error_sum * ki > max_i) error_sum = max_i / ki;
            if (error_sum * ki < -max_i) error_sum = -max_i / ki;
        }

        float p_term = kp * error;
        float i_term = ki * error_sum;
        
        float d_term;
        if (use_velocity_feedback) {
            // 使用电机反馈的真实速度，避免微分噪声
            d_term = -kd * current_velocity_rad_s;
        } else {
            d_term = kd * (error - last_error) / dt;
            last_error = error;
        }

        float output = p_term + i_term + d_term;
        if (output > max_output) output = max_output;
        if (output < -max_output) output = -max_output;
        return output;
    }
};

// 全局变量：电机 PID 控制器、目标角度、当前角度、当前速度、在线状态
PIDController motorPids[MOTOR_COUNT];
float targetAngles[MOTOR_COUNT] = {0.0f};
float currentAngles[MOTOR_COUNT] = {0.0f};
float currentVelocities[MOTOR_COUNT] = {0.0f};
bool motorFeedbackReceived[MOTOR_COUNT] = {false};
unsigned long lastFeedbackTime[MOTOR_COUNT] = {0};

// 辅助映射函数：DriveID 到 motorIndex
int getMotorIndex(int driveId) {
    if (driveId == 1) return 0; // MOTOR_L1 (左腿电机 1)
    if (driveId == 2) return 1; // MOTOR_L2 (左腿电机 2)
    if (driveId == 3) return 2; // MOTOR_L3 (左腿辅助电机)
    if (driveId == 5) return 3; // MOTOR_R1 (右腿电机 1)
    if (driveId == 6) return 4; // MOTOR_R2 (右腿电机 2)
    if (driveId == 7) return 5; // MOTOR_R3 (右腿辅助电机)
    return -1;
}

// 函数声明
void processSerialInput();
void executeTrajectory();
bool moveToPosition(float x, float y);
void updateBreathingLed();
void setMotorPosition(int motorIndex, float rad);
void initTwai();
void receiveCanMessages();
void sendCanCommands();

void setup() {
    // 1. 初始化调试串口
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n==============================================");
    Serial.println("ESP32-C3 Wheel-Leg CAN Controller Init...");
    Serial.println("==============================================");

    // 2. 初始化 IMU
    imu.begin();

    // 3. 初始化 TWAI (CAN) 驱动
    initTwai();

    // 4. 初始化所有电机的位置闭环 PID 控制器
    // 输入误差以 rad 为单位，输出为 millivolts (mV)，最大幅值为 3000mV (3.0V)
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorPids[i].init(6000.0f, 50.0f, 150.0f, 3000.0f);
    }
    Serial.println("All motor PID controllers initialized.");

    // 5. 初始化板载 LED
    pinMode(LED_PIN, OUTPUT);

    // 初始位置
    moveToPosition(0, -100);
    Serial.println("Robot initialized to standby position (0, -100)");

    // 打印菜单
    Serial.println("\n可用的控制指令:");
    Serial.println("  'standby'     - 保持在中位 (0, -100)");
    Serial.println("  'balance'     - 开启 IMU 自平衡反馈");
    Serial.println("  'X Y'         - 手动坐标控制 (例如: '10 -90' 或 '-5 -110')");
    Serial.println("==============================================");
}

void loop() {
    // 1. 接收所有来自 CAN 总线的电机反馈帧
    receiveCanMessages();

    // 2. 更新 IMU 数据
    imu.update();

    // 3. 处理串口输入
    processSerialInput();

    // 4. 根据模式执行轨迹，计算足端逆运动学并更新 targetAngles
    executeTrajectory();

    // 5. 运行 PID 控制器计算输出并通过 CAN 发送电压指令
    sendCanCommands();

    // 6. 发送 IMU 数据到 PioPulse (FireWater 协议)
    if (imu.isConnected()) {
        Serial.printf("imu:%.2f,%.2f,%.2f,0.0,0.0,0.0\n", 
                      imu.getPitch(), imu.getRoll(), imu.getYaw());
    }

    // 7. 更新板载 LED 呼吸灯
    updateBreathingLed();

    // 8. 延时控制更新频率 (约 50Hz, 20ms)
    delay(20);
}

// 初始化 TWAI (CAN) 驱动器，引脚为 TX=GPIO6, RX=GPIO7，波特率为 1 Mbps
void initTwai() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_6, GPIO_NUM_7, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            Serial.println("TWAI (CAN) driver started successfully at 1 Mbps.");
        } else {
            Serial.println("Failed to start TWAI driver.");
        }
    } else {
        Serial.println("Failed to install TWAI driver.");
    }
}

// 接收 CAN 总线的电机角度/速度反馈
void receiveCanMessages() {
    twai_message_t message;
    // 循环读取接收缓冲区中所有的消息
    while (twai_receive(&message, 0) == ESP_OK) {
        if (!(message.rtr) && message.identifier >= 0x101 && message.identifier <= 0x108) {
            int driveId = message.identifier - 0x100;
            int idx = getMotorIndex(driveId);
            if (idx != -1 && message.data_length_code == 8) {
                // 解析反馈帧：
                // bytes 0..3: int32_t angle_mrad (小端序)
                // bytes 4..5: int16_t speed_rpm_x10 (小端序)
                int32_t angle_mrad = (int32_t)(message.data[0] | 
                                              (message.data[1] << 8) | 
                                              (message.data[2] << 16) | 
                                              (message.data[3] << 24));
                int16_t speed_rpm_x10 = (int16_t)(message.data[4] | 
                                                 (message.data[5] << 8));
                
                currentAngles[idx] = (float)angle_mrad / 1000.0f; // 转换为弧度
                
                // 将 speed_rpm_x10 转换为 rad/s
                float speed_rpm = (float)speed_rpm_x10 / 10.0f;
                currentVelocities[idx] = speed_rpm * 0.104719755f;
                
                motorFeedbackReceived[idx] = true;
                lastFeedbackTime[idx] = millis();
            }
        }
    }
}

// 计算 PID 输出，打包并发送 CAN 控制帧 (0x100 和 0x200)
void sendCanCommands() {
    static unsigned long lastSendTime = 0;
    unsigned long now = millis();
    float dt = (float)(now - lastSendTime) * 0.001f;
    if (dt <= 0.0f) dt = 0.02f; // 避免除零
    lastSendTime = now;

    int16_t left_mv[4] = {0, 0, 0, 0};
    int16_t right_mv[4] = {0, 0, 0, 0};

    // 为每个电机运行位置 PID 控制
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // 安全检查：如果从未收到反馈或者超时（500ms无响应），则输出 0 mV 保证安全
        if (!motorFeedbackReceived[i] || (now - lastFeedbackTime[i] > 500)) {
            if (i < 3) {
                left_mv[i] = 0;
            } else {
                right_mv[i - 3] = 0;
            }
            continue;
        }

        float error = targetAngles[i] - currentAngles[i];
        float output_mv = motorPids[i].update(error, dt, currentVelocities[i], true);

        if (i < 3) {
            left_mv[i] = (int16_t)output_mv;
        } else {
            right_mv[i - 3] = (int16_t)output_mv;
        }
    }

    // 组装并发送左腿控制帧 (ID: 0x100)
    twai_message_t left_msg;
    left_msg.identifier = 0x100;
    left_msg.extd = 0;
    left_msg.rtr = 0;
    left_msg.data_length_code = 8;
    for (int i = 0; i < 4; i++) {
        left_msg.data[i * 2] = (uint8_t)(left_mv[i] & 0xFF);
        left_msg.data[i * 2 + 1] = (uint8_t)((left_mv[i] >> 8) & 0xFF);
    }
    twai_transmit(&left_msg, pdMS_TO_TICKS(5));

    // 组装并发送右腿控制帧 (ID: 0x200)
    twai_message_t right_msg;
    right_msg.identifier = 0x200;
    right_msg.extd = 0;
    right_msg.rtr = 0;
    right_msg.data_length_code = 8;
    for (int i = 0; i < 4; i++) {
        right_msg.data[i * 2] = (uint8_t)(right_mv[i] & 0xFF);
        right_msg.data[i * 2 + 1] = (uint8_t)((right_mv[i] >> 8) & 0xFF);
    }
    twai_transmit(&right_msg, pdMS_TO_TICKS(5));
}

// 设定目标电机角度 (只是更新 targetAngles 数组，发送由 sendCanCommands 统一周期性进行)
void setMotorPosition(int motorIndex, float rad) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;
    targetAngles[motorIndex] = rad;
}

// 驱动双腿足端移动到目标坐标 (x, y) 
bool moveToPosition(float x, float y) {
    // 限幅检查，防止超出工作空间
    if (x < LIMIT_X_MIN) x = LIMIT_X_MIN;
    if (x > LIMIT_X_MAX) x = LIMIT_X_MAX;
    if (y < LIMIT_Y_MIN) y = LIMIT_Y_MIN;
    if (y > LIMIT_Y_MAX) y = LIMIT_Y_MAX;

    float left_t1, left_t2;
    // 逆运动学求解左腿
    if (kinematics.inverseKinematics(x, y, left_t1, left_t2)) {
        // 驱动左腿的两个主动轮毂电机 (直接传入弧度)
        setMotorPosition(MOTOR_L1, left_t1);
        setMotorPosition(MOTOR_L2, left_t2);

        // 镜像右腿 (X 坐标镜像对称)
        float right_t1, right_t2;
        kinematics.inverseKinematics(-x, y, right_t1, right_t2);
        setMotorPosition(MOTOR_R1, right_t1);
        setMotorPosition(MOTOR_R2, right_t2);

        // 定时打印状态，包含设定角度与电机反馈角度，方便调试
        if (millis() - lastPrintTime > 200) {
            Serial.printf("Target: (%.1f, %.1f) | L Target: (%.2f, %.2f) Act: (%.2f, %.2f) | R Target: (%.2f, %.2f) Act: (%.2f, %.2f)\n", 
                          x, y, left_t1, left_t2, currentAngles[MOTOR_L1], currentAngles[MOTOR_L2],
                          right_t1, right_t2, currentAngles[MOTOR_R1], currentAngles[MOTOR_R2]);
            lastPrintTime = millis();
        }
        return true;
    } else {
        Serial.printf("Warning: Target (%.1f, %.1f) is unreachable!\n", x, y);
        return false;
    }
}

// 处理串口输入指令
void processSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("standby")) {
            currentMode = MODE_STANDBY;
            targetX = 0.0f;
            targetY = -100.0f;
            Serial.println("Switching to STANDBY mode.");
        } else if (input.equalsIgnoreCase("balance")) {
            if (imu.isConnected()) {
                currentMode = MODE_BALANCE;
                Serial.println("Switching to IMU BALANCE feedback mode.");
            } else {
                Serial.println("Error: Cannot enable balance mode because IMU is offline.");
            }
        } else {
            // 尝试解析为 X Y 坐标
            int spaceIndex = input.indexOf(' ');
            if (spaceIndex != -1) {
                float rawX = input.substring(0, spaceIndex).toFloat();
                float rawY = input.substring(spaceIndex + 1).toFloat();
                
                if (rawY > 0) {
                    Serial.println("提示: 机器人在下方工作，Y坐标应为负数 (如 -100)。已自动将其转为负值。");
                    rawY = -rawY;
                }

                currentMode = MODE_INTERACTIVE;
                targetX = rawX;
                targetY = rawY;
                Serial.printf("Switching to INTERACTIVE mode. Target: (%.1f, %.1f)\n", targetX, targetY);
            } else {
                Serial.println("未知指令。请输入 'standby', 'balance' 或 'X Y'。");
            }
        }
    }
}

// 根据当前模式计算轨迹位置并移动
void executeTrajectory() {
    switch (currentMode) {
        case MODE_STANDBY:
            moveToPosition(targetX, targetY);
            break;

        case MODE_BALANCE: {
            // IMU 自适应平衡反馈：当机身向前倾斜 (pitch > 0) 时，控制脚掌向前偏移维持中心平衡
            float kp = -1.2f;
            float compensatedX = imu.getPitch() * kp;
            float defaultY = -100.0f;
            moveToPosition(compensatedX, defaultY);
            break;
        }

        case MODE_INTERACTIVE:
            moveToPosition(targetX, targetY);
            break;
    }
}

// 更新板载 LED 呼吸灯 (使用 sine 曲线实现平滑呼吸效果)
void updateBreathingLed() {
    float breathingVal = sin(millis() * 0.003f); // 周期约 2.1s
    int brightness = (int)(127.5f * (1.0f + breathingVal));
    analogWrite(LED_PIN, brightness);
}
