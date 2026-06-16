#include <Arduino.h>
#include "driver/uart.h" // 引入 ESP32 底层 UART 驱动
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

// 函数声明
void processSerialInput();
void executeTrajectory();
bool moveToPosition(float x, float y);
void updateBreathingLed();
void setMotorPosition(int motorIndex, float rad);
void setMotorMode(int motorIndex, int modeVal);


void setup() {
    // 1. 初始化调试串口
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n==============================================");
    Serial.println("ESP32-C3 Parallel Leg Robot Closed-Loop Init...");
    Serial.println("==============================================");

    // 2. 初始化 IMU
    imu.begin();

    // 3. 初始化用于控制电机的 Hardware UART1
    // 初始将 TX 映射到第一个电机引脚，RX 设为 -1 (仅发送)
    Serial1.begin(MOTOR_BAUD, SERIAL_8N1, -1, MOTOR_PINS[0]);
    Serial.println("Hardware UART1 configured for closed-loop motors.");

    // 4. 将所有 6 个电机初始化为位置闭环模式 (M0=0!)
    Serial.println("Configuring all motors to Position Closed-loop Mode (M0=0)...");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        setMotorMode(i, 0);
        delay(50); // 适当延时防拥堵
    }
    Serial.println("All motors configured.");

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
    // 1. 更新 IMU 数据
    imu.update();

    // 2. 处理串口输入
    processSerialInput();

    // 3. 根据模式执行轨迹
    executeTrajectory();

    // 4. 发送 IMU 数据到 PioPulse (FireWater 协议：前 3 位为姿态角，后 3 位为平移量)
    if (imu.isConnected()) {
        Serial.printf("imu:%.2f,%.2f,%.2f,0.0,0.0,0.0\n", 
                      imu.getPitch(), imu.getRoll(), imu.getYaw());
    }


    // 5. 更新板载 LED 呼吸灯
    updateBreathingLed();

    // 6. 延时控制更新频率 (约 50Hz, 20ms)
    delay(20);
}


// 物理层切换引脚并向目标电机发送弧度指令
void setMotorPosition(int motorIndex, float rad) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;

    // 1. 动态切换 UART1 TX 信号到目标电机的 GPIO 引脚
    uart_set_pin(UART_NUM_1, MOTOR_PINS[motorIndex], UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 2. 发送 ASCII 闭环控制协议指令: P0=<rad>!
    Serial1.printf("P0=%.4f!", rad);

    // 3. 强制刷空串口发送缓存，等待电信号发送完毕后再切换引脚
    Serial1.flush();
}

// 物理层切换引脚并设定电机的控制模式
void setMotorMode(int motorIndex, int modeVal) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;

    uart_set_pin(UART_NUM_1, MOTOR_PINS[motorIndex], UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    Serial1.printf("M0=%d!", modeVal);
    Serial1.flush();
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

        // 定时打印状态
        if (millis() - lastPrintTime > 200) {
            Serial.printf("Target: (%.1f, %.1f) | Left Rad: (%.2f, %.2f) | Right Rad: (%.2f, %.2f)\n", 
                          x, y, left_t1, left_t2, right_t1, right_t2);
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
