#include "ImuManager.h"
#include "LegConfig.h"

#define MPU6050_ADDR MPU6050_I2C_ADDR
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

ImuManager::ImuManager() 
    : connected(false), pitch(0.0f), roll(0.0f), yaw(0.0f),
      gyroXDegS(0.0f), gyroYDegS(0.0f), gyroZDegS(0.0f), lastTime(0),
      gyroXOffset(0.0f), gyroYOffset(0.0f), gyroZOffset(0.0f) {}

bool ImuManager::begin() {
    Serial.println("Initializing I2C Wire for IMU...");
    // 初始化 I2C，使用在 LegConfig.h 中配置的引脚
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(I2C_BUS_HZ);
    Wire.setTimeOut(20);

    // 检查 MPU6050 是否在线
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("Error: MPU6050 not found on I2C bus!");
        connected = false;
        return false;
    }

    Serial.println("MPU6050 detected. Configuring sensor...");
    
    // 唤醒 MPU6050 (写入 0x00 到 PWR_MGMT_1)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);

    // 设置陀螺仪量程为 +/- 250 deg/s (写入 0x00 到 GYRO_CONFIG)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();

    // 设置加速度计量程为 +/- 2g (写入 0x00 到 ACCEL_CONFIG)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();

    connected = true;
    lastTime = millis();

    // 校准陀螺仪零偏
    calibrateGyro();

    Serial.println("MPU6050 successfully initialized and calibrated.");
    return true;
}

void ImuManager::calibrateGyro() {
    Serial.println("Calibrating IMU Gyroscope (Keep robot still)...");
    long sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 200;

    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x43); // 陀螺仪数据起始寄存器
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)6, (bool)true);

        if (Wire.available() >= 6) {
            int16_t gx = (Wire.read() << 8) | Wire.read();
            int16_t gy = (Wire.read() << 8) | Wire.read();
            int16_t gz = (Wire.read() << 8) | Wire.read();
            sumX += gx;
            sumY += gy;
            sumZ += gz;
        }
        delay(3);
    }

    gyroXOffset = (float)sumX / samples;
    gyroYOffset = (float)sumY / samples;
    gyroZOffset = (float)sumZ / samples;

    Serial.printf("Gyro Offsets -> X: %.2f | Y: %.2f | Z: %.2f\n", gyroXOffset, gyroYOffset, gyroZOffset);
}

void ImuManager::update() {
    if (!connected) return;

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    if (dt <= 0.0f || dt > 0.5f) return; // 过滤异常延时

    // 读取加速度计与陀螺仪的 raw 数据 (共 14 字节)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, (bool)true);

    if (Wire.available() < 14) return;

    int16_t axRaw = (Wire.read() << 8) | Wire.read();
    int16_t ayRaw = (Wire.read() << 8) | Wire.read();
    int16_t azRaw = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // 跳过温度数据 (Temp_H, Temp_L)
    int16_t gxRaw = (Wire.read() << 8) | Wire.read();
    int16_t gyRaw = (Wire.read() << 8) | Wire.read();
    int16_t gzRaw = (Wire.read() << 8) | Wire.read();

    // 转换单位
    // 加速度计 +/- 2g 量程，灵敏度 16384 LSB/g
    float ax = (float)axRaw / 16384.0f;
    float ay = (float)ayRaw / 16384.0f;
    float az = (float)azRaw / 16384.0f;

    // 陀螺仪 +/- 250 deg/s 量程，灵敏度 131 LSB/(deg/s)
    float gx = ((float)gxRaw - gyroXOffset) / 131.0f;
    float gy = ((float)gyRaw - gyroYOffset) / 131.0f;
    float gz = ((float)gzRaw - gyroZOffset) / 131.0f;
    gyroXDegS = gx;
    gyroYDegS = gy;
    gyroZDegS = gz;

    // 计算加速度计姿态角 (单位: 度)
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    float accelRoll = atan2(ay, az) * 180.0f / M_PI;

    // 互补滤波融合
    // Pitch 角使用 Y 轴陀螺仪积分，Roll 角使用 X 轴陀螺仪积分
    pitch = 0.96f * (pitch + gy * dt) + 0.04f * accelPitch;
    roll = 0.96f * (roll + gx * dt) + 0.04f * accelRoll;
    yaw += gz * dt; // Yaw 轴只能累积积分

    // 将 Yaw 轴限制在 [-180, 180] 范围内
    if (yaw > 180.0f) yaw -= 360.0f;
    else if (yaw < -180.0f) yaw += 360.0f;
}
