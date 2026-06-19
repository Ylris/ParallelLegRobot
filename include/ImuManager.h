#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

class ImuManager {
public:
    ImuManager();
    bool begin();
    void update();
    
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }
    float getYaw() const { return yaw; }
    float getPitchRateDegS() const { return gyroYDegS; }
    float getRollRateDegS() const { return gyroXDegS; }
    float getYawRateDegS() const { return gyroZDegS; }
    bool isConnected() const { return connected; }

private:
    bool connected;
    float pitch;
    float roll;
    float yaw;
    float gyroXDegS;
    float gyroYDegS;
    float gyroZDegS;
    unsigned long lastTime;

    // 校准偏移量
    float gyroXOffset;
    float gyroYOffset;
    float gyroZOffset;

    void calibrateGyro();
};

#endif // IMU_MANAGER_H
