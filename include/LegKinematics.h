#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <Arduino.h>


struct Point2D {
    float x;
    float y;
};

struct JointAngles {
    float theta1; // 左舵机角度 (弧度)
    float theta2; // 右舵机角度 (弧度)
};

class LegKinematics {
private:
    float d;  // 舵机间距一半
    float l1; // 主动臂长
    float l2; // 从动臂长

public:
    // 构造函数，默认使用 LegConfig.h 中的参数
    LegKinematics();
    LegKinematics(float d_val, float l1_val, float l2_val);

    // 逆运动学 (IK): 输入足端坐标 (x, y)，输出左右舵机弧度 (theta1, theta2)
    // 返回值: true 表示有点，false 表示超出工作空间
    bool inverseKinematics(float x, float y, float &theta1, float &theta2) const;
    bool inverseKinematics(Point2D pos, JointAngles &angles) const;

    // 顺运动学 (FK): 输入左右舵机弧度 (theta1, theta2)，输出足端坐标 (x, y)
    // 返回值: true 成功，false 失败 (如不满足闭链几何条件)
    bool forwardKinematics(float theta1, float theta2, float &x, float &y) const;
    bool forwardKinematics(JointAngles angles, Point2D &pos) const;



    // 角度弧度转换辅助函数
    static float radToDeg(float rad) { return rad * 180.0f / PI; }
    static float degToRad(float deg) { return deg * PI / 180.0f; }
};

#endif // LEG_KINEMATICS_H
