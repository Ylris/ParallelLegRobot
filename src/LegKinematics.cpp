#include "LegKinematics.h"
#include "LegConfig.h"
#include <math.h>

LegKinematics::LegKinematics() : d(LEG_D), l1(LEG_L1), l2(LEG_L2) {}

LegKinematics::LegKinematics(float d_val, float l1_val, float l2_val) : d(d_val), l1(l1_val), l2(l2_val) {}

bool LegKinematics::inverseKinematics(float x, float y, float &theta1, float &theta2) const {
    // Y 轴必须为负值 (工作空间在下方)
    if (y >= 0.0f) return false;

    // --- 左半部分 (左舵机位于 -d) ---
    float dx1 = x + d;
    float dy1 = y;
    float D1 = sqrt(dx1 * dx1 + dy1 * dy1);

    // 检查可达性 (三角形两边之和大于第三边，两边之差小于第三边)
    if (D1 > (l1 + l2) || D1 < fabs(l1 - l2)) return false;

    // 连线 A1P 的夹角
    float alpha1 = atan2(dy1, dx1);

    // 余弦定理求 beta1 (主动臂与 A1P 的夹角)
    // l2^2 = l1^2 + D1^2 - 2 * l1 * D1 * cos(beta1)
    float cosBeta1 = (l1 * l1 + D1 * D1 - l2 * l2) / (2.0f * l1 * D1);
    if (cosBeta1 > 1.0f) cosBeta1 = 1.0f;
    if (cosBeta1 < -1.0f) cosBeta1 = -1.0f;
    float beta1 = acos(cosBeta1);

    // 左侧肘部向外弯曲
    theta1 = alpha1 - beta1;

    // --- 右半部分 (右舵机位于 d) ---
    float dx2 = x - d;
    float dy2 = y;
    float D2 = sqrt(dx2 * dx2 + dy2 * dy2);

    // 检查可达性
    if (D2 > (l1 + l2) || D2 < fabs(l1 - l2)) return false;

    // 连线 A2P 的夹角
    float alpha2 = atan2(dy2, dx2);

    // 余弦定理求 beta2
    float cosBeta2 = (l1 * l1 + D2 * D2 - l2 * l2) / (2.0f * l1 * D2);
    if (cosBeta2 > 1.0f) cosBeta2 = 1.0f;
    if (cosBeta2 < -1.0f) cosBeta2 = -1.0f;
    float beta2 = acos(cosBeta2);

    // 右侧肘部向外弯曲
    theta2 = alpha2 + beta2;

    return true;
}

bool LegKinematics::inverseKinematics(Point2D pos, JointAngles &angles) const {
    return inverseKinematics(pos.x, pos.y, angles.theta1, angles.theta2);
}

bool LegKinematics::forwardKinematics(float theta1, float theta2, float &x, float &y) const {
    // 1. 计算主动臂末端关节 B1 和 B2 的位置
    float x1 = -d + l1 * cos(theta1);
    float y1 = l1 * sin(theta1);

    float x2 = d + l1 * cos(theta2);
    float y2 = l1 * sin(theta2);

    // 2. 计算 B1-B2 间距
    float dx = x2 - x1;
    float dy = y2 - y1;
    float D = sqrt(dx * dx + dy * dy);

    // 几何限制检查：两关节间距不能大于两从动臂长之和，也不能太小
    if (D > 2.0f * l2 || D < 1e-4f) return false;

    // 3. 计算中点 M
    float mx = (x1 + x2) * 0.5f;
    float my = (y1 + y2) * 0.5f;

    // 4. 计算垂直偏移距离 h
    float h = sqrt(l2 * l2 - (D * D) * 0.25f);

    // 5. 确定足端位置 P (足端始终向下，由于右手系中 y 向下为负，取减号)
    x = mx + (h / D) * dy;
    y = my - (h / D) * dx;

    return true;
}

bool LegKinematics::forwardKinematics(JointAngles angles, Point2D &pos) const {
    return forwardKinematics(angles.theta1, angles.theta2, pos.x, pos.y);
}


