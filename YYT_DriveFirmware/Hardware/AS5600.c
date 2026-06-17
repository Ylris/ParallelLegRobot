#include "AS5600.h"


//extern I2C_HandleTypeDef hi2c2;  // 使用 I2C2


//uint16_t I2C_getRawCount(I2C_HandleTypeDef *hi2c)
//{
//    uint8_t angleData[2] = {0};   // 用于存储从 AS5600 读取的高低字节数据
//    uint16_t angle = 0;

//    // 从 AS5600 的 RAW_Angle_Hi 寄存器读取 2 字节数据
//    if (HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS, RAW_Angle_Hi, I2C_MEMADD_SIZE_8BIT, angleData, 2, 50) == HAL_OK)
//    {
//        // 将读取的高低字节数据合成为一个 16 位的原始角度值
//        angle = (angleData[0] << 8) | angleData[1];
//    }
//    else
//    {
//        // 处理读取错误
//			return 0xFFFF;
//    }

//    return angle;
//}


//// 编码器结构体
//ENCODER_TypeDef E1;

//// 获取角度
//float Get_Angel_L(ENCODER_TypeDef *CODERx, I2C_HandleTypeDef *hi2c)
//{
//    float angle_data = 0, d_angle = 0;

//    CODERx->cpr = AS5600_CPR;         // 设置编码器的 CPR
//    angle_data = I2C_getRawCount(hi2c);  // 读取 AS5600 原始角度数据

//    // 计算当前角度和上一次角度的差值
//    d_angle = angle_data - CODERx->angle_data_prev;

//    // 检查角度跳变，调整全转偏移量
//    if (fabs(d_angle) > (0.8f * CODERx->cpr))
//        CODERx->full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI;

//    // 更新上一次的角度数据
//    CODERx->angle_data_prev = angle_data;

//    // 计算并返回当前角度，包含多圈角度
//    return (CODERx->full_rotation_offset + (angle_data / (float)CODERx->cpr) * _2PI);
//}

//// 获取角度的外部接口
//float Get_Angel(void)
//{
//    return Get_Angel_L(&E1, &hi2c2);  // 使用 I2C2
//}

//// 获取速度
//float Get_Velocity_L(ENCODER_TypeDef *CODERx)
//{
//    unsigned long now_us;
//    float Ts, angle_c, vel;

//    // 计算采样时间
//    now_us = SysTick->VAL; // 或者使用其他获取当前时间的函数
//    if (now_us < CODERx->velocity_calc_timestamp)
//        Ts = (float)(CODERx->velocity_calc_timestamp - now_us) / 9 * 1e-6;
//    else
//        Ts = (float)(0xFFFFFF - now_us + CODERx->velocity_calc_timestamp) / 9 * 1e-6;

//    // 快速修复奇怪情况（微秒溢出）
//    if (Ts == 0 || Ts > 0.5) Ts = 1e-3;

//    // 当前角度
//    angle_c = Get_Angel();
//    // 速度计算
//    vel = (angle_c - CODERx->angle_prev) / Ts;

//    // 保存变量以供将来使用
//    CODERx->angle_prev = angle_c;
//    CODERx->velocity_calc_timestamp = now_us;

//    return vel;
//}

//uint32_t TIM2_GetTimestamp(void)
//{
//    return __HAL_TIM_GET_COUNTER(&htim2); // 返回当前计数值
//}

//float Calculate_Time_Interval(uint32_t *prev_timestamp)
//{
//    uint32_t now = TIM2_GetTimestamp();
//    float interval;

//    // 处理定时器溢出情况
//    if (now >= *prev_timestamp)
//    {
//        interval = (now - *prev_timestamp) / 1000000.0f; // 转换为秒
//    }
//    else
//    {
//        // 处理溢出
//        interval = ((0xFFFFFFFF - *prev_timestamp) + now + 1) / 1000000.0f;
//    }

//    // 更新上一次时间戳
//    *prev_timestamp = now;

//    return interval;
//}

//float Get_Velocity_L(ENCODER_TypeDef *CODERx)
//{
//    static uint32_t prev_timestamp = 0; // 上一次时间戳
//    float Ts, angle_c, vel;

//    // 计算采样时间间隔（单位：秒）
//    Ts = Calculate_Time_Interval(&prev_timestamp);

//    // 检查时间间隔异常
//    if (Ts <= 0 || Ts > 0.5f)
//    {
//        return 0.0f; // 异常情况下返回 0
//    }

//    // 获取当前角度
//    angle_c = Get_Angel();

//    // 检查角度跳变，防止速度计算出错
//    if (fabs(angle_c - CODERx->angle_prev) > _2PI)
//    {
//        CODERx->angle_prev = angle_c; // 修正上一次角度
//        return 0.0f;
//    }

//    // 计算速度
//    vel = (angle_c - CODERx->angle_prev) / Ts;

//    // 更新上一次角度
//    CODERx->angle_prev = angle_c;

//    return vel;
//}

//ENCODER_TypeDef E2;
//// 无滤波速度
//float Get_Velocity2(void)
//{
//    return Get_Velocity_L(&E2);
//}

//// 有滤波速度
//float Get_Velocity(void)
//{
//    float vel_ori = Get_Velocity_L(&E2);
//    float vel_flit = LowPassFilter_operator(&LPF_velocity, vel_ori); // 假设有低通滤波器
//    return vel_flit;   
//}

//// 获取未跟踪的角度
//float Get_Angel_Notrack(void)
//{
//    return I2C_getRawCount(&hi2c2) * 0.08789 * PI / 180; // 使用 I2C2
//}
