#include "axdr_zengliang_pid.h"

__pid pid4;
void PID_param_init_zls() 
{
    // 初始化参数 //
    //printf("PID-init begin \n");
    pid4.target_val = 0.0;
    pid4.actual_val = 0.0;
    pid4.err = 0.0;
    pid4.err_last = 0.0;
    pid4.err_next = 0.0;
    pid4.Kp = 0.21;
    pid4.Ki = 0.80;
    pid4.Kd = 0.01;
    //printf("PID_init end \n");
}

float PID_realize_zls(float temp_val) 
{
    // 目标值只在这里参与计算，计算目标值与实际值的误差 *
    pid4.err = pid4.target_val - temp_val;
    /*PID 算法实现 */
    float increment_val = pid4.Kp * (pid4.err - pid4.err_next) + pid4.Ki * pid4.err +
        pid4.Kd * (pid4.err - 2 * pid4.err_next + pid4.err_last);

    // 累加 *
    pid4.actual_val += increment_val;

    // 传递误差 *
    pid4.err_last = pid4.err_next;
    pid4.err_next = pid4.err;

    // 返回的是经过 pid4 运算以后的值 *
    return pid4.actual_val;
}