#ifndef __POSITION_PID_H__
#define __POSITION_PID_H__

#include "main.h"

typedef struct {
    float target_val;        //目标值
    float actual_val;        //实际值
    float err;               //偏差值
    float err_last;          //上一个偏差值
    float Kp, Ki, Kd;        //比例、积分、微分系数
    float integral;          //积分值
} _pid;


void PID_param_init();
float PID_realize_q(float target,float temp_val);
float PID_realize_d(float target,float temp_val);

#endif 
