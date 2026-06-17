#ifndef __ZENGLIANG_PID_H__
#define __ZENGLIANG_PID_H__

#include "main.h"
#include "axdr_position_pid.h"
typedef struct {
    float target_val;        //目标值
    float actual_val;        //实际值
    float err;               //当前偏差值
    float err_next;          //下一个偏差值
    float err_last;          //最后一个偏差值
    float Kp, Ki, Kd;        //比例、积分、微分系数
} __pid;

void PID_param_init_zls();
float PID_realize_zls(float temp_val);

#endif 


