#ifndef PID_H
#define PID_H

#include "stm32g431xx.h"
#include "mt6816.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct{
    float P;
    float I;
    float D;
    float output;
    float limit;
    float output_ramp;
    float error_prev;//最后跟踪的误差
    float output_prev;//最后的pid输出值
    float integral_prev;//最后的积分输出值
    uint32_t timestamp_prev;//最后的时间戳
	  float error;
}PID;

float PID_operator(PID *pid,float error);

#endif 
