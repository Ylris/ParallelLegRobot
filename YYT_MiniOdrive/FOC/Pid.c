#include "Pid.h"


float PID_operator(PID *pid,float error)
{
    
	  uint32_t timestamp_now = Get_Timestamp_us();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
    // P环
    float proportional = pid->P * error;
    
		// I环
    float integral = pid->integral_prev + pid->I*Ts*0.5f*(error + pid->error_prev);
    integral = _constrain(integral, - pid->limit,  pid->limit);

    // D环
    float derivative = pid->D*(error - pid->error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, - pid->limit,  pid->limit);

    // 保存值（为了下一次循环）
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;
    return output;
}