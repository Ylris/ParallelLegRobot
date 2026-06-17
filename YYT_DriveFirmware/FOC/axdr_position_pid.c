#include "axdr_position_pid.h"
_pid pid_q,pid_d;
void PID_param_init() 
{
    // 初始化参数 //
    //printf("PID_init begin \n");
    pid_q.target_val = 0.0;
    pid_q.actual_val = 0.0;
    pid_q.err = 0.0;
    pid_q.err_last = 0.0;
    pid_q.integral = 0.0;
    pid_q.Kp = 0.012835f;
    pid_q.Ki = 56.23704f;
    pid_q.Kd = 0.0;
	
    //pid的方向考量

	pid_d.target_val = 0.0;
    pid_d.actual_val = 0.0;
    pid_d.err = 0.0;
    pid_d.err_last = 0.0;
    pid_d.integral = 0.0;
    pid_d.Kp = 0.011175f;
    pid_d.Ki = 56.23704f;
    pid_d.Kd = 0.0;

}

float PID_realize_q(float target,float temp_val) 
{
		
		pid_q.target_val = target;
    // 目标值只在这里参与计算，计算目标值与实际值的误差 *
    pid_q.err = pid_q.target_val - temp_val;
    // 误差累积 *
    pid_q.integral += pid_q.err;

    // pid_q 算法实现 *
    pid_q.actual_val = pid_q.Kp * pid_q.err + pid_q.Ki * pid_q.integral + pid_q.Kd * (pid_q.err - pid_q.err_last);

    // 误差传递 *
    pid_q.err_last = pid_q.err;
	
    // 返回的是经过 pid_q 运算以后的值 *
    return pid_q.actual_val;
}


float PID_realize_d(float target,float temp_val) 
{
		
		pid_d.target_val = target;
    // 目标值只在这里参与计算，计算目标值与实际值的误差 *
    pid_d.err = pid_d.target_val - temp_val;
    // 误差累积 *
    pid_d.integral += pid_d.err;

    // pid_d 算法实现 *
    pid_d.actual_val = pid_d.Kp * pid_d.err + pid_d.Ki * pid_d.integral + pid_d.Kd * (pid_d.err - pid_d.err_last);

    // 误差传递 *
    pid_d.err_last = pid_d.err;
	
    // 返回的是经过 pid_d 运算以后的值 *
    return pid_d.actual_val;
}
