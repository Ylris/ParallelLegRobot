#ifndef Motor_H
#define Motor_H

#include "stm32g431xx.h"
#include "FOC.h"
#include "Pid.h"
//#include "Adc.h"
#include "Data.h"
#include "mt6816.h"

#ifndef MOTOR_ZERO_ELECTRIC_ANGLE
#define MOTOR_ZERO_ELECTRIC_ANGLE 3.585681f
#endif

#ifndef MOTOR_AUTO_ELECTRIC_ZERO
#define MOTOR_AUTO_ELECTRIC_ZERO 0
#endif

#ifndef MOTOR_ALIGN_VOLTAGE
#define MOTOR_ALIGN_VOLTAGE 3.0f
#endif

#ifndef MOTOR_AUTO_ELECTRIC_ZERO_OFFSET
#define MOTOR_AUTO_ELECTRIC_ZERO_OFFSET 0.0f
#endif

#ifndef MOTOR_POLE_PAIRS
#define MOTOR_POLE_PAIRS 11.0f
#endif

//初始化
void Motor_init();
//开环速度控制
float velocityOpenloop(float target_velocity);
float velocityOpenloopVoltage(float target_velocity, float target_voltage);
//闭环位置控制(单闭环无电流环)
void PID_Pos_Set(float P, float I, float D, float limit);// PID_Pos_Set(0.10,0.01,0,6);
void PID_Pos_Cur_Set(float P, float I, float D, float limit);
void PID_Pos_Vel_Cur_Set(float P, float I, float D, float limit);

void PositionCloseloop(float position);//位置
//速度闭环(单闭环无电流环)
void PID_Vel_Set(float P, float I, float D, float limit);// PID_Pos_Set(0.001,0.01,0,6);
void PID_Vel_Cur_Set(float P, float I, float D, float limit);
void VelocityCloseloop(float Velocity);
//电流环
void PID_Cur_D_Set(float P, float I, float D, float limit);
void PID_Cur_Q_Set(float P, float I, float D, float limit);
//void PID_Cur_Set(float P, float I, float D, float limit);//	PID_Cur_Set(10,5,0,6);
void CurrentCloseloop(float current_q,float current_d);
//位置速度环
void Position_VelocityCloseloop(float position);// PID_Pos_Set(0.10,0.01,0,6);// PID_Pos_Set(0.10,1.5,0,6);
void Position_Velocity_CurrentCloseloop(float position);
//速度电流环
void Velocity_CurrentCloseloop(float velocity);
//位置电流环
void Position_CurrentCloseloop(float position);



extern PID PID_Pos;
extern PID PID_Vel;
extern float position;
extern float velocity;
extern float current;



#endif 
