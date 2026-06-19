#ifndef DATA_H
#define DATA_H


#include "stm32g431xx.h"
#include "Pid.h"
#include "main.h"


#define IIC I2C2 //磁编码器通信


extern float sensor_direction;//编码器方向
extern float pole_pairs;//电机极对数
extern float voltage_power_supply ;//供电电压
extern float zero_electric_angle;


extern PID PID_Pos;//位置PID
extern PID PID_Pos_Cur;
extern PID PID_Pos_Vel_Cur;
extern PID PID_Vel;//速度PID
extern PID PID_Vel_Cur;
extern PID PID_Cur_Q;//电流Q轴PID
extern PID PID_Cur_D;//电流D轴PID
extern float position;//位置
extern float velocity;//速度
extern float current;//电流


extern float gain_a,gain_b,gain_c;//欧姆定律系数
extern float ia,ib,ic;//采集的3路电流
extern float bias_ia,bias_ib,bias_ic;//采集的3路电流
extern float Ialpha,Ibeta;
extern float Id,Iq;
extern float bias_vbus;
extern float vbus;
extern uint32_t adc_value_in1_raw;
extern uint32_t adc_value_in2_raw;
extern uint32_t adc_value_in3_raw;
extern uint32_t adc_value_in4_raw;
extern uint32_t adc_value_in1;
extern uint32_t adc_value_in2;
extern uint32_t adc_value_in3;
extern uint32_t adc_value_in4;
extern float u_a,u_b,u_c;
extern float volts_to_amps_ratio;
extern float shunt_resistor ;//采样电阻值
extern float amp_gain;//运放增益
extern float adc_cpr;
extern float vcc;
extern float convert;
extern float current_a,current_b;//帕克变换的值

//VOFA+上位机通信相关
extern float Pos_set;
extern float Velo_set;
extern float IQ_set;
extern float ID_set;
extern float mode;
extern float uq_limit;
extern float func_flag;
extern float PID_output;




#endif 
