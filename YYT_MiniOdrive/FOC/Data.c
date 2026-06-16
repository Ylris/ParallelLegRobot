#include "Data.h"


//电机相关
float sensor_direction =1;//编码器方向
float pole_pairs = 14;//电机极对数
float voltage_power_supply =24;//供电电压
float Ualpha=0,Ubeta=0,Ua=0,Ub=0,Uc=0;
float zero_electric_angle=0;
float zero_electric_angle_norm=0;

//PID相关
PID PID_Pos;//位置PID
PID PID_Pos_Cur;//位置PID
PID PID_Pos_Vel_Cur;
PID PID_Vel;//速度PID
PID PID_Vel_Cur;//速度PID
PID PID_Cur_Q;//电流Q轴PID
PID PID_Cur_D;//电流D轴PID
float position;//位置
float velocity;//速度
float current;//电流

//电流采集相关
float gain_a,gain_b,gain_c;
float ia,ib,ic;//采集的3路电流
float bias_ia,bias_ib,bias_ic;//采集的3路电流
float Ialpha,Ibeta;
float Id,Iq;
float bias_vbus;
float vbus;
uint32_t adc_value_in1_raw;
uint32_t adc_value_in2_raw;
uint32_t adc_value_in3_raw;
uint32_t adc_value_in4_raw;
uint32_t adc_value_in1;
uint32_t adc_value_in2;
uint32_t adc_value_in3;
uint32_t adc_value_in4;
float u_a,u_b,u_c;
float volts_to_amps_ratio;
float shunt_resistor = 0.01;//采样电阻值
float amp_gain  = 50;//运放增益
float adc_cpr=4096;
float vcc=3.3;
float convert;
float current_a,current_b;//帕克变换的值

//VOFA+上位机通信相关
float Pos_set=3.14;
float Velo_set=100;
float IQ_set=0;
float ID_set=0;

float mode=-1;
float uq_limit=3;
float func_flag=0;
float PID_output;

