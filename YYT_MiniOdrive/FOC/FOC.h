#ifndef FOC_H
#define FOC_H

#include "stm32g431xx.h"
#include "Data.h"
#include "tim.h"
#include <math.h>
#include "mt6816.h"


#define PI 3.1415926
#define _3PI_2 4.71238898038f
#define _PI_3 1.0471975512
#define _PI_2 1.57079632679
#define ONE_BY_SQRT3		0.57735026919f

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


extern float sensor_direction;
extern float pole_pairs;
extern float voltage_power_supply;
extern float zero_electric_angle;
extern float zero_electric_angle_norm;
extern uint32_t adc_value_in1;
extern uint32_t adc_value_in2;
extern uint32_t adc_value_in3;
extern uint32_t adc_value_in4;

void setPhaseVoltage(float Uq, float Ud, float angle_el);
float _normalizeAngle(float angle);
float _electricalAngle();
void Set_Pwm(uint8_t channel, uint16_t pulse_value);
void clarke_transform();
void park_transform(float Theta);

#endif 

