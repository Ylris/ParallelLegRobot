#include "FOC.h"

#ifndef FOC_MODULATION_LIMIT
#define FOC_MODULATION_LIMIT 0.577f
#endif


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


float _electricalAngle(){
		zero_electric_angle_norm=_normalizeAngle((float)(sensor_direction * pole_pairs) * MT6816_Get_AngleData() -zero_electric_angle);
  	return  _normalizeAngle((float)(sensor_direction * pole_pairs) * MT6816_Get_AngleData() -zero_electric_angle);
}

static float clamp_unit(float value)
{
	if (value < 0.0f) return 0.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

void clarke_transform()
{
	Ialpha = ia;
		Ibeta  = (ib - ic) * ONE_BY_SQRT3;
}

void park_transform(float Theta)
{
	Id =  Ialpha * cos(Theta) + Ibeta * sin(Theta);
	Iq = -Ialpha * sin(Theta) + Ibeta * cos(Theta);
}


void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	if(Uq>uq_limit) Uq=uq_limit;
	if(Uq<-uq_limit) Uq=-uq_limit;

	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = sqrtf(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if(Uout> FOC_MODULATION_LIMIT)Uout= FOC_MODULATION_LIMIT;
	if(Uout<-FOC_MODULATION_LIMIT)Uout=-FOC_MODULATION_LIMIT;
	
	sector = (angle_el / _PI_3) + 1;
	T1 = sqrt(3)*sin(sector*_PI_3 - angle_el) * Uout;
	T2 = sqrt(3)*sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	//扇区计算
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	Ta = clamp_unit(Ta);
	Tb = clamp_unit(Tb);
	Tc = clamp_unit(Tc);
	Set_Pwm(1, Ta*8400);
	Set_Pwm(2, Tb*8400);
	Set_Pwm(3, Tc*8400);
//	Set_Pwm(1, 2100);
//	Set_Pwm(2, 2100);
//	Set_Pwm(3, 2100);
	u_a=Ta*8400;
	u_b=Tb*8400;
	u_c=Tc*8400;
}

// 设置 PWM 占空比函数
void Set_Pwm(uint8_t channel, uint16_t pulse_value)
{
    // 根据通道设置 PWM 占空比
    switch (channel)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_value);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_value);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_value);
            break;
        default:
            // 错误处理
            break;
    }
}
