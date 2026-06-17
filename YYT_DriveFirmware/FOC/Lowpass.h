#ifndef LOWPASS_H
#define LOWPASS_H


#include "stm32g431xx.h"
#include "FOC.h"
#include "Usart.h"
#include "main.h"
#include "mt6816.h"

typedef struct 
{
	float Tf; 
	float y_prev; 
	uint32_t timestamp_prev;  
} LowPassFilter;

extern LowPassFilter  LPF_current,LPF_velocity;

void LOWPass_Init();
float LowPassFilter_operator(LowPassFilter *Lfi,Velocity_Time_Pack prev_time,Velocity_Time_Pack current_time);


#endif