
#include "Lowpass.h"


LowPassFilter  LPF_current,LPF_velocity;

void LOWPass_Init()
{
    LPF_current.timestamp_prev= Get_Timestamp_us();
    LPF_current.Tf=0.05;
		LPF_current.y_prev = 0.0;

    LPF_velocity.timestamp_prev= Get_Timestamp_us();
    LPF_velocity.Tf=0.05;
		LPF_velocity.y_prev = 0.0;
}


float LowPassFilter_operator(LowPassFilter *Lfi,Velocity_Time_Pack prev_time,Velocity_Time_Pack current_time)
{

		float Ts;

	if(current_time.timestamp>prev_time.timestamp)Ts = (float)( current_time.timestamp-prev_time.timestamp)*1e-6;
	else
		Ts = (float)(0xFFFFFFFF + current_time.timestamp - prev_time.timestamp)*1e-6;

	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

    float alpha = Lfi->Tf/(Lfi->Tf + Ts);
//		float alpha=0.5;
    float y = alpha*Lfi->y_prev + (1.0f - alpha)*current_time.velocity;
    Lfi->y_prev = y;
    return y;
}
