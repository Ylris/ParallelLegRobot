#include "Motor.h"


float shaft_angle=0,open_loop_timestamp=0;

#ifndef OPEN_SOURCE_SAFE_HOLD
#define OPEN_SOURCE_SAFE_HOLD 0
#endif


void Motor_init()
{
    float angle;

#if OPEN_SOURCE_SAFE_HOLD
	zero_electric_angle=0.0f;
	setPhaseVoltage(0, 0,_3PI_2);
	HAL_Delay(50);
	setPhaseVoltage(1.6f, 0,_3PI_2);
	HAL_Delay(350);
	zero_electric_angle=_normalizeAngle((float)(sensor_direction * pole_pairs) * MT6816_Get_AngleData());
	setPhaseVoltage(0, 0,_3PI_2);
	return;
#endif

    for(int i=0; i<=500*1; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(2.5, 0,  angle);
		HAL_Delay(1);
	}
	
	for(int i=500*1; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setPhaseVoltage(2.5, 0,  angle);
		HAL_Delay(1);
	}
	setPhaseVoltage(0, 0,  angle);
	HAL_Delay(100);
	setPhaseVoltage(3, 0,_3PI_2);
	HAL_Delay(300);
	//zero_electric_angle=_electricalAngle();
	zero_electric_angle=3.585681f;
  setPhaseVoltage(0, 0,_3PI_2);

}



//开环速度控制
float velocityOpenloop(float target_velocity)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = SysTick->VAL; //读取滴答定时器的值
	if(now_us<open_loop_timestamp)Ts = (float)(open_loop_timestamp - now_us)/9*1e-6;//转化为时间
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/9*1e-6;
	open_loop_timestamp=now_us;  //储存这次的值

    if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	

  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 
	
    Uq = 3;

  setPhaseVoltage(Uq,  0, shaft_angle*pole_pairs);
	
	return Uq;
	


}

//位置闭环


void PID_Pos_Set(float P, float I, float D, float limit)
{
    PID_Pos.P = P;
    PID_Pos.I = I;
    PID_Pos.D = D;
    PID_Pos.limit = limit;
}

void PID_Pos_Cur_Set(float P, float I, float D, float limit)
{
    PID_Pos_Cur.P = P;
    PID_Pos_Cur.I = I;
    PID_Pos_Cur.D = D;
    PID_Pos_Cur.limit = limit;
}

void PID_Pos_Vel_Cur_Set(float P, float I, float D, float limit)
{
    PID_Pos_Vel_Cur.P = P;
    PID_Pos_Vel_Cur.I = I;
    PID_Pos_Vel_Cur.D = D;
    PID_Pos_Vel_Cur.limit = limit;
}


void PositionCloseloop(float position)
{
    float output;
    output=PID_operator(&PID_Pos,(position-sensor_direction*MT6816_Get_FullAngleData()));
    setPhaseVoltage(output,0,_electricalAngle());
		func_flag=1;
}

//速度闭环


void PID_Vel_Set(float P, float I, float D, float limit)
{
    PID_Vel.P = P;
    PID_Vel.I = I;
    PID_Vel.D = D;
    PID_Vel.limit = limit;
}

void PID_Vel_Cur_Set(float P, float I, float D, float limit)
{
    PID_Vel_Cur.P = P;
    PID_Vel_Cur.I = I;
    PID_Vel_Cur.D = D;
    PID_Vel_Cur.limit = limit;
}

void VelocityCloseloop(float Velocity)
{
    float output;
		PID_Vel.error=Velocity-sensor_direction*MT6816_Get_Velocity_L();
    output=PID_operator(&PID_Vel,(Velocity-sensor_direction*MT6816_Get_Velocity_L()));
    setPhaseVoltage(output,0,_electricalAngle());
		func_flag=2;
}

//电流环
void PID_Cur_Q_Set(float P, float I, float D, float limit)
{
    PID_Cur_Q.P = P;
    PID_Cur_Q.I = I;
    PID_Cur_Q.D = D;
    PID_Cur_Q.limit = limit;
}

void PID_Cur_D_Set(float P, float I, float D, float limit)
{
    PID_Cur_D.P = P;
    PID_Cur_D.I = I;
    PID_Cur_D.D = D;
    PID_Cur_D.limit = limit;
}

void CurrentCloseloop(float current_q,float current_d)
{
    float Ud,Uq;
		Uq=PID_operator(&PID_Cur_Q,(current_q-Iq));
    Ud=PID_operator(&PID_Cur_D,(current_d-Id));
    setPhaseVoltage(Uq,Ud,_electricalAngle());
		func_flag=3;
	
}




//位置速度电流闭环

void Position_Velocity_CurrentCloseloop(float position)
{
    float output;
    output=PID_operator(&PID_Pos_Vel_Cur,(position-sensor_direction*MT6816_Get_FullAngleData()));
		Velocity_CurrentCloseloop(output);
}

//速度电流闭环
void Velocity_CurrentCloseloop(float velocity)
{
    float output;
    output=PID_operator(&PID_Vel_Cur,(velocity-sensor_direction*MT6816_Get_Velocity_L()));
		PID_output=output;
		if(output>2) output=2;
		if(output<-2) output=-2;
		CurrentCloseloop(output,0);
		func_flag=4;
}

//位置电流闭环
void Position_CurrentCloseloop(float position)
{
    float output;
    output=PID_operator(&PID_Pos_Cur,(position-sensor_direction*MT6816_Get_FullAngleData()));
		if(output>2) output=2;
		if(output<-2) output=-2;
    CurrentCloseloop(output,0);
		func_flag=5;
}
