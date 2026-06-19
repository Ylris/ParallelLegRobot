#include "Motor.h"


float shaft_angle=0,open_loop_timestamp=0;

#ifndef MOTOR_SKIP_BOOT_SWEEP
#define MOTOR_SKIP_BOOT_SWEEP 0
#endif


void Motor_init()
{
    float angle;

#if MOTOR_AUTO_ELECTRIC_ZERO
	pole_pairs = MOTOR_POLE_PAIRS;
#endif

#if MOTOR_ALIGN_ONLY
	/* Align-only mode for on-leg calibration.
	   Slowly sweeps ONE electrical cycle (25.7 deg mechanical for pp=14)
	   to drag the rotor to a known electrical position, then reads the
	   encoder to compute the electrical zero.  Safe on assembled legs. */
	pole_pairs = MOTOR_POLE_PAIRS;

	/* Sweep forward one full electrical cycle at low speed */
	for (int i = 0; i <= 200; i++) {
		float sweep_angle = _3PI_2 + _2PI * (float)i / 200.0f;
		setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, sweep_angle);
		HAL_Delay(3);
	}
	/* Sweep back to the target angle */
	for (int i = 200; i >= 0; i--) {
		float sweep_angle = _3PI_2 + _2PI * (float)i / 200.0f;
		setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, sweep_angle);
		HAL_Delay(3);
	}
	/* Hold at target angle and let rotor fully settle */
	setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, _3PI_2);
	HAL_Delay(500);
	zero_electric_angle = _normalizeAngle((float)(sensor_direction * pole_pairs) * MT6816_Get_AngleData() + MOTOR_AUTO_ELECTRIC_ZERO_OFFSET);
	setPhaseVoltage(0, 0, _3PI_2);
	return;
#endif

#if MOTOR_SKIP_BOOT_SWEEP
	pole_pairs = MOTOR_POLE_PAIRS;
	zero_electric_angle=MOTOR_ZERO_ELECTRIC_ANGLE;
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
	setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0,_3PI_2);
	HAL_Delay(300);
#if MOTOR_AUTO_ELECTRIC_ZERO
	zero_electric_angle = _normalizeAngle((float)(sensor_direction * pole_pairs) * MT6816_Get_AngleData() + MOTOR_AUTO_ELECTRIC_ZERO_OFFSET);
#else
	zero_electric_angle=MOTOR_ZERO_ELECTRIC_ANGLE;
#endif
  setPhaseVoltage(0, 0,_3PI_2);

}



//开环速度控制
float velocityOpenloop(float target_velocity)
{
	return velocityOpenloopVoltage(target_velocity, 6.0f);
}

float velocityOpenloopVoltage(float target_velocity, float target_voltage)
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
	
    Uq = target_voltage;

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
