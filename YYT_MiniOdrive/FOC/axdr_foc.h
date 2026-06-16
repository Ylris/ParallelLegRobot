#ifndef _FOC_H
#define _FOC_H

	
#include "main.h"

extern volatile float vel_cnt, vel_angle_0, vel_angle_1, vel, last_vel, low_alpha, vel_ref, vel_filt, xian_vel, xian_vel_filt;
extern volatile float Iq_ref, vel_ref;
extern volatile float pos_cnt, pos_angle, last_pos_angle, pos_angle_360, pos_ref, circle_count, all_angle;

#define LOW_PASS_FILTER(prev_value, new_value, alpha) ((alpha) * (prev_value) + (1 - (alpha)) * (new_value))
#define SQUARE_ROOT_SUM(x, y) (sqrt((x) * (x) + (y) * (y)))

void VF_RUN(void);
void IF_RUN(void);
void Calibration_Zero(void);
void Voltage_Open_Loop(void);

void Current_Closed_Loop(void);
void Velocity_Closed_Loop(void);
void Position_Closed_Loop(void);
#endif 