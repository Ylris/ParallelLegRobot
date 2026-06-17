#ifndef __TRANS_H
#define __TRANS_H
#include "main.h"
#include "math.h"

#define SQRT3 1.7320508075688772935274463415059F
#define PI 3.1415926535897932384626433832795F

//extern float theta_angle;
extern volatile float theta_angle;
extern volatile float theta_hudu;
extern volatile float sin_val;
extern volatile float cos_val;
extern volatile float Iq, Id, Ialpha, Ibeta;
extern volatile float Vd, Vq, Valpha, Vbeta;
extern volatile float Ia, Ib, Ic;
extern volatile float Tpwm_temp, Udc_temp;
extern volatile float Tcmp1, Tcmp2, Tcmp3;

void tran_angle(float angle);
void sin_cos_val();
void park_transf();
void anti_park_transf();
void clark_transf();
void svpwm_calc();
#endif
