#ifndef _PID_H
#define _PID_H

typedef struct
{
  float kp;
  float ki;
  float kd;

  float kis;

  float deltaT;
  float range;

  float i;
  float prev;
} Pid_Controller_t;

void Pid_Init(Pid_Controller_t *pid, float kp, float ki, float kd, float kis, float deltaT, float range);

float Pid_Cal(Pid_Controller_t *pid, float target, float curr);
#endif // !
