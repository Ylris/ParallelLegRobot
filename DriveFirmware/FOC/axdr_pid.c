#include "main.h"
#include "axdr_pid.h"

void Pid_Init(Pid_Controller_t *pid, float kp, float ki, float kd, float kis, float deltaT, float range)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->kis = kis;

  pid->deltaT = deltaT;
  pid->range = range;
  pid->i = 0.0f;
}

float Pid_Cal(Pid_Controller_t *pid, float target, float curr)
{
  float err = target - curr;
  float derr = (err - pid->prev) / pid->deltaT;

  pid->prev = err;
  pid->i += err * pid->deltaT;

  if (pid->i > 30.0f)
  {
      pid->i = 30.0f;
  }

  float output = pid->kp * err + pid->ki * pid->i + pid->kd * derr;

  if (output > pid->range)
  {
    pid->i += pid->kis * (pid->range - output) * pid->deltaT;
    return pid->range;
  }
  else if (output < -pid->range)
  {
    pid->i -= pid->kis * (pid->range + output) * pid->deltaT;
    return -pid->range;
  }
  return output;
}