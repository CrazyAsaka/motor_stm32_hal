#include "pid.h"

/*variable definition*/
// pid_typedef pid_rMotorVel;
// pid_typedef pid_rMotorPos;
// pid_typedef pid_lMotorVel;
// pid_typedef pid_lMotorPos;


inline uint32_t pid_getTick(pid_typedef * pid)
{
	return pid->nowTick;
}

inline void pid_timeTick(pid_typedef * pid)
{
	pid->nowTick++;
}

void pid_init(pid_typedef * pid, float kp, float ki, float kd, float ramp, float limit, uint32_t usTickPeriod)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->outRamp = ramp;
	pid->limit = limit;
	pid->prev_error = 0.0f;
  pid->prev_output = 0.0f;
  pid->prev_integral = 0.0f;
	
	pid->nowTick = 0;
	pid->lastTick = pid_getTick(pid);
	pid->usTickPeriod = usTickPeriod;
}

float pid_calc(pid_typedef * pid, float error)
{
	uint32_t nowTick = pid_getTick(pid);
	float ts = (nowTick - pid->lastTick) * pid->usTickPeriod * 1e-6f;
	if (ts <= 0 || ts > 0.5f) ts = pid->usTickPeriod*1e-6f;		//这句话不知道有没有用
	
	//	P
  float proportional = pid->kp * error;
  //  I
  float integral = 
		pid->prev_integral + pid->ki * ts * 0.5f * (error + pid->prev_error);
  integral = _constrain(integral, -pid->limit, pid->limit);
  //	D
  float derivative = pid->kd * (error - pid->prev_error) / ts;
  // plus P, I, D
  float output = proportional + integral + derivative;
  output = _constrain(output, -pid->limit, pid->limit);
	
	if (pid->outRamp > 0) {
    // Limiting the rate of change (acceleration) of the PID controller
    float output_rate = (output - pid->prev_output) / ts;
    if (output_rate > pid->outRamp)
      output = pid->prev_output + pid->outRamp * ts;
    else if (output_rate < -pid->outRamp)
      output = pid->prev_output - pid->outRamp * ts;
  }

  pid->prev_integral = integral;
  pid->prev_output = output;
  pid->prev_error = error;
  pid->lastTick = nowTick;
	
  return output;
}

void pid_setVal(pid_typedef * pid, float kp, float ki, float kd, float ramp)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->outRamp = ramp;
}

void pid_setRamp(pid_typedef * pid, float ramp)
{
	pid->outRamp = ramp;
}

void pid_setKi(pid_typedef * pid, float ki)
{
	pid->ki	= ki;
}
