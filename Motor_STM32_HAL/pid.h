#ifndef __PID_H__
#define __PID_H__

#include "main.h"

#define _constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef struct
{
	float kp;
	float ki;
	float kd;
	float outRamp;
	float limit;
	float prev_error;
  float prev_output;
  float prev_integral;

  uint32_t lastTick;
	uint32_t nowTick;
	uint32_t usTickPeriod;
}pid_typedef;

/*declare extern variables*/


uint32_t pid_getTick(pid_typedef * pid);
void pid_timeTick(pid_typedef * pid);
void pid_init(pid_typedef * pid,
							float kp, float ki, float kd, 
							float ramp, float limit,
							uint32_t usTickPeriod);
void pid_setVal(pid_typedef * pid, float kp, float ki, float kd, float ramp);
void pid_setRamp(pid_typedef * pid, float ramp);
float pid_calc(pid_typedef * pid, float error);			
void pid_setKi(pid_typedef * pid, float ki);							
#endif