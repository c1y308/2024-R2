#include "pid.h"
#include "main.h"
#include "stdio.h"


void PID_limit_maxout(PID_t *pid)   	
{                        				
	if (pid->Iout > pid->max_iout)       				
  {                      			
		pid->Iout = pid->max_iout;       	 				
  }                      				
  else if (pid->Iout < -pid->max_iout) 				
  {                      				
		pid->Iout = -pid->max_iout;        				
  };
  if (pid->out > pid->max_out)       				
  {                      			
		pid->out = pid->max_out;       	 				
  }                      				
  else if (pid->out < -pid->max_out) 				
  {                      				
		pid->out = -pid->max_out;        				
  }               	
}


void PID_init(PID_t *pid, PIDInitConfig_t config)
{
	if(pid == NULL)
	{
		return;
	}
	pid->mode = config.mode;

	pid->Kp = config.Kp;
	pid->Ki = config.Ki;
	pid->Kd = config.Kd;

	pid->max_out  = config.MaxOut;
	pid->max_iout = config.IntegralLimit;
	
	pid->Dbuf = 0;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}



void PID_clear(PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf = 0;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->value = pid->target = 0.0f;
}


float PID_calc(PID_t *pid, float ref, float target)
{
    if (pid == NULL)
    {
			return;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->target = (float)target;
    pid->value = (float)ref;
    pid->error[0] = target - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf;
        pid->out = (float)(pid->Pout + pid->Iout + pid->Dout);
			  pid->out_l = (float)pid->out;
			  LimitMax(pid);
    }

    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf = (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf;
        pid->out += (float)(pid->Pout + pid->Iout + pid->Dout);
			  pid->out_l = (float)pid->out;
			  LimitMax(pid);
    }
	return pid->out;
}

