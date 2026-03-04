#include "pid.h"
#include "main.h"
#include "stdio.h"

//PID参数定义外环位置内环速度
//float DJI_pos_outer_para_cos[3]={0.0032, 0.0021, 0.22};//底盘移动位置环


//pid结构体定义
// PID_t pid_DJI_outer[3];//分别为x，y，z速度


// PID_t pid_DJI_outer_CAN2[2];//分别为云台转动
// PID_t pid_DJI_motor_CAN2[8];//分别为云台转动,抬升,与夹爪
/**********************************************************/

//输出限幅函数
void LimitMax(PID_t *pid)   	
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


//PID初始化需输入选择PID模式，PID参数的值，输出限幅值，积分限幅值
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


//清除PID
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


//PID计算,需输入PID结构体，当前值，目标值
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
    if (pid->mode == PID_POSITION)//位置式PID
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf;
        pid->out = (float)(pid->Pout + pid->Iout + pid->Dout);
			  pid->out_l = (float)pid->out;
			  LimitMax(pid);
    }

    else if (pid->mode == PID_DELTA)//增量式PID
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

