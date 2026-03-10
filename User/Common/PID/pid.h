#ifndef __PID_H__
#define __PID_H__

#include "stdint.h"
#include "main.h"
#include "user_lib.h"
#include "can_trx.h"


#define M3508_DANGER_OUT 3500
#define M3508_GRAP_OUT   4000

#define M3508_MOTOR_POSITION_PID_POUT_LIMIT 1.2f
#define M3508_MOTOR_POSITION_PID_IOUT_LIMIT 0.08

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA = 1
};

typedef struct
{
    uint8_t mode;
    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 

    float target;
    float value;
	
    float out_l;
    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf;
    float error[3];
}PID_t;


typedef struct
{
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

    float MaxOut;
    float DeadBand;
    float IntegralLimit; 
} PIDInitConfig_t;

PID_t pid_DJI_outer[3];//分别为x，y，z速度

void  PID_limit_maxout(PID_t *pid);
void  PID_init(PID_t *pid, PIDInitConfig_t config);
void  PID_clear(PID_t *pid);
float PID_calc(PID_t *pid,float ref,float target);


#endif
