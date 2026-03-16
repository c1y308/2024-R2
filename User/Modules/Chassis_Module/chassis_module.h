#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "pid.h"
#include "can_trx.h"
#include "pid.h"
#include <stdbool.h>
#include "user_config.h"
#define TIMEFORACC 0.03f
#define M3508_MOTOR_SPEED_PID_KP 10000.0f
#define M3508_MOTOR_SPEED_PID_KI 10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f

#define M3508_MOTOR_SPEED_PID_POUT_LIMIT 8000
#define M3508_MOTOR_SPEED_PID_IOUT_LIMIT 1000


typedef struct {
    uint32_t cmd_id;
    float tar_x;
    float tar_y;
    float tar_z;
    uint8_t mode;
} ChassisCmd_t;


typedef enum
{
    TASK_TYPE_CHASSIS_INIT = 0x01,
    TASK_TYPE_TEST         = 0x02,
    TASK_TYPE_GRAP_TEST    = 0x03,
    TASK_TYPE_GET_SEED     = 0x04,
    TASK_TYPE_PUT_SEED     = 0x05,
    TASK_TYPE_TRANSITION   = 0x06,
    TASK_TYPE_BALL         = 0x07,
    TASK_TYPE_REMAKE       = 0x08,
    TASK_TYPE_MANUAL       = 0x09,
    TASK_TYPE_SINGLE       = 0x0A,
    TASK_TYPE_SINGLE_BALL  = 0x0B,
} TaskType_e;


typedef enum
{
    CHASSIS_MODE_AUTO 	   = 0x20,
    CHASSIS_MODE_MANUAL    = 0x24,
    CHASSIS_HYBRID_XS      = 0x28,  // X 轴速度环
    CHASSIS_HYBRID_YS      = 0x32,  // Y 轴速度环
    CHASSIS_HYBRID_XSYS    = 0x36,  // X 轴速度环 + Y 轴速度环
    CHASSIS_MODE_MIX_SEED  = 0x40,
	CHASSIS_MODE_STOP      = 0x4C,
}ChassisState_e;


typedef enum
{
    CHASSIS_MODE_TOL_BIG   = 0x44,
    CHASSIS_MODE_TOL_SMALL = 0x48,
}TolState_e;

typedef struct
{
  	float pos_x;
	float pos_y;
 	float pos_z;
}PosTarget_t;


typedef struct {
	float pos_x;
	float pos_y;
	float angle;
	
	float pos_z;
	float posZ_speed;
}PosNow_t;


typedef struct
{
	float ErrorposX;
	float ErrorposY; 	
	float ErrorposZ; 
	float ErrorposX_temp;
	float ErrorposY_temp;
	float Error_posDes;
}PosError_t;


typedef struct
{
	float vx;
	float vy;
	float wz;

	float target_vx_direct;
	float target_vy_direct;
	float target_wz_direct;
}SpeedTarget_t;


typedef struct
{
	float vx;
	float vy;
	float wz;
}SpeedNow_t;



typedef struct
{
	uint32_t sac;

	uint8_t limit_vy_flag;
  	uint8_t red_single_flag;
	uint8_t blue_single_flag;
	uint8_t sssstar;
	uint8_t stop_crack;
	
	TolState_e tol_state;

	TaskType_e task_type;
	ChassisState_e chassis_state;
	
	PosTarget_t pos_target;
	PosNow_t 	pos_now;
	PosError_t  pos_error;

	SpeedTarget_t speed_target;
	SpeedNow_t 	  speed_now;
	
	int16_t init_tick;
	
	uint8_t chassis_arrive;

	float check_flag;
	float check_flag_2;
	float check_flag_3;
	
}Robotinfo_t;

extern int8_t sign_t;
extern Robotinfo_t robot_info;

void chassis_feedback_update(Robotinfo_t *robot_info);
void cal_chassis_speed_2_motor(Robotinfo_t *robot_info);
void chassis_pos_calc(Robotinfo_t *chassis_auto_build);
void input_tarspeed_chassis(Robotinfo_t *robot_info, float tarx, float tary, float tarz);
void input_tarpos_chassis(Robotinfo_t *robot_info, float tarpx, float tarpy,float tarpz);
void stop_chassis(Robotinfo_t *robot_info);
bool chassis_arrive_check(Robotinfo_t *robot_info);

void chassis_PID_init_outer(void);

extern float controller_vx;
extern float controller_vy;
extern float controller_wz;
#endif

