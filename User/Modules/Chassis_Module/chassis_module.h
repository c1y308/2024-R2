#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "pid.h"
#include "can_trx.h"
#include <stdbool.h>
#include "user_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define TIMEFORACC 0.03f
#define M3508_MOTOR_SPEED_PID_KP 10000.0f
#define M3508_MOTOR_SPEED_PID_KI 10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f

#define M3508_MOTOR_SPEED_PID_POUT_LIMIT 8000
#define M3508_MOTOR_SPEED_PID_IOUT_LIMIT 1000
/**************************************************/
// 定义单轴的控制模式
typedef enum {
    AXIS_MODE_STOP = 0,   // 刹车/锁死
    AXIS_MODE_VEL  = 1,   // 速度控制
    AXIS_MODE_POS  = 2    // 位置控制
} AxisMode_e;


// 极其清晰的底盘指令包
typedef struct {
    uint32_t cmd_seq_id;  // 命令序列号，用于同步
    
    // 三轴控制模式
    AxisMode_e mode_x;
    AxisMode_e mode_y;
    AxisMode_e mode_z; 
    
	// 目标位置
	PosTarget_t pos_target;
    
    // 目标速度
	SpeedTarget_t speed_target;

    
} ChassisCmd_t;
/**************************************************/

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
}SpeedTarget_t;


typedef struct
{
	float vx;
	float vy;
	float wz;
}SpeedNow_t;


typedef struct
{
	uint32_t cmd_seq_id;
	uint32_t arrived_seq_id;
	// 三轴控制模式
    AxisMode_e mode_x;
    AxisMode_e mode_y;
    AxisMode_e mode_z; 

	uint32_t sac;

	uint8_t limit_vy_flag;
  	uint8_t red_single_flag;
	uint8_t blue_single_flag;
	uint8_t sssstar;
	uint8_t stop_crack;
	
	TolState_e tol_state;
	TaskType_e task_type;

	// ChassisMode_e chassis_mode;
	
	PosTarget_t pos_target;
	PosNow_t 	pos_now;
	PosError_t  pos_error;

	SpeedTarget_t speed_target;
	SpeedNow_t 	  speed_now;
	
	int16_t init_tick;

	float check_flag;
	float check_flag_2;
	float check_flag_3;
	
}Robotinfo_t;

extern int8_t sign_t;
extern osMessageQueueId_t chassis_cmd_queueHandle;
void chassis_task_entry(void *argument);
#endif

