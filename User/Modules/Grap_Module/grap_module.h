#ifndef GRAP_TASK_H
#define GRAP_TASK_H
#include "chassis_module.h"
#include "dji_motor.h"
#include "user_config.h"
#include <stdbool.h>
#include "cmsis_os2.h"

#define EVENT_GRAP_ARRIVE        (1 << 1)  // 爪子到达目标位置
#define EVENT_GRAP_HALF_READY    (1 << 2)  // 爪子准备好放置


typedef enum {
    CMD_NONE = 0,
    CMD_SEED_STORE,     // 抓取并存储
    CMD_SEED_GRAP,      // 仅抓取
    CMD_SEED_PREPUT,    // 预放苗
    CMD_SEED_PUT        // 放苗
} GrapCommand_e;


typedef enum{
	GARP_STATE_IDLE = 0,
	GARP_STATE_INIT = 1,

	GRAP_STATE_START_STORAGE,     // 把场地上的苗抓取并存储
	GRAP_STATE_GRAP,
	GRAP_STATE_PRELIFT,
	GRAP_STATE_ROTATE,
	GRAP_STATE_ROTATE_2,
	GRAP_STATE_OPEN,
	GRAP_STATE_BACK2WAIT,

	GARP_STATE_LIFT,  // 抓取场地上的苗之后抬升

	GRAP_STATE_PRE_PUT,

	GRAP_STATE_PUT_ROTATE,
	GRAP_STATE_PUT_ROTATEBACK,

    GRAP_STATE_WAIT_AND_PUT,
}GrapState_e;


typedef struct
{
    GrapState_e grap_last_state;
	GrapState_e grap_state;
	uint16_t grap_tick;

	uint8_t start_storage;
	uint8_t start_storage_out;
	
	uint8_t grap_arrive;
	uint8_t ready_2_move;  // 考虑以信号量进行替代
    
	uint8_t test_semophare;
	
	uint16_t get_lift_tick;
	
	uint16_t put_lift_tick; 
	
	float ErrorAngle[2];
	float ErrorLift[2];
}grap_t;


extern DjiMotorHandle_t *motor_yaw_left, *motor_yaw_right, *motor_lift_left, *motor_lift_right, *motor_grap_left, *motor_grap_right;


extern grap_t grap_ifo;
extern osMessageQueueId_t grap_cmd_queueHandle;

void grap_seed_store();
void grap_seed();
void preput_seed();
void put_seed();
// grap_init


#endif
