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
	CMD_SEED_INIT,
    CMD_SEED_GRAP_2_STORE,       // 抓取后进行存储
	CMD_SEED_GRAP_2_DIRECT_PUT,  // 抓取后进行种植
	CMD_SEED_PUT_RESTORED_SEED,  // 种植存储的苗
} GrapCommand_e;


typedef enum{
	GRAP_STATE_IDLE = 0,
	GRAP_STATE_INIT = 1,

	GRAP_STATE_GRAP_2_STORE,
	GRAP_STATE_GRAP_2_DIRECT_PUT,

	GRAP_STATE_RESTORE_ROTATE_FRONT,
	GRAP_STATE_RESTORE_ROTATE_BACK,

	GRAP_STATE_PUT_ROTATE_FRONT,
	GRAP_STATE_PUT_ROTATE_BACK,

    GRAP_STATE_PUT_CORRECT,
}GrapState_e;


typedef struct
{
	GrapState_e grap_state;

	uint8_t grap_arrive;
    
	float ErrorAngle[2];
	float ErrorLift[2];
}grap_t;


extern DjiMotorHandle_t *motor_yaw_left, *motor_yaw_right, *motor_lift_left, *motor_lift_right, *motor_grap_left, *motor_grap_right;


extern grap_t grap_ifo;
extern osMessageQueueId_t grap_cmd_queueHandle;


void grap_motor_init(void);
void grap_task(grap_t *grap_ifo);

void grap_init(void);
void grap_seed_2_store(void);
void grap_seed_2_put(void);
void put_stored_seed(void);

// grap_init


#endif
