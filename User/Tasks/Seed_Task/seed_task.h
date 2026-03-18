#ifndef SEED_TASK_H
#define SEED_TASK_H

#include "chassis_module.h"
#include "grap_module.h"
#include "dji_motor.h"
#include "can_trx.h"
#include "usart.h"
typedef enum 
{ 
	SEED_STATE_INIT = 1, 
	SEED_STATE_INIT_2,
	SEED_STATE_MOVE_2_GET,
	SEED_STATE_GET,
	SEED_STATE_MOVE_2_PUT,
	SEED_STATE_PUT,
	SEED_STATE_CORRECT,
} SeedState_e;


typedef enum {
    GRAP_COUNT_EVEN,  // 偶数次触发
    GRAP_COUNT_ODD,    // 奇数次触发
} GrapParity_e;


typedef enum{
	CHASSIS_PARITY_EVEN,  // 偶数次触发
	CHASSIS_PARITY_ODD,    // 奇数次触发
} ChassisParity_e;


typedef enum{
	PUT_MOVE_2_FRONT,  // 偶数次触发
	PUT_MOVE_2_BACK,    // 奇数次触发
} PutMovePos_e;


typedef enum{
	PUT_ODD,
	PUT_EVEN,
} PutParity_e;


typedef struct
{
	SeedState_e seed_state;

	uint8_t  	pos_index;

	uint8_t  	move_count;
	uint8_t  	grap_count;
	uint8_t  	putm_count;
	uint8_t  	put_count;
}SeedInfo_t;


extern SeedInfo_t seed_info;
extern osEventFlagsId_t motion_arrive_eventHandle;

void seedtask_init(SeedInfo_t *seed_info);
void plant_task(SeedInfo_t *seed_info);

void GP_Task_Single(void);
void Ball_Task_Single(void);

#endif

