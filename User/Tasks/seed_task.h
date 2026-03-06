#ifndef SEED_TASK_H
#define SEED_TASK_H
#include "main.h"
#include "chassis_module.h"
#include "dji_motor.h"
#include "can_trx.h"
#include "grap_module.h"
typedef enum 
{ 
	SEED_STATE_INIT = 1, 
	SEED_STATE_INIT_2, // 
	SEED_STATE_MOVE,   // 2
	SEED_STATE_GET,    // 3
	SEED_STATE_MOVE_2_PUT,     // 4
	SEED_STATE_PUT,    // 5
	SEED_STATE_CORRECT,// 6
	
	SEED_STATE_TRANSITION_F,// 7
	SEED_STATE_TRANSITION_S,// 8
	SEED_STATE_TRANSITION_T,// 9
	SEED_STATE_TRANSITION_4,// 10
} SeedState_e;


typedef struct
{
	SeedState_e seed_state;
	uint16_t run_tick;

	uint8_t  pos_index;

	uint8_t  move_count;
	uint8_t  grap_count;
	uint8_t  putm_count;
	uint8_t   put_count;

	uint8_t seed_ok;
}SeedIfo_t;


extern SeedIfo_t seed_ifo;

void seedtask_init(void);
void transition_task(void);
void plant_task(void);

void GP_Task_Single(void);
void Ball_Task_Single(void);

#endif

