#ifndef __TRANS_TASK_H__
#define __TRANS_TASK_H__

#include "chassis_module.h"
#include "grap_module.h"
#include "usart.h"

typedef enum{
    TRANS_STATE_IDLE,
	TRANS_STATE_F,
	TRANS_STATE_S,
	TRANS_STATE_T,
	TRANS_STATE_4,
}trans_state_e;

typedef struct{
    trans_state_e trans_state;
    uint32_t run_tick;
}TransIfo_t;


extern TransIfo_t trans_ifo;


void init_transition_task(Robotinfo_t *robot_ifo, TransIfo_t *trans_ifo);
void start_transition_task(Robotinfo_t *robot_ifo, TransIfo_t *trans_ifo);
#endif
