#ifndef __LAUNCH_TASK_H__
#define __LAUNCH_TASK_H__

#include "launch_module.h"
#include "chassis_module.h"

extern int8_t ball_temp[3];


typedef enum{
    SWITCH_POS_ACT_L = 0x00,
    SWITCH_POS_ACT_R = 0x01,
}SwitchPosActLR_e;


typedef enum{
    SWITCH_POS_ACT_B = 0x00,
    SWITCH_POS_ACT_F = 0x01,
}SwitchPosActFB_e;

typedef enum{
    FIRST_LINE = 0x00,
    SECOND_LINE = 0x01,
}LineState_e;

typedef struct
{
	BALLState_e current_state;
	uint8_t disable_rotation_flag;
	uint8_t protect_flag_2;
	
	uint8_t protect_flag;
  	uint8_t special_flag;
  	uint8_t confirm_flag;
	uint8_t ball_confirm;

    uint8_t current_ball;
    uint8_t last_ball;
  	uint8_t target_ball;
	
	uint16_t storm_speed;
	
	LineState_e current_line;
	int16_t line_compare;

	int16_t run_tick;
	SwitchPosActLR_e line_lr;
	SwitchPosActFB_e line_fb;
}LaunchInfo_t;


typedef enum
{
    BALL_STATE_IDLE  = 0x00,
    BALL_STATE_CHECK = 0x01,
    BALL_STATE_LINE_CHANGE_F,
    BALL_STATE_LINE_CHANGE_S,
    
    BALL_STATE_GET_BALL_POS_SPECIAL_F,
    BALL_STATE_GET_BALL_POS_SPECIAL_B,
    
    BALL_STATE_GET_BALL_POS_F,
    BALL_STATE_GET_BALL_POS_S,
    BALL_STATE_GET_BALL_POS_T,
    BALL_STATE_BALL_ACTION,
    BALL_STATE_ANGLE_CORRECT,
    
    BALL_STATE_LINE_CHANGE_SINGLE_1,
    BALL_STATE_LINE_CHANGE_SINGLE_2,
    BALL_STATE_LINE_CHANGE_SINGLE_3,
}BALLState_e;


#endif
