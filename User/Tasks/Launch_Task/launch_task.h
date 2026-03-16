#ifndef __LAUNCH_TASK_H__
#define __LAUNCH_TASK_H__

#include "launch_module.h"
#include "chassis_module.h"
#include <stdbool.h>

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


typedef enum{
    LAUNCH_CONFIRM_WAIT = 0x00,
    LAUNCH_CONFIRM_READY = 0x01,
}LaunchConfirm_e;


typedef struct
{
	BALLState_e current_state;
	bool disable_rotation_flag;
	
  	bool special_switch;
	LaunchConfirm_e confirm_2_launch;

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
    BALL_STATE_START_LAUNCH = 0x01,
    BALL_STATE_LINE_CHANGE_F,
    BALL_STATE_LINE_CHANGE_S,
    
    BALL_STATE_GET_BALL_POS_SPECIAL_F,
    BALL_STATE_GET_BALL_POS_SPECIAL_B,
    
    BALL_STATE_MOVE_TO_BALL_POS,
    BALL_STATE_GET_BALL_ACTION,
    BALL_STATE_READY_TO_LAUNCH,
    BALL_STATE_LAUNCH_ACTION,
    BALL_STATE_ANGLE_CORRECT,
    
    BALL_STATE_LINE_CHANGE_SINGLE_1,
    BALL_STATE_LINE_CHANGE_SINGLE_2,
    BALL_STATE_LINE_CHANGE_SINGLE_3,
}BALLState_e;

void launch_task(LaunchInfo_t *launch_info, Robotinfo_t *robot_info);

#endif
