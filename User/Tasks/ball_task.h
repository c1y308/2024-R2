#ifndef BALL_TASK_H
#define BALL_TASK_H
#include "main.h"
#include "chassis_task.h"
#include "dji_motor.h"
#define BALL_LEFT 1
#define BALL_RIGHT 2
#define BALL_FORWARD 1
#define BALL_BACK 2

typedef enum
{
    BALL_STATE_IDLE = 0x00,
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

typedef struct
{
	BALLState_e current_state;
	uint8_t disable_rotation_flag;
	uint8_t protect_flag_2;
	
	uint8_t protect_flag;
  	uint8_t special_flag;
  	uint8_t confirm_flag;
	uint8_t ball_confirm;
	uint8_t now_ball;
  	uint8_t target_ball;
	
	uint16_t storm_speed;
	
	uint8_t line_now;
	
	int16_t line_compare;

	int16_t bt_check;
	uint8_t line_lr;
	uint8_t line_fb;
}BallInfo_t;

extern int8_t ball_temp[3];
void ball_task(void);


void ball_init(void);


void magnet_control(void);

#endif
