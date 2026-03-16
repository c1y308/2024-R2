#include "remake_task.h"

void Remake_Task(Robotifo_t *robot_ifo)
{
  	static uint16_t remake_tick = 0;
	static float temp_x, temp_y;
	
	remake_tick++;
	
	if(remake_tick <= REMAKE_TX)
		temp_x = REMAKE_SPEEDX * sign_t;
	else
		temp_x = 0;
	if(remake_tick <= REMAKE_TY)
		temp_y = REMAKE_SPEEDY;
	else
		temp_y = 0;
	if(250 + REMAKE_TY > remake_tick && remake_tick > REMAKE_TY)
	{
		magnet_control();
	  temp_x = -0.4 * sign_t;
		temp_y = 0;
	}

	input_tarspeed_chassis(robot_ifo, temp_x, temp_y, 0);

	if(REMAKE_TY + 320 > remake_tick && remake_tick > 300 + REMAKE_TY)
	{
		CD_SETX(&huart3, -(float)real_lv100);
	}
	if(REMAKE_TY+320 <= remake_tick)
	{
	  CD_SETY(&huart3, 0);

		Input_TarPos_Chassis(0, robot_ifo->pos_target.pos_y, 0);

		input_tarpos_chassis(robot_ifo, 0, robot_ifo->pos_target.pos_y, 0);
		remake_tick = 0;
		robot_ifo->task_type = TASK_TYPE_BALL;
	}
}