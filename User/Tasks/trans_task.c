#include "trans_task.h"
TransIfo_t trans_ifo;

void init_transition_task(Robotifo_t *robot_ifo, TransIfo_t *trans_ifo){
     trans_ifo->trans_state = TRANS_STATE_IDLE;
     trans_ifo->run_tick = 0;
}


void start_transition_task(Robotifo_t *robot_ifo, TransIfo_t *trans_ifo)
{
    robot_ifo->task_type = TASK_TYPE_TRANSITION;
    trans_ifo->trans_state = TRANS_STATE_F;
    trans_ifo->run_tick = 0;
}


void transition_task(Robotifo_t *robot_ifo, TransIfo_t *trans_ifo)
{
  switch(trans_ifo->trans_state)
	{
		static uint8_t clear_PID_flag = 1;
	  	case TRANS_STATE_F:{
			if( (chassis_arrive_check(robot_ifo) == true && trans_ifo->run_tick >= 50) || trans_ifo->run_tick >= 270)
			{
				trans_ifo->run_tick = 0;
			  	trans_ifo->trans_state = TRANS_STATE_S;
			}
			else
			{
				if(clear_PID_flag == 1)
				{
                    set_grap_motor_zero_speed();
					clear_PID_flag = 0;
				}
				trans_ifo->run_tick++;
				input_tarpos_chassis(robot_ifo, sign_t * -790, 1800, 0);
			}
			break;
		}
		case TRANS_STATE_S:{  
		  if(trans_ifo->run_tick >= TRANS_TICK_S)
			{
				trans_ifo->run_tick = 0;
			  	trans_ifo->trans_state = TRANS_STATE_T;
			}
			else
			{
				if(robot_ifo->blue_single_flag == 0 && robot_ifo->red_single_flag == 0)
				  	grap_ifo.grap_state = GRAP_STATE_PUT_ROTATE; 
				else
				  	grap_ifo.grap_state = GRAP_STATE_GRAP;

				trans_ifo->run_tick++;
        		robot_ifo->chassis_state = CHASSIS_HYBRID_YS;
				robot_ifo->speed_target.target_vy_direct = 1.9;
				robot_ifo->pos_target.pos_z = 0;
			}
			break;
		}

		case TRANS_STATE_T:{
			if(trans_ifo->run_tick >= CRACK_SECOND_TICK)
			{
				CD_SETY(&huart3, 0);
				trans_ifo->trans_state = TRANS_STATE_4;
			  	trans_ifo->run_tick = 0;
			}
			else
			{
				// for(uint8_t i = 0; i < 8; i++)
				// {
				// 	DJI_motor_CAN2[i].target_speed = 0;
				//     PID_clear(&pid_DJI_motor_CAN2[i]);
				// }
				magnet_control();
				trans_ifo->run_tick++;
				input_tarspeed_chassis(robot_ifo, -1 * sign_t * CRACK_SPEED_SECOND, 0, 0);
			}
			break;
		}
		case TRANS_STATE_4:{//
			static int32_t lv_100_total = 0;
			if(trans_ifo->run_tick < 100)
			  	lv_100_total += real_lv100;

			trans_ifo->run_tick++;

			if(trans_ifo->run_tick >= 100)
			{
				CD_SETX(&huart3, -(float) (lv_100_total / 100));
				CD_SETZ(&huart3, 0);
				input_tarpos_chassis(robot_ifo, robot_ifo->pos_now.pos_x, real_lv100, robot_ifo->pos_now.pos_z);
				
				if(robot_ifo->blue_single_flag == 1 || robot_ifo->red_single_flag == 1)
				  	robot_ifo->task_type = TASK_TYPE_SINGLE_BALL;
				else
					robot_ifo->task_type = TASK_TYPE_BALL;
			}
			stop_chassis(robot_ifo);
			break;
		}
		default:break;
	}
}