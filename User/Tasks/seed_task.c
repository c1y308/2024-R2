#include "seed_task.h"
#include "Code_Disc.h"
#include "usart.h"
#include "pid.h"
#include "main.h"
#include "launch_module.h"

SeedIfo_t seed_ifo;
float speed_move_seed = -2;


GrapParity_e get_grap_parity(SeedIfo_t *seed_ifo){
	if(seed_ifo->grap_count % 2 == 0)
		return GRAP_COUNT_EVEN;
	else
		return GRAP_COUNT_ODD;
}


<<<<<<< Updated upstream
ChassisParity_e get_chassis_parity(SeedIfo_t *seed_ifo){
	if(seed_ifo->move_count % 2 == 0)
		return CHASSIS_PARITY_EVEN;
	else
		return CHASSIS_PARITY_ODD;
}


void seedtask_init(SeedIfo_t *seed_ifo)
=======
void plant_task(Robotifo_t *robot_ifo, grap_t *grap_ifo)
>>>>>>> Stashed changes
{
  	seed_ifo->seed_state = SEED_STATE_INIT;//transition_f
	seed_ifo->run_tick   = 0;
	seed_ifo->pos_index  = 0;
	seed_ifo->move_count = 0;
	seed_ifo->grap_count = 0;
	seed_ifo->put_count  = 0;
}


void plant_task(SeedIfo_t *seed_ifo, grap_t *grap_ifo, Robotifo_t *robot_ifo)
{
  	switch(seed_ifo->seed_state)
	{
		case SEED_STATE_INIT:
    	{
			grap_init(grap_ifo);

			robot_ifo->chassis_state = CHASSIS_MODE_MANUAL;
<<<<<<< Updated upstream
		  	if(seed_ifo->run_tick <= INIT_OUT_TICK)
=======
		  	if(seed_ifo.run_tick <= INIT_OUT_TICK)
>>>>>>> Stashed changes
			{
				input_tarspeed_chassis(0, 1, 0);
			}
			else{
				seed_ifo->run_tick = 0;
				seed_ifo->seed_state = SEED_STATE_INIT_2;
			}
			break;
		}

		case SEED_STATE_INIT_2:
    	{
			robot_ifo->tol_state = CHASSIS_MODE_TOL_BIG;
			robot_ifo->chassis_state = CHASSIS_MODE_AUTO;

<<<<<<< Updated upstream
			if(chassis_arrive_check(robot_ifo) == true)
=======
			if(robot_ifo->chassis_arrive == 1)
>>>>>>> Stashed changes
			{
				seed_ifo->seed_state = SEED_STATE_MOVE;
			}
			else
			{
<<<<<<< Updated upstream
				input_tarpos_chassis(sign_t * seed_pos_x[5 - seed_ifo->pos_index], SEED_FORWARD_DES, 0);
=======
				input_tarpos_chassis(robot_ifo, sign_t * seed_pos_x[5 - seed_ifo.pos_index], SEED_FORWARD_DES, 0);
>>>>>>> Stashed changes
			}
			break;
		}
	  	case SEED_STATE_MOVE:
    	{
			robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;

<<<<<<< Updated upstream
		    if( (get_chassis_parity(seed_ifo) == CHASSIS_PARITY_EVEN && chassis_arrive_check(robot_ifo) == true) ||   // 
		  	    (get_chassis_parity(seed_ifo) == CHASSIS_PARITY_ODD  && chassis_arrive_check(robot_ifo) == true && check_grap_arrive(grap_ifo) == true) ) 
=======
		    if((seed_ifo.move_count % 2 == 0 && robot_ifo->chassis_arrive == 1) ||   // 
		  	   (seed_ifo.move_count % 2 != 0 && robot_ifo->chassis_arrive == 1 && grap_ifo->grap_arrive == 1) ) 
>>>>>>> Stashed changes
			{                           
				seed_ifo->move_count++;
				seed_ifo->pos_index++;

				seed_ifo->run_tick = 0;

				robot_ifo->limit_vy_flag = 0;
<<<<<<< Updated upstream
				seed_ifo->seed_state = SEED_STATE_GET;
=======
				seed_ifo.seed_state = SEED_STATE_GET;
>>>>>>> Stashed changes
			}
			else
			{
				robot_ifo->limit_vy_flag = 1;

<<<<<<< Updated upstream
				input_tarpos_chassis(sign_t * seed_pos_x[5 - seed_ifo->pos_index], crack_posY, 0);
=======
				input_tarpos_chassis(sign_t * seed_pos_x[5 - seed_ifo.pos_index], crack_posY, 0);
>>>>>>> Stashed changes
				robot_ifo->chassis_state = CHASSIS_MODE_MIX_SEED;
				robot_ifo->speed_target.target_vy_direct = -1.6;

				seed_ifo->run_tick++;
			}
			break;
		}
		case SEED_STATE_GET:
    	{
			static uint8_t reset_ops9_z = 1;
			static uint8_t reset_ops9_y = 1;

		  	if(	( ( get_grap_parity(&seed_ifo) == GRAP_COUNT_EVEN ) && ( grap_ifo->ready_2_move == 1 ) ) ||
			 	( ( get_grap_parity(&seed_ifo) == GRAP_COUNT_ODD )  && ( check_grap_arrive(grap_ifo) == false  ) )  )
			{
				seed_ifo->grap_count++;
				reset_ops9_z = 1;
				reset_ops9_y = 1;

				seed_ifo->run_tick = 0;

				if(get_grap_parity(seed_ifo) == GRAP_COUNT_ODD)
					seed_ifo->seed_state = SEED_STATE_MOVE;
				else
				{
					grap_ifo->grap_arrive = 0;
					dji_motor_setref(motor_lift_left,  0);
					dji_motor_setref(motor_lift_right, 0);
					seed_ifo->seed_state = SEED_STATE_MOVE_2_PUT;
				}
			}
			else
			{
				if(get_grap_parity(seed_ifo) == GRAP_COUNT_EVEN)
					grap_seed_store(grap_ifo);  // 切换爪子系统的状态：抓取地面上的苗，存储在车上
				else
				{
					if(check_grap_arrive(grap_ifo) == true)
						grap_seed(grap_ifo);   // 切换爪子系统的状态：抓取地上的苗，但不存储
				}

				if(reset_ops9_z == 1 && get_grap_parity(seed_ifo) == GRAP_COUNT_EVEN)
				{
					CD_SETZ(&huart3, 0);
					reset_ops9_z = 0;
				}
				if(reset_ops9_y == 1 && get_grap_parity(seed_ifo) == GRAP_COUNT_ODD)
				{
					CD_SETX(&huart3, -250);
					reset_ops9_y = 0;
				}
				stop_chassis();
				seed_ifo->run_tick++;
			}
			break;
		}
		
		case SEED_STATE_MOVE_2_PUT:
    	{
			robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;
<<<<<<< Updated upstream
		  	if((seed_ifo->run_tick >= 50 && chassis_arrive_check(robot_ifo) == true && seed_ifo->putm_count % 2 == 0) ||
		  	   (seed_ifo->run_tick >= 50 && chassis_arrive_check(robot_ifo) == true && seed_ifo->putm_count % 2 == 1)) 
=======
		  	if((seed_ifo.run_tick >= 50 && robot_ifo->chassis_arrive == 1 && seed_ifo.putm_count % 2 == 0) ||
		  	   (seed_ifo.run_tick >= 50 && robot_ifo->chassis_arrive == 1 && seed_ifo.putm_count % 2 == 1)) 
>>>>>>> Stashed changes
			{
				seed_ifo->putm_count++;
				seed_ifo->run_tick = 0;
				seed_ifo->seed_state = SEED_STATE_PUT;
			}
			else
			{
				grap_ifo->grap_state = GRAP_STATE_PRE_PUT;
				seed_ifo->run_tick++;
			}
			break;
		}

		case SEED_STATE_PUT:
    	{
			if( ( seed_ifo->putm_count % 2 == 1 && check_grap_arrive(grap_ifo) == true) ||  
			    ( seed_ifo->putm_count % 2 == 0 && seed_ifo->run_tick >= (GRAP_TICK_OPEN + 10) ) )  
			{
				seed_ifo->put_count++;
				grap_ifo->grap_arrive = 0;
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_lift_right,  0);

				if(seed_ifo->put_count % 2 != 0)
				{
					grap_ifo->put_lift_tick = 0;
					seed_ifo->seed_state = SEED_STATE_MOVE_2_PUT;
				}
				else
				{
          			seed_ifo->seed_state = SEED_STATE_CORRECT;
				}
				seed_ifo->run_tick = 0;
			}
			else
			{
				put_seed(grap_ifo);
        		stop_chassis();
				seed_ifo->run_tick++;
			}
			break;
		}
		
		case SEED_STATE_CORRECT:
    	{
		  	if(seed_ifo->run_tick <= 120)
			{
<<<<<<< Updated upstream
				seed_ifo->run_tick++;
=======
				seed_ifo.run_tick++;
>>>>>>> Stashed changes
			  	robot_ifo->chassis_state = CHASSIS_MODE_MANUAL;
				robot_ifo->speed_target.target_vx_direct = -sign_t * 0.8;
				robot_ifo->speed_target.target_vy_direct =  0.4;
			}
			else
			{
			  	if(seed_ifo->pos_index >= 6)
				{
<<<<<<< Updated upstream
					seed_ifo->run_tick = 0;
					seed_ifo->seed_state = SEED_STATE_TRANSITION_F;
=======
					seed_ifo.run_tick = 0;
					seed_ifo.seed_state = SEED_STATE_TRANSITION_F;
>>>>>>> Stashed changes
					robot_ifo->task_type = TASK_TYPE_TRANSITION;
					break;
				}
				else
					seed_ifo->seed_state = SEED_STATE_MOVE;
			}
			break;
		}
		default:
			break;
	}
}


void transition_task(SeedIfo_t *seed_ifo, Robotifo_t *robot_ifo)
{
  switch(seed_ifo->seed_state)
	{
		static uint8_t clear_PID_flag = 1;
	  	case SEED_STATE_TRANSITION_F:{
<<<<<<< Updated upstream
			if( (chassis_arrive_check(robot_ifo) == true && seed_ifo->run_tick >= 50) || seed_ifo->run_tick >= 270)
=======
			if( (robot_ifo->chassis_arrive == 1 && seed_ifo.run_tick >= 50) || seed_ifo.run_tick >= 270)
>>>>>>> Stashed changes
			{
				seed_ifo->run_tick = 0;
			  	seed_ifo->seed_state = SEED_STATE_TRANSITION_S;
			}
			else
			{
				if(clear_PID_flag == 1)
				{
					for(int i = 0; i<2; i++)
					{
						PID_Init(&pid_DJI_outer[i], PID_POSITION, DJI_pos_outer_para, RUN_S, M3508_MOTOR_POSITION_PID_IOUT_LIMIT);
					} 
					clear_PID_flag = 0;
				}
				seed_ifo->run_tick++;
				input_tarpos_chassis(sign_t * -790, 1800, 0);
			}
			break;
		}
		case SEED_STATE_TRANSITION_S:{  
		  if(seed_ifo->run_tick >= TRANS_TICK_S)
			{
				seed_ifo->run_tick = 0;
			  	seed_ifo->seed_state = SEED_STATE_TRANSITION_T;
			}
			else
			{
				if(robot_ifo->blue_single_flag == 0 && robot_ifo->red_single_flag == 0)
				  	grap_ifo.grap_state = GRAP_STATE_PUT_ROTATE; 
				else
				  	grap_ifo.grap_state = GRAP_STATE_GRAP;

<<<<<<< Updated upstream
				seed_ifo->run_tick++;
=======
				seed_ifo.run_tick++;
>>>>>>> Stashed changes
        		robot_ifo->chassis_state = CHASSIS_HYBRID_YS;
				robot_ifo->speed_target.target_vy_direct = 1.9;
				robot_ifo->pos_target.pos_z = 0;
			}
			break;
		}

		case SEED_STATE_TRANSITION_T:{
			if(seed_ifo->run_tick >= CRACK_SECOND_TICK)
			{
				CD_SETY(&huart3, 0);
				seed_ifo->seed_state = SEED_STATE_TRANSITION_4;
			  	seed_ifo->run_tick = 0;
			}
			else
			{
				for(uint8_t i = 0; i<8; i++)
				{
					DJI_motor_CAN2[i].target_speed = 0;
				  PID_clear(&pid_DJI_motor_CAN2[i]);
				}
				magnet_control();
				seed_ifo->run_tick++;
				Input_TarSpeed_Chassis(-1 * sign_t  *CRACK_SPEED_SECOND, 0, 0);
			}
			break;
		}
		case SEED_STATE_TRANSITION_4:{//
			static int32_t lv_100_total = 0;
			if(seed_ifo->run_tick < 100)
			  	lv_100_total += real_lv100;

			seed_ifo->run_tick++;

			if(seed_ifo->run_tick >= 100)
			{
				CD_SETX(&huart3, -(float) (lv_100_total / 100));
				CD_SETZ(&huart3, 0);
				input_tarpos_chassis(robot_ifo->pos_now.pos_x, real_lv100, robot_ifo->pos_now.pos_z);
				
				if(robot_ifo->blue_single_flag == 1 || robot_ifo->red_single_flag == 1)
				  	robot_ifo->task_type = TASK_TYPE_SINGLE_BALL;
				else
					robot_ifo->task_type = TASK_TYPE_BALL;
			}
			stop_chassis();
			break;
		}
		default:break;
	}
}

