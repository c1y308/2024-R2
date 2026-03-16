#include "seed_task.h"


SeedIfo_t seed_ifo;
float speed_move_seed = -2;


/* 判断爪子抓取次数是第奇数次还是第偶数次 */
static GrapParity_e get_grap_parity(SeedIfo_t *seed_ifo){
	if(seed_ifo->grap_count % 2 == 0)
		return GRAP_COUNT_EVEN;
	else
		return GRAP_COUNT_ODD;
}


/* 判断地盘去抓取苗是第奇数次还是第偶数次 */
static ChassisParity_e get_chassis_parity(SeedIfo_t *seed_ifo){
	if(seed_ifo->move_count % 2 == 0)
		return CHASSIS_PARITY_EVEN;
	else
		return CHASSIS_PARITY_ODD;
}


static PutMovePos_e get_put_move_pos(SeedIfo_t *seed_ifo){
	if(seed_ifo->putm_count % 2 == 0)
		return PUT_MOVE_2_FRONT;
	else
		return PUT_MOVE_2_BACK;
}


static PutParity_e get_put_parity(SeedIfo_t *seed_ifo){
	if(seed_ifo->put_count % 2 == 0)
		return PUT_EVEN;
	else
		return PUT_ODD;
}


void seedtask_init(SeedIfo_t *seed_ifo)
{
  	seed_ifo->seed_state = SEED_STATE_INIT;//transition_f
	seed_ifo->run_tick   = 0;
	seed_ifo->pos_index  = 0;
	seed_ifo->move_count = 0;
	seed_ifo->grap_count = 0;
	seed_ifo->put_count  = 0;
}

/* 调用地盘module 和 爪子module 的相关API 来完成种植任务，主要进行逻辑实现 */
void plant_task(SeedIfo_t *seed_ifo, grap_t *grap_ifo, Robotifo_t *robot_ifo, TransIfo_t *trans_ifo)
{
  	switch(seed_ifo->seed_state)
	{
		case SEED_STATE_INIT:
    	{
			grap_init(grap_ifo);
			
			robot_ifo->chassis_state = CHASSIS_MODE_MANUAL;
		  	if(seed_ifo->run_tick <= INIT_OUT_TICK)
			{
				input_tarspeed_chassis(robot_ifo, 0, 1, 0);
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

			if(chassis_arrive_check(robot_ifo) == true)
			{
				seed_ifo->seed_state = SEED_STATE_MOVE_2_GET;
			}
			else
			{
				input_tarpos_chassis(robot_ifo, sign_t * seed_pos_x[5 - seed_ifo->pos_index], SEED_FORWARD_DES, 0);
			}
			break;
		}
	  	case SEED_STATE_MOVE_2_GET:
    	{
			robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;

		    if( (get_chassis_parity(seed_ifo) == CHASSIS_PARITY_EVEN && chassis_arrive_check(robot_ifo) == true) ||   // 
		  	    (get_chassis_parity(seed_ifo) == CHASSIS_PARITY_ODD  && chassis_arrive_check(robot_ifo) == true && check_grap_arrive(grap_ifo) == true) ) 
			{                           
				seed_ifo->move_count++;
				seed_ifo->pos_index++;

				seed_ifo->run_tick = 0;

				robot_ifo->limit_vy_flag = 0;
				seed_ifo->seed_state = SEED_STATE_GET;
			}
			else
			{
				robot_ifo->limit_vy_flag = 1;

				input_tarpos_chassis(robot_ifo, sign_t * seed_pos_x[5 - seed_ifo->pos_index], crack_posY, 0);
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
					seed_ifo->seed_state = SEED_STATE_MOVE_2_GET;
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

				stop_chassis(robot_ifo);
				seed_ifo->run_tick++;
			}
			break;
		}
		
		case SEED_STATE_MOVE_2_PUT:
    	{
			robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;
		  	if((seed_ifo->run_tick >= 50 && chassis_arrive_check(robot_ifo) == true && get_put_move_pos(seed_ifo) == PUT_MOVE_2_FRONT) ||
		  	   (seed_ifo->run_tick >= 50 && chassis_arrive_check(robot_ifo) == true && get_put_move_pos(seed_ifo) == PUT_MOVE_2_BACK)) 
			{
				seed_ifo->putm_count++;
				seed_ifo->run_tick = 0;
				seed_ifo->seed_state = SEED_STATE_PUT;
			}
			else
			{
				if(get_put_move_pos(seed_ifo) == PUT_MOVE_2_FRONT)
					input_tarpos_chassis(robot_ifo, sign_t * put_pos_x[seed_ifo->pos_index - 2], put_pos_y[0], 0);
				else
					input_tarpos_chassis(robot_ifo, sign_t * put_pos_x[seed_ifo->pos_index - 2], put_pos_y[1], 0);

				preput_seed(grap_ifo);
				seed_ifo->run_tick++;
			}
			break;
		}

		case SEED_STATE_PUT:
    	{
			if( ( get_put_move_pos(seed_ifo) == PUT_MOVE_2_BACK && check_grap_arrive(grap_ifo) == true) ||  
			    ( get_put_move_pos(seed_ifo) == PUT_MOVE_2_FRONT && seed_ifo->run_tick >= (GRAP_TICK_OPEN + 10) ) )  
			{
				seed_ifo->put_count++;
				grap_ifo->grap_arrive = 0;
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_lift_right,  0);

				if(get_put_parity(seed_ifo) == PUT_ODD)
				{
					grap_ifo->put_lift_tick = 0;
					seed_ifo->seed_state = SEED_STATE_MOVE_2_PUT;
				}
				else
          			seed_ifo->seed_state = SEED_STATE_CORRECT;

				seed_ifo->run_tick = 0;
			}
			else
			{
				put_seed(grap_ifo);
        		stop_chassis(robot_ifo);
				seed_ifo->run_tick++;
			}
			break;
		}
		
		case SEED_STATE_CORRECT:
    	{
		  	if(seed_ifo->run_tick <= 120)
			{
				seed_ifo->run_tick++;
			  	robot_ifo->chassis_state = CHASSIS_MODE_MANUAL;
				robot_ifo->speed_target.target_vx_direct = -sign_t * 0.8;
				robot_ifo->speed_target.target_vy_direct =  0.4;
			}
			else
			{
			  	if(seed_ifo->pos_index >= 6)
				{
					seed_ifo->run_tick = 0;
					/* 此处可以修改为任务调度的形式 */
					start_transition_task(robot_ifo, trans_ifo);
					break;
				}
				else
					seed_ifo->seed_state = SEED_STATE_MOVE_2_GET;
			}
			break;
		}
		default:
			break;
	}
}

