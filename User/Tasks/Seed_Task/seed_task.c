#include "seed_task.h"

SeedInfo_t seed_info;
float speed_move_seed = -2;


/* 判断爪子抓取苗次数是第奇数次还是第偶数次 */
static GrapParity_e get_grap_parity(SeedInfo_t *seed_info){
	if(seed_info->grap_count % 2 == 0)
		return GRAP_COUNT_EVEN;
	else
		return GRAP_COUNT_ODD;
}


/* 判断地盘抓取苗是奇数次还是偶数次 */
static ChassisParity_e get_chassis_parity(SeedInfo_t *seed_info){
	if(seed_info->move_count % 2 == 0)
		return CHASSIS_PARITY_EVEN;
	else
		return CHASSIS_PARITY_ODD;
}


/*  判断当前是第几次种植, 是第奇数次还是第偶数次 */
static PutMovePos_e get_put_move_pos(SeedInfo_t *seed_info){
	if(seed_info->putm_count % 2 == 0)
		return PUT_MOVE_2_FRONT;
	else
		return PUT_MOVE_2_BACK;
}


static PutParity_e get_put_parity(SeedInfo_t *seed_info){
	if(seed_info->put_count % 2 == 0)
		return PUT_EVEN;
	else
		return PUT_ODD;
}


void seedtask_init(SeedInfo_t *seed_info)
{
  	seed_info->seed_state = SEED_STATE_INIT;//transition_f
	seed_info->pos_index  = 0;
	seed_info->move_count = 0;
	seed_info->grap_count = 0;
	seed_info->put_count  = 0;
}


/* 调用地盘module 和 爪子module 的相关API 来完成种植任务，主要进行逻辑实现 */
void plant_task(SeedInfo_t *seed_info)
{
  	switch(seed_info->seed_state)
	{
		case SEED_STATE_INIT:
    	{
			// 需要让爪子完成初始化状态

			/* 通过消息队列给地盘发送速度控制信息 */
			ChassisCmd_t chassis_cmd;
			chassis_cmd.speed_target.target_vx = 0;
			chassis_cmd.speed_target.target_vy = 1;
			chassis_cmd.speed_target.target_vz = 0;
			chassis_cmd.mode_x = AXIS_MODE_VEL;
			chassis_cmd.mode_y = AXIS_MODE_VEL;
			chassis_cmd.mode_z = AXIS_MODE_VEL;
			osMessageQueuePut(chassis_cmd_queueHandle, &chassis_cmd, 0, 0);

			osDelay(INIT_OUT_TICK);

			seed_info->seed_state = SEED_STATE_INIT_2;

			break;
		}

		case SEED_STATE_INIT_2:
    	{
			/* 通过消息队列给地盘发送速度控制信息 */
			ChassisCmd_t chassis_cmd;
			chassis_cmd.pos_target.pos_x = sign_t * seed_pos_x[5 - seed_info->pos_index];
			chassis_cmd.pos_target.pos_y = SEED_FORWARD_DES;
			chassis_cmd.pos_target.pos_z = 0;
			
			chassis_cmd.mode_x = AXIS_MODE_POS;
			chassis_cmd.mode_y = AXIS_MODE_POS;
			chassis_cmd.mode_z = AXIS_MODE_POS;
			osMessageQueuePut(chassis_cmd_queueHandle, &chassis_cmd, 0, 0);

			osEventFlagsWait(motion_arrive_eventHandle, EVENT_CHASSIS_ARRIVE, osFlagsWaitAny, osWaitForever);

			/* 3. 代码能走到这里，说明底盘100%已经到位了，直接切状态 */
			seed_info->seed_state = SEED_STATE_MOVE_2_GET;

			break;
		}
	  	case SEED_STATE_MOVE_2_GET:
    	{
			// robot_info->limit_vy_flag = 1;
			ChassisCmd_t chassis_cmd;
			chassis_cmd.pos_target.pos_x = sign_t * seed_pos_x[5 - seed_info->pos_index];
			chassis_cmd.pos_target.pos_y = crack_posY;
			chassis_cmd.pos_target.pos_z = 0;
			
			chassis_cmd.speed_target.target_vy = -1.6;

			chassis_cmd.mode_x = AXIS_MODE_POS;
			chassis_cmd.mode_y = AXIS_MODE_VEL;
			chassis_cmd.mode_z = AXIS_MODE_POS;
			osMessageQueuePut(chassis_cmd_queueHandle, &chassis_cmd, 0, 0);

			if(get_chassis_parity(seed_info) == CHASSIS_PARITY_EVEN){
				osEventFlagsWait(motion_arrive_eventHandle, EVENT_CHASSIS_ARRIVE, osFlagsWaitAll, osWaitForever);
			}else if(get_chassis_parity(seed_info) == CHASSIS_PARITY_ODD){
				uint32_t wait_flags = EVENT_CHASSIS_ARRIVE | EVENT_GRAP_ARRIVE;
				osEventFlagsWait(motion_arrive_eventHandle, wait_flags, osFlagsWaitAll, osWaitForever);
			}

			/* 硬件动作都做完了，更新逻辑状态 */
			seed_info->move_count++;
			seed_info->pos_index++;
			seed_info->seed_state = SEED_STATE_GET;

			break;
		}
		case SEED_STATE_GET:
    	{
			/* 可以添加重置里程计的操作 */


			stop_chassis();

			/* 偶数次触发，抓取地面苗并存储在车上 */
			if(get_grap_parity(seed_info) == GRAP_COUNT_EVEN){
				grap_seed_store();
				osEventFlagsWait(motion_arrive_eventHandle, EVENT_GRAP_HALF_READY, osFlagsWaitAny, osWaitForever);
			}
			/* 奇数次触发，抓取地上的苗 */
			else{
				osEventFlagsWait(motion_arrive_eventHandle, EVENT_GRAP_ARRIVE, osFlagsWaitAny, osWaitForever);
				grap_seed();
				osEventFlagsWait(motion_arrive_eventHandle, EVENT_GRAP_ARRIVE, osFlagsWaitAny, osWaitForever);
			}

			/* 动作均已完成，被唤醒后处理后续逻辑 */
			seed_info->grap_count++;
			if(get_grap_parity(seed_info) == GRAP_COUNT_EVEN) {
				dji_motor_setref(motor_lift_left,  0);
				dji_motor_setref(motor_lift_right, 0);
				seed_info->seed_state = SEED_STATE_MOVE_2_PUT;
			} else {
				seed_info->seed_state = SEED_STATE_MOVE_2_GET;
			}
			break;
		}
		
		case SEED_STATE_MOVE_2_PUT:
    	{
			ChassisCmd_t chassis_cmd;
			chassis_cmd.pos_target.pos_x = sign_t * put_pos_x[seed_info->pos_index - 2];
			chassis_cmd.pos_target.pos_y = put_pos_y[0];
			chassis_cmd.pos_target.pos_z = 0;

			chassis_cmd.mode_x = AXIS_MODE_POS;
			chassis_cmd.mode_y = AXIS_MODE_POS;
			chassis_cmd.mode_z = AXIS_MODE_POS;

			if(get_put_move_pos(seed_info) == PUT_MOVE_2_FRONT)
				chassis_cmd.pos_target.pos_y = put_pos_y[0];
			else
				chassis_cmd.pos_target.pos_y = put_pos_y[1];
			
			osMessageQueuePut(chassis_cmd_queueHandle, &chassis_cmd, 0, 0);

			preput_seed();

			osEventFlagsWait(motion_arrive_eventHandle, EVENT_CHASSIS_ARRIVE, osFlagsWaitAll, osWaitForever);

			seed_info->putm_count++;
			seed_info->seed_state = SEED_STATE_PUT;
			break;
		}

		case SEED_STATE_PUT:
    	{
			put_seed();
        	stop_chassis();
			
			if(get_put_move_pos(seed_info) == PUT_MOVE_2_FRONT){
				osDelay(GRAP_TICK_OPEN + 10);
			}else{
				osEventFlagsWait(motion_arrive_eventHandle, EVENT_GRAP_ARRIVE, osFlagsWaitAll, osWaitForever);
			}

			seed_info->put_count++;

			dji_motor_setref(motor_lift_left,   0);
			dji_motor_setref(motor_lift_right,  0);

			if(get_put_parity(seed_info) == PUT_EVEN)
			{
				seed_info->seed_state = SEED_STATE_CORRECT;
			}
			else{
				grap_info->put_lift_tick = 0;
				seed_info->seed_state = SEED_STATE_MOVE_2_PUT;
			}
			break;
		}
		
		case SEED_STATE_CORRECT:
    	{
			ChassisCmd_t chassis_cmd;
			chassis_cmd.speed_target.target_vx = -sign_t * 0.8;
			chassis_cmd.speed_target.target_vy =  0.4;
			chassis_cmd.speed_target.target_vz = 0;

			chassis_cmd.mode_x = AXIS_MODE_VEL;
			chassis_cmd.mode_y = AXIS_MODE_VEL;
			chassis_cmd.mode_z = AXIS_MODE_VEL;

			osMessageQueuePut(chassis_cmd_queueHandle, &chassis_cmd, 0, 0);
			osDelay(120);

			if(seed_info->pos_index >= 6)
			{
				/* 修改为任务调度的形式, 切换到过渡任务 */
				// start_transition_task(robot_info, trans_info);
				break;
			}
			else
				seed_info->seed_state = SEED_STATE_MOVE_2_GET;

			break;
		}
		default:
			break;
	}
}

