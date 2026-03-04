#include "grap_task.h"

grap_t grap_ifo; 


void grap_init()
{   
    grap_ifo.grap_state = GARP_STATE_IDLE;
    grap_ifo.grap_last_state = GARP_STATE_IDLE;

	grap_ifo.grap_tick++;
	dji_motor_setref(motor_yaw_left, - GRAP_ANGLE_SPEED);
	dji_motor_setref(motor_yaw_right,  GRAP_ANGLE_SPEED);

	dji_motor_setref(motor_lift_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_lift_right, - SAFE_GRAP_SPEED);
	
	dji_motor_setref(motor_grap_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_grap_right, - SAFE_GRAP_SPEED);

	if(grap_ifo.grap_tick >= 150)
	{
		dji_motor_setref(motor_yaw_left, - LOW_ANGLE_SPEED);
		dji_motor_setref(motor_yaw_right,  LOW_ANGLE_SPEED);

		dji_motor_setref(motor_lift_left,  - GRAP_LIFT_SPEED);
		dji_motor_setref(motor_lift_right,   GRAP_LIFT_SPEED);
		
		dji_motor_setref(motor_grap_left,    GRAP_SPEED_OPEN);
		dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);
	}
	else if(grap_ifo.grap_tick >= 300)
		set_grap_motor_zero_speed();
}


void set_grap_motor_zero_speed()//设置目标速度为0
{
	static uint8_t clear_pid_flag  = 1;
	if(clear_pid_flag == 1)
	{
	    PID_clear(&motor_grap_left->motor_controller.speed_PID);
	    PID_clear(&motor_yaw_left->motor_controller.speed_PID);
		PID_clear(&motor_lift_left->motor_controller.speed_PID);

	    PID_clear(&motor_grap_right->motor_controller.speed_PID);
		PID_clear(&motor_yaw_right->motor_controller.speed_PID);
	    PID_clear(&motor_lift_right->motor_controller.speed_PID);
		clear_pid_flag = 0;
	}
	dji_motor_setref(motor_yaw_left, - 0);
	dji_motor_setref(motor_yaw_right,  0);

	dji_motor_setref(motor_lift_left,  - 0);
	dji_motor_setref(motor_lift_right,   0);
	
	dji_motor_setref(motor_grap_left,    0);
	dji_motor_setref(motor_grap_right, - 0);
}


void grap_task(){
	switch (grap_ifo.grap_state)
	{
		case GARP_STATE_IDLE:{
			grap_ifo.grap_tick = 0;
			break;
		}

		case GARP_STATE_INIT:{
			grap_ifo.grap_tick++;
			if(grap_ifo.grap_tick < 150){
				dji_motor_setref(motor_yaw_left, - GRAP_ANGLE_SPEED);
				dji_motor_setref(motor_yaw_right,  GRAP_ANGLE_SPEED);

				dji_motor_setref(motor_lift_left,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right,   SAFE_GRAP_SPEED);

				dji_motor_setref(motor_grap_left,    SAFE_GRAP_SPEED);
				dji_motor_setref(motor_grap_right, - SAFE_GRAP_SPEED);
			}
            else if(grap_ifo.grap_tick >= 150 && grap_ifo.grap_tick < 300){
				dji_motor_setref(motor_yaw_left, - LOW_ANGLE_SPEED);
				dji_motor_setref(motor_yaw_right,  LOW_ANGLE_SPEED);

				dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED);
				dji_motor_setref(motor_lift_right,    GRAP_LIFT_SPEED);

				dji_motor_setref(motor_grap_left,    GRAP_SPEED_OPEN);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);
			}
			else
				set_grap_motor_zero_speed();

			break;
		}
		/*************************************一共七个状态****************************************************/
		case GRAP_STATE_START_STORAGE:{  // 进入把场地上的苗抓取并存储状态
                grap_ifo.grap_last_state = GRAP_STATE_START_STORAGE;
				grap_ifo.grap_state = GRAP_STATE_GRAP;
			break;
		}

		case GRAP_STATE_GRAP:{  // 抓取苗
			if(grap_ifo.grap_tick >= GRAP_TICK_CLOSE)
			{
                if(grap_ifo.grap_last_state == GRAP_STATE_START_STORAGE){  // 抓取完毕需要把苗存储在车上
                    grap_ifo.grap_state = GRAP_STATE_PRELIFT;
                }
                else if(grap_ifo.grap_last_state == GRAP_STATE_BACK2WAIT){  // 抓取完毕直接抬升准备种植
                    grap_ifo.grap_state = GARP_STATE_LIFT;
                }
                else if(grap_ifo.grap_last_state == GRAP_STATE_PUT_ROTATE){  // 抓取存储在车上的苗准备种植
                    grap_ifo.grap_state = GRAP_STATE_PUT_ROTATEBACK;
                }
				else if(grap_ifo.grap_last_state == GARP_STATE_IDLE)
					grap_ifo.grap_state = GRAP_STATE_PUT_ROTATEBACK;
					
                grap_ifo.grap_last_state = GRAP_STATE_GRAP;
                grap_ifo.grap_tick = 0;
			}else{
				grap_ifo.grap_tick++;
				grap_only();
			}
			break;
		}

		case GRAP_STATE_PRELIFT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE)//预抬升到指定高度
			{
				grap_ifo.grap_tick = 0;
                grap_ifo.grap_last_state = GRAP_STATE_PRELIFT;
				grap_ifo.grap_state = GRAP_STATE_ROTATE;
			}
			else
			{
				dji_motor_setref(motor_lift_left,    SPEED_LIFT_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE:{
			if(grap_ifo.grap_tick >= CAUTION_TICK)//只有转了一定角度后才允许地盘进行移动防止地盘移动后转动打到场地苗
			{
				grap_ifo.ready_2_move = 1;//允许车移动到下一个取苗点位（考虑以信号量进行替代，此处释放信号量）
                grap_ifo.grap_last_state = GRAP_STATE_ROTATE;
				grap_ifo.grap_state = GRAP_STATE_ROTATE_2;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);

				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE_2:{
			if(grap_ifo.grap_tick >= CONTINUE_STORAGE_TICK)//接着上一步继续转到位(next_state == 0 && grap_ifo.grap_arrive == 0)
			{
                grap_ifo.grap_last_state = GRAP_STATE_ROTATE_2;
				grap_ifo.grap_state = GRAP_STATE_OPEN;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
				
				if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE_COM)  // 降速
				{
					dji_motor_setref(motor_lift_left,   -1);
					dji_motor_setref(motor_lift_right,   1);
				}
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_OPEN:{
			if(grap_ifo.grap_tick >= GRAP_TICK_OPEN)//打开爪子
			{
				dji_motor_setref(motor_grap_left,   0);
				dji_motor_setref(motor_grap_right, 0);
                if(grap_ifo.grap_last_state == GRAP_STATE_ROTATE_2)
				    grap_ifo.grap_state = GRAP_STATE_BACK2WAIT;

                grap_ifo.grap_last_state = GRAP_STATE_OPEN;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED_OPEN);

				dji_motor_setref(motor_yaw_right,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right, - 0);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);

				grap_ifo.grap_tick++;
			}
			break;
		}

		case GRAP_STATE_BACK2WAIT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 20)
			{
				dji_motor_setref(motor_yaw_left,    -0);
				dji_motor_setref(motor_lift_left,   -LOW_LIFT_SPEED);
				
				dji_motor_setref(motor_yaw_right,   0);
				dji_motor_setref(motor_lift_right,  LOW_LIFT_SPEED);
				
				grap_ifo.grap_arrive = 1;  // 划重点
				grap_ifo.grap_tick = 0;
                grap_ifo.grap_last_state = GRAP_STATE_BACK2WAIT;
				grap_ifo.grap_state = GARP_STATE_IDLE;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED  - SPEED_LIFT_PLUS);
				
				dji_motor_setref(motor_yaw_right,   GRAP_ANGLE_SPEED  + SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right,  GRAP_LIFT_SPEED   + SPEED_LIFT_PLUS);

				grap_ifo.grap_tick++;
			}
				break;
		}

		case GARP_STATE_LIFT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE)
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_arrive = 0;
                grap_ifo.grap_last_state = GARP_STATE_LIFT;
				grap_ifo.grap_state = GRAP_STATE_PRE_DOWN;
			}
			else
			{
				dji_motor_setref(motor_lift_left,     SPEED_LIFT_PLUS);
				dji_motor_setref(motor_lift_right,  - SPEED_LIFT_PLUS);
				
				dji_motor_setref(motor_yaw_left,     0.1);
				dji_motor_setref(motor_yaw_right,  - 0.1);
				
				dji_motor_setref(motor_grap_left,    GRAP_SPEED);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
			}
			break;
		}
		/***********************************************************************************************/
		case GRAP_STATE_PRE_DOWN:{
			if(seed_ifo.putm_count % 2 == 0)  // 降低高度预放苗
			{
				if(seed_ifo.run_tick <= 280)
				{
					dji_motor_setref(motor_lift_left,    2.3);
					dji_motor_setref(motor_lift_right, - 2.3);
				}
				else
				{
					dji_motor_setref(motor_lift_left,   0);
					dji_motor_setref(motor_lift_right,  0);
				}
                grap_ifo.grap_state = GRAP_STATE_WAIT_AND_PUT;
                grap_ifo.grap_last_state = GRAP_STATE_PRE_DOWN;
				input_tarpos_chassis(sign_t * put_pos_x[seed_ifo.pos_index - 2], put_pos_y[0], 0);
			}
			else  // 抓取存储在车上的苗并预放高度
			{
				grap_ifo.grap_state = GRAP_STATE_PUT_ROTATE;
                grap_ifo.grap_last_state = GRAP_STATE_PRE_DOWN;
				input_tarpos_chassis(sign_t * put_pos_x[seed_ifo.pos_index - 2], put_pos_y[1], 0);
			}
		}

		case GRAP_STATE_PUT_ROTATE:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 70) //转动到存储苗的位置
			{
                grap_ifo.grap_last_state = GRAP_STATE_PUT_ROTATE;
				grap_ifo.grap_state = GRAP_STATE_GRAP;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				grap_ifo.grap_tick++;
				if(grap_ifo.grap_tick >= 20)
				{
					dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS + 0.4);//减速+ 0.1
					dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS  - 0.4);//加速- 0.1

					dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS - 0.4);
					dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS  + 0.4);		
				}
			}
		}

		// case GRAP_STATE_PUT_GRAP:{
		// 	if(grap_ifo.grap_tick >= GRAP_TICK_CLOSE + 20)
		// 	{
        //         grap_ifo.grap_last_state = GRAP_STATE_PUT_GRAP;
		// 		grap_ifo.grap_state = GRAP_STATE_PUT_ROTATEBACK;
		// 		grap_ifo.grap_tick = 0;
		// 	}
		// 	else
		// 	{
		// 		dji_motor_setref(motor_yaw_left,    SAFE_GRAP_SPEED);//减速+ 0.1
		// 		dji_motor_setref(motor_lift_left,   - 1);//加速- 0.1
		// 		dji_motor_setref(motor_grap_left,   GRAP_SPEED);

		// 		dji_motor_setref(motor_yaw_right,  - SAFE_GRAP_SPEED);
		// 		dji_motor_setref(motor_lift_right,    1);		
		// 		dji_motor_setref(motor_grap_right, - GRAP_SPEED);				
		// 		grap_ifo.grap_tick++;
		// 	}
		// }

		case GRAP_STATE_PUT_ROTATEBACK:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + 50)
			{
                // 之前在这确认可以进行放苗
                grap_ifo.grap_last_state = GRAP_STATE_PUT_ROTATEBACK;
				grap_ifo.grap_state = GRAP_STATE_WAIT_AND_PUT;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				grap_ifo.grap_tick++;
				if(grap_ifo.grap_tick >= START_DOWN_COM_TICK)//大于这个TICK才降低高度
				{
					dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED);//减速+ 0.1
					dji_motor_setref(motor_lift_left,   - SPEED_LIFT_PLUS  + 0.8);//加速- 0.1
					dji_motor_setref(motor_grap_left,     GRAP_SPEED);

					dji_motor_setref(motor_yaw_right,     GRAP_ANGLE_SPEED);
					dji_motor_setref(motor_lift_right,    SPEED_LIFT_PLUS - 0.8);		
					dji_motor_setref(motor_grap_right,  - GRAP_SPEED);	
				}
				else{
					dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED);//减速+ 0.1
					dji_motor_setref(motor_lift_left,     0);
					dji_motor_setref(motor_grap_left,     GRAP_SPEED);

					dji_motor_setref(motor_yaw_right,     GRAP_ANGLE_SPEED);
					dji_motor_setref(motor_lift_right,    0);		
					dji_motor_setref(motor_grap_right,  - GRAP_SPEED);	
				}
			}
		}

        case GRAP_STATE_WAIT_AND_PUT:{
            // 需要seed_task一个信号量来进行ack确认可以打开爪子放苗
			grap_ifo.grap_last_state = GRAP_STATE_WAIT_AND_PUT;
			grap_ifo.grap_state = GARP_STATE_IDLE;
            out_only();
			break;
		}
		default:
			break;
	}
}


//左侧爪子负数为抬升，负数为转向存储方向
void grap_only()//闭合夹爪
{
	dji_motor_setref(motor_yaw_left,  0);
	dji_motor_setref(motor_lift_left, 0);
	dji_motor_setref(motor_grap_left,    GRAP_SPEED);

	dji_motor_setref(motor_yaw_right,  0);
	dji_motor_setref(motor_lift_right, 0);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED);
}


void out_only()
{
	dji_motor_setref(motor_yaw_left,  0);
	dji_motor_setref(motor_lift_left, 0);
	dji_motor_setref(motor_grap_left,    GRAP_SPEED_OPEN);

	dji_motor_setref(motor_yaw_right,  0);
	dji_motor_setref(motor_lift_right, 0);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);
}


void grap_storage_pre()//把场地上的苗存储在车上第一步
{
switch(grap_ifo.grap_state)
	{
		case GARP_STATE_IDLE:{
			if(grap_ifo.grap_tick >= GRAP_TICK_CLOSE)//抓取苗
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_state = GRAP_STATE_PRELIFT;
			}
			else
			{
				grap_only();
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_PRELIFT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE)//预抬升到指定高度
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_state = GRAP_STATE_ROTATE;
			}
			else
			{
				dji_motor_setref(motor_lift_left,    SPEED_LIFT_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE:{
			if(grap_ifo.grap_tick >= CAUTION_TICK)//只有转了一定角度后才允许地盘进行移动防止地盘移动后转动打到场地苗
			{
				grap_ifo.ready_2_move = 1;//允许车移动到下一个取苗点位（考虑以信号量进行替代，此处释放信号量）
				grap_ifo.grap_state = GRAP_STATE_ROTATE_2;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);

				grap_ifo.grap_tick++;
			}
			break;
		}
		default:
			break;
	}
}


void grap_storage_last()//把场地苗存储到车上第二步
{
	switch (grap_ifo.grap_state)
	{
		case GRAP_STATE_ROTATE_2:{
			if(grap_ifo.grap_tick >= CONTINUE_STORAGE_TICK)//接着上一步继续转到位(next_state == 0 && grap_ifo.grap_arrive == 0)
			{
				grap_ifo.grap_state = GRAP_STATE_STORAGE;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
				
				if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE_COM)
				{
					dji_motor_setref(motor_lift_left,   -1);
					dji_motor_setref(motor_lift_right,   1);
				}
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_STORAGE:{
			if(grap_ifo.grap_tick >= GRAP_TICK_OPEN)//打开爪子
			{
				dji_motor_setref(motor_grap_left,   0);
				dji_motor_setref(motor_grap_right, 0);
				grap_ifo.grap_state = GRAP_STATE_BACK2WAIT;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED_OPEN);

				dji_motor_setref(motor_yaw_right,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right, - 0);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);

				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_BACK2WAIT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 20)
			{
				dji_motor_setref(motor_yaw_left,    -0);
				dji_motor_setref(motor_lift_left,   -LOW_LIFT_SPEED);
				
				dji_motor_setref(motor_yaw_right,   0);
				dji_motor_setref(motor_lift_right,  LOW_LIFT_SPEED);
				
				grap_ifo.grap_arrive = 1;
				grap_ifo.grap_tick = 0;

				grap_ifo.grap_state = GARP_STATE_IDLE;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED  - SPEED_LIFT_PLUS);
				
				dji_motor_setref(motor_yaw_right,   GRAP_ANGLE_SPEED  + SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right,  GRAP_LIFT_SPEED   + SPEED_LIFT_PLUS);

				grap_ifo.grap_tick++;
			}
				break;
		}
		default:
			break;
	}
}


void grap_storage_lift()//抓了之后抬升
{
	switch(grap_ifo.grap_state){
		case GARP_STATE_IDLE:{
			if(grap_ifo.grap_tick > GRAP_TICK_CLOSE)
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_state = GARP_STATE_LIFT;
			}
			else
			{
				grap_ifo.grap_tick++;
				dji_motor_setref(motor_grap_left,    GRAP_SPEED);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
				
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_lift_right,  0);
						
				dji_motor_setref(motor_yaw_left,    0);
				dji_motor_setref(motor_yaw_right,   0);
			}
		}
		case GARP_STATE_LIFT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE)
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_arrive = 0;
				grap_ifo.grap_state = GARP_STATE_IDLE;
			}
			else
			{
				dji_motor_setref(motor_lift_left,     SPEED_LIFT_PLUS);
				dji_motor_setref(motor_lift_right,  - SPEED_LIFT_PLUS);
				
				dji_motor_setref(motor_yaw_left,     0.1);
				dji_motor_setref(motor_yaw_right,  - 0.1);
				
				dji_motor_setref(motor_grap_left,    GRAP_SPEED);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
			}
			break;
		}
		default:{
			dji_motor_setref(motor_yaw_left,     - 0);
			dji_motor_setref(motor_yaw_right,      0);
			
			dji_motor_setref(motor_grap_left,    0);
			dji_motor_setref(motor_grap_right,   0);
			break;
		}
	}
}


void grap_store()  // 存储苗
{
	switch(grap_ifo.grap_state)
	{
		case GARP_STATE_IDLE:{
			if(grap_ifo.grap_tick >= GRAP_TICK_CLOSE)//抓取苗
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_state = GRAP_STATE_PRELIFT;
			}
			else
			{
				grap_only();
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_PRELIFT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE)//预抬升到指定高度
			{
				grap_ifo.grap_tick = 0;
				grap_ifo.grap_state = GRAP_STATE_ROTATE;
			}
			else
			{
				dji_motor_setref(motor_lift_left,    SPEED_LIFT_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE:{
			if(grap_ifo.grap_tick >= CAUTION_TICK)//只有转了一定角度后才允许地盘进行移动防止地盘移动后转动打到场地苗
			{
				grap_ifo.ready_2_move = 1;//允许车移动到下一个取苗点位（考虑以信号量进行替代，此处释放信号量）
				grap_ifo.grap_state = GRAP_STATE_ROTATE_2;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);

				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE_2:{
			if(grap_ifo.grap_tick >= CONTINUE_STORAGE_TICK)//接着上一步继续转到位(next_state == 0 && grap_ifo.grap_arrive == 0)
			{
				grap_ifo.grap_state = GRAP_STATE_STORAGE;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
				
				if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_PRE_COM)
				{
					dji_motor_setref(motor_lift_left,   -1);
					dji_motor_setref(motor_lift_right,   1);
				}
				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_STORAGE:{
			if(grap_ifo.grap_tick >= GRAP_TICK_OPEN)//打开爪子
			{
				dji_motor_setref(motor_grap_left,   0);
				dji_motor_setref(motor_grap_right, 0);
				grap_ifo.grap_state = GRAP_STATE_BACK2WAIT;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED_OPEN);

				dji_motor_setref(motor_yaw_right,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right, - 0);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);

				grap_ifo.grap_tick++;
			}
			break;
		}
		case GRAP_STATE_BACK2WAIT:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 20)
			{
				dji_motor_setref(motor_yaw_left,    -0);
				dji_motor_setref(motor_lift_left,   -LOW_LIFT_SPEED);
				
				dji_motor_setref(motor_yaw_right,   0);
				dji_motor_setref(motor_lift_right,  LOW_LIFT_SPEED);
				
				grap_ifo.grap_arrive = 1;
				grap_ifo.grap_tick = 0;

				grap_ifo.grap_state = GARP_STATE_IDLE;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED  - SPEED_LIFT_PLUS);
				
				dji_motor_setref(motor_yaw_right,   GRAP_ANGLE_SPEED  + SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right,  GRAP_LIFT_SPEED   + SPEED_LIFT_PLUS);

				grap_ifo.grap_tick++;
			}
				break;
		}
		case GRAP_STATE_PUT:{
			out_only();
		}
		default:
			break;
	}
}


void out_storage_pre()//种植存储的苗的一阶段
{
	switch(grap_ifo.grap_state){
		case GRAP_STATE_PUT_ROTATE:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 70)//转动到取苗位置
			{
				grap_ifo.grap_state = GRAP_STATE_PUT_GRAP;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				grap_ifo.grap_tick++;
				if(grap_ifo.grap_tick >= 20)
				{
					dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS + 0.4);//减速+ 0.1
					dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS  - 0.4);//加速- 0.1

					dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS - 0.4);
					dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS  + 0.4);		
				}
			}
		}
		case GRAP_STATE_PUT_GRAP:{
			if(grap_ifo.grap_tick >= GRAP_TICK_CLOSE + 20)
			{
				grap_ifo.grap_state = GRAP_STATE_PUT_ROTATEBACK;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SAFE_GRAP_SPEED);//减速+ 0.1
				dji_motor_setref(motor_lift_left,   - 1);//加速- 0.1
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right,    1);		
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);				
				grap_ifo.grap_tick++;
			}
		}
		case GRAP_STATE_PUT_ROTATEBACK:{
			if(grap_ifo.grap_tick >= GRAP_LIFT_TICK_AFT + 50)
			{
				grap_ifo.middle_go_put = 1;//可以进行放苗
				grap_ifo.grap_state = GARP_STATE_IDLE;
				grap_ifo.grap_tick = 0;
			}
			else
			{
				grap_ifo.grap_tick++;
				if(grap_ifo.grap_tick >= START_DOWN_COM_TICK)//大于这个TICK才降低高度
				{
					dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED);//减速+ 0.1
					dji_motor_setref(motor_lift_left,   - SPEED_LIFT_PLUS  + 0.8);//加速- 0.1
					dji_motor_setref(motor_grap_left,     GRAP_SPEED);

					dji_motor_setref(motor_yaw_right,     GRAP_ANGLE_SPEED);
					dji_motor_setref(motor_lift_right,    SPEED_LIFT_PLUS - 0.8);		
					dji_motor_setref(motor_grap_right,  - GRAP_SPEED);	
				}
				else{
					dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED);//减速+ 0.1
					dji_motor_setref(motor_lift_left,     0);
					dji_motor_setref(motor_grap_left,     GRAP_SPEED);

					dji_motor_setref(motor_yaw_right,     GRAP_ANGLE_SPEED);
					dji_motor_setref(motor_lift_right,    0);		
					dji_motor_setref(motor_grap_right,  - GRAP_SPEED);	
				}
			}
		}
		default:
			break;
	}
}



void Grap_Init_Single()//单项赛初始化代码
{
	grap_ifo.grap_tick++;
	if(robot_ifo.blue_single_flag==1)
	{
		DJI_motor_CAN2[0].target_angle =   GET_ANGLE_SINGLE;//转动到取苗角度
		DJI_motor_CAN2[1].target_speed =   SAFE_GRAP_SPEED;
		DJI_motor_CAN2[2].target_speed =   SAFE_GRAP_SPEED;
	}
	else if(robot_ifo.red_single_flag==1)
	{
		DJI_motor_CAN2[3].target_angle = - GET_ANGLE_SINGLE;//转动到取苗角度
		DJI_motor_CAN2[4].target_speed = - SAFE_GRAP_SPEED;
		DJI_motor_CAN2[5].target_speed = - SAFE_GRAP_SPEED;
	}
	if(grap_ifo.grap_tick >= 230)
	{
		if(robot_ifo.blue_single_flag==1)
		{
			DJI_motor_CAN2[1].target_speed = - GRAP_LIFT_SPEED;
			DJI_motor_CAN2[2].target_speed =   GRAP_SPEED_OPEN + 1;		
		}
		else if(robot_ifo.red_single_flag==1)
		{
			DJI_motor_CAN2[4].target_speed =   GRAP_LIFT_SPEED;
			DJI_motor_CAN2[5].target_speed = - GRAP_SPEED_OPEN - 1;
		}
	}
	else if(grap_ifo.grap_tick >= 400)
		set_grap_motor_zero_speed();
}


// void Grap_Cal_2_Speed()//求解得到云台旋转的目标速度
// {
// 	static uint8_t grap_arrive_check = 0;
// 	/*******************************云台转动角度误差*********************************/
// 	grap_ifo.ErrorAngle[0] = -DJI_motor_CAN2[0].target_angle + DJI_motor_CAN2[0].angle;
// 	grap_ifo.ErrorAngle[1] = -DJI_motor_CAN2[3].target_angle + DJI_motor_CAN2[3].angle;
// 	/********************************************************************************/
// 	if(fabs(grap_ifo.ErrorAngle[0])<=10&&fabs(grap_ifo.ErrorAngle[1])<=10)
// 	{
// 		grap_arrive_check++;
// 		if(grap_arrive_check>=10)
// 		{
// 			grap_ifo.grap_arrive = 1;
// 			grap_arrive_check = 0;
// 		}
// 		return;
// 	}
// 	grap_ifo.grap_arrive = 0;
// 	if(robot_ifo.blue_single_flag == 1)
//     PID_Calc(&pid_DJI_outer_CAN2[0], grap_ifo.ErrorAngle[0],0);
// 	else if(robot_ifo.red_single_flag == 1)
// 	  PID_Calc(&pid_DJI_outer_CAN2[1], grap_ifo.ErrorAngle[1],0);
// }


void Grapping_Callback()
{
  if((robot_ifo.blue_single_flag==1)||(robot_ifo.red_single_flag==1))//单项赛启用云台闭环计算
		Grap_Cal_2_Speed();
	  	PID_Cal_DJI_CAN2_Speed();//对CAN2上的电机进行速度环计算
		if(robot_ifo.blue_single_flag==1)
		  CAN_Cmd_Chassis_CAN2(send_current_CAN2[0],send_current_CAN2[1],send_current_CAN2[2],0);
		else if(robot_ifo.red_single_flag==1)
		{
			CAN_Cmd_Chassis_CAN2(0,0,0,send_current_CAN2[3]);
			CAN_Cmd_Gimbal_CAN2(send_current_CAN2[4],send_current_CAN2[5],0,0);
		}
		else if(robot_ifo.blue_single_flag==0&&robot_ifo.red_single_flag==0)
		{
		  CAN_Cmd_Chassis_CAN2(send_current_CAN2[0],send_current_CAN2[1],send_current_CAN2[2],send_current_CAN2[3]);
			CAN_Cmd_Gimbal_CAN2(send_current_CAN2[4],send_current_CAN2[5],send_current_CAN2[6],send_current_CAN2[7]);
		}
}