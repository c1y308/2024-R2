#include "grap_module.h"

grap_t grap_ifo; 


DjiMotorHandle_t *motor_yaw_left, *motor_yaw_right, *motor_lift_left, *motor_lift_right, *motor_grap_left, *motor_grap_right;


bool check_grap_arrive(grap_t *grap_ifo){
	return grap_ifo->grap_arrive == 1;
}


void grap_motor_init()
{
    MotorInitConfig_t chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_init_config = {
			.angle_PID = {
				.Kp = 0.01,
				.Ki = 0.007,
				.Kd = 0.0f,
			},
            .speed_PID = {
                .Kp = 10000.0f,
                .Ki = 10.0f,
                .Kd = 0.0f,
                .IntegralLimit = M3508_MOTOR_SPEED_PID_IOUT_LIMIT,
                .MaxOut = M3508_MOTOR_SPEED_PID_POUT_LIMIT,
            },
			.outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
			.motor_direction = MOTOR_DIRECTION_REVERSE,
        },
		.motor_id = 0,
        .motor_type = DJI_MOTOR_2006,
    };

    
    chassis_motor_config.motor_id = 0;
    motor_yaw_left = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 1;
    motor_lift_left = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 2;
    motor_grap_left = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 3;
    motor_yaw_right = djimotor_init(&chassis_motor_config);

	chassis_motor_config.motor_id = 4;
    motor_yaw_right = djimotor_init(&chassis_motor_config);

	chassis_motor_config.motor_id = 5;
    motor_yaw_right = djimotor_init(&chassis_motor_config);
}


void grap_init(grap_t *grap_ifo)
{   
    grap_ifo->grap_state = GARP_STATE_INIT;
    grap_ifo->grap_last_state = GARP_STATE_IDLE;
	grap_ifo->grap_arrive = 0;


	grap_ifo->grap_tick++;
	dji_motor_setref(motor_yaw_left, - GRAP_ANGLE_SPEED);
	dji_motor_setref(motor_yaw_right,  GRAP_ANGLE_SPEED);

	dji_motor_setref(motor_lift_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_lift_right, - SAFE_GRAP_SPEED);
	
	dji_motor_setref(motor_grap_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_grap_right, - SAFE_GRAP_SPEED);

	if(grap_ifo->grap_tick >= 150)
	{
		dji_motor_setref(motor_yaw_left, - LOW_ANGLE_SPEED);
		dji_motor_setref(motor_yaw_right,  LOW_ANGLE_SPEED);

		dji_motor_setref(motor_lift_left,  - GRAP_LIFT_SPEED);
		dji_motor_setref(motor_lift_right,   GRAP_LIFT_SPEED);
		
		dji_motor_setref(motor_grap_left,    GRAP_SPEED_OPEN);
		dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);
	}
	else if(grap_ifo->grap_tick >= 300){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		set_grap_motor_zero_speed();
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}


/*  设置所有抓取电机速度为0 */
static void set_grap_motor_zero_speed()
{
	static uint8_t clear_pid_flag = 1;
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


void grap_seed_store(grap_t *grap_ifo){
	grap_ifo->grap_state = GRAP_STATE_START_STORAGE;
}


void grap_seed(grap_t *grap_ifo){
	grap_ifo->grap_state = GRAP_STATE_GRAP;
}


void put_seed(grap_t *grap_ifo){
	grap_ifo->grap_state = GRAP_STATE_WAIT_AND_PUT;
}

void grap_task(grap_t *grap_ifo){
	switch (grap_ifo->grap_state)
	{
		case GARP_STATE_IDLE:{
			grap_ifo->grap_tick = 0;
			break;
		}

		case GARP_STATE_INIT:{
			grap_ifo->grap_tick++;
			if(grap_ifo->grap_tick < 150){
				dji_motor_setref(motor_yaw_left, - GRAP_ANGLE_SPEED);
				dji_motor_setref(motor_yaw_right,  GRAP_ANGLE_SPEED);

				dji_motor_setref(motor_lift_left,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right,   SAFE_GRAP_SPEED);

				dji_motor_setref(motor_grap_left,    SAFE_GRAP_SPEED);
				dji_motor_setref(motor_grap_right, - SAFE_GRAP_SPEED);
			}
            else if(grap_ifo->grap_tick >= 150 && grap_ifo->grap_tick < 300){
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
                grap_ifo->grap_last_state = GRAP_STATE_START_STORAGE;
				grap_ifo->grap_state = GRAP_STATE_GRAP;
			break;
		}

		case GRAP_STATE_GRAP:{  // 抓取苗
			if(grap_ifo->grap_tick >= GRAP_TICK_CLOSE)
			{
                if(grap_ifo->grap_last_state == GRAP_STATE_START_STORAGE){  // 抓取完毕需要把苗存储在车上
                    grap_ifo->grap_state = GRAP_STATE_PRELIFT;
                }
                else if(grap_ifo->grap_last_state == GRAP_STATE_BACK2WAIT){  // 抓取完毕直接抬升准备种植
                    grap_ifo->grap_state = GARP_STATE_LIFT;
                }
                else if(grap_ifo->grap_last_state == GRAP_STATE_PUT_ROTATE){  // 抓取存储在车上的苗准备种植
                    grap_ifo->grap_state = GRAP_STATE_PUT_ROTATEBACK;
                }
				else if(grap_ifo->grap_last_state == GARP_STATE_IDLE)
					grap_ifo->grap_state = GRAP_STATE_PUT_ROTATEBACK;
					
                grap_ifo->grap_last_state = GRAP_STATE_GRAP;
                grap_ifo->grap_tick = 0;
			}else{
				grap_ifo->grap_tick++;
				grap_only();
			}
			break;
		}

		case GRAP_STATE_PRELIFT:{
			if(grap_ifo->grap_tick >= GRAP_LIFT_TICK_PRE)//预抬升到指定高度
			{
				grap_ifo->grap_tick = 0;
                grap_ifo->grap_last_state = GRAP_STATE_PRELIFT;
				grap_ifo->grap_state = GRAP_STATE_ROTATE;
			}
			else
			{
				dji_motor_setref(motor_lift_left,    SPEED_LIFT_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				grap_ifo->grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE:{
			if(grap_ifo->grap_tick >= CAUTION_TICK)//只有转了一定角度后才允许地盘进行移动防止地盘移动后转动打到场地苗
			{
				grap_ifo->ready_2_move = 1;//允许车移动到下一个取苗点位（考虑以信号量进行替代，此处释放信号量）
                grap_ifo->grap_last_state = GRAP_STATE_ROTATE;
				grap_ifo->grap_state = GRAP_STATE_ROTATE_2;
				grap_ifo->grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);

				grap_ifo->grap_tick++;
			}
			break;
		}
		case GRAP_STATE_ROTATE_2:{
			if(grap_ifo->grap_tick >= CONTINUE_STORAGE_TICK)//接着上一步继续转到位(next_state == 0 && grap_ifo.grap_arrive == 0)
			{
                grap_ifo->grap_last_state = GRAP_STATE_ROTATE_2;
				grap_ifo->grap_state = GRAP_STATE_OPEN;
				grap_ifo->grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED);

				dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED);
				
				if(grap_ifo->grap_tick >= GRAP_LIFT_TICK_PRE_COM)  // 降速
				{
					dji_motor_setref(motor_lift_left,   -1);
					dji_motor_setref(motor_lift_right,   1);
				}
				grap_ifo->grap_tick++;
			}
			break;
		}
		case GRAP_STATE_OPEN:{
			if(grap_ifo->grap_tick >= GRAP_TICK_OPEN)//打开爪子
			{
				dji_motor_setref(motor_grap_left,   0);
				dji_motor_setref(motor_grap_right, 0);
                if(grap_ifo->grap_last_state == GRAP_STATE_ROTATE_2)
				    grap_ifo->grap_state = GRAP_STATE_BACK2WAIT;

                grap_ifo->grap_last_state = GRAP_STATE_OPEN;
				grap_ifo->grap_tick = 0;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_left,   0);
				dji_motor_setref(motor_grap_left,   GRAP_SPEED_OPEN);

				dji_motor_setref(motor_yaw_right,  - SAFE_GRAP_SPEED);
				dji_motor_setref(motor_lift_right, - 0);
				dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);

				grap_ifo->grap_tick++;
			}
			break;
		}

		case GRAP_STATE_BACK2WAIT:{
			if(grap_ifo->grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 20)
			{
				dji_motor_setref(motor_yaw_left,    -0);
				dji_motor_setref(motor_lift_left,   -LOW_LIFT_SPEED);
				
				dji_motor_setref(motor_yaw_right,   0);
				dji_motor_setref(motor_lift_right,  LOW_LIFT_SPEED);
				
				grap_ifo->grap_arrive = 1;  // 划重点
				grap_ifo->grap_tick = 0;
                grap_ifo->grap_last_state = GRAP_STATE_BACK2WAIT;
				grap_ifo->grap_state = GARP_STATE_IDLE;
			}
			else
			{
				dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED - SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED  - SPEED_LIFT_PLUS);
				
				dji_motor_setref(motor_yaw_right,   GRAP_ANGLE_SPEED  + SPEED_ANGLE_PLUS);
				dji_motor_setref(motor_lift_right,  GRAP_LIFT_SPEED   + SPEED_LIFT_PLUS);

				grap_ifo->grap_tick++;
			}
				break;
		}

		case GARP_STATE_LIFT:{
			if(grap_ifo->grap_tick >= GRAP_LIFT_TICK_PRE)
			{
				grap_ifo->grap_tick = 0;
				grap_ifo->grap_arrive = 0;
                grap_ifo->grap_last_state = GARP_STATE_LIFT;
				grap_ifo->grap_state = GRAP_STATE_PRE_PUT;
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
		case GRAP_STATE_PRE_PUT:{
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
                grap_ifo->grap_state = GRAP_STATE_WAIT_AND_PUT;
                grap_ifo->grap_last_state = GRAP_STATE_PRE_PUT;
				input_tarpos_chassis(sign_t * put_pos_x[seed_ifo.pos_index - 2], put_pos_y[0], 0);
			}
			else  // 抓取存储在车上的苗并预放高度
			{
				grap_ifo->grap_state = GRAP_STATE_PUT_ROTATE;
                grap_ifo->grap_last_state = GRAP_STATE_PRE_PUT;
				input_tarpos_chassis(sign_t * put_pos_x[seed_ifo.pos_index - 2], put_pos_y[1], 0);
			}
		}

		case GRAP_STATE_PUT_ROTATE:{
			if(grap_ifo->grap_tick >= GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 70) //转动到存储苗的位置
			{
                grap_ifo->grap_last_state = GRAP_STATE_PUT_ROTATE;
				grap_ifo->grap_state = GRAP_STATE_GRAP;
				grap_ifo->grap_tick = 0;
			}
			else
			{
				grap_ifo->grap_tick++;
				if(grap_ifo->grap_tick >= 20)
				{
					dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS + 0.4);//减速+ 0.1
					dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS  - 0.4);//加速- 0.1

					dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS - 0.4);
					dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS  + 0.4);		
				}
			}
		}

		case GRAP_STATE_PUT_ROTATEBACK:{
			if(grap_ifo->grap_tick >= GRAP_LIFT_TICK_AFT + 50)
			{
                // 之前在这确认可以进行放苗
                grap_ifo->grap_last_state = GRAP_STATE_PUT_ROTATEBACK;
				grap_ifo->grap_state = GRAP_STATE_WAIT_AND_PUT;
				grap_ifo->grap_tick = 0;
			}
			else
			{
				grap_ifo->grap_tick++;
				if(grap_ifo->grap_tick >= START_DOWN_COM_TICK)//大于这个TICK才降低高度
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
            // 需要seed_task一个信号量来进行 ack 确认可以打开爪子放苗
			grap_ifo->grap_last_state = GRAP_STATE_WAIT_AND_PUT;
			grap_ifo->grap_state = GARP_STATE_IDLE;
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


// void Grapping_Callback()
// {
//   if((robot_ifo.blue_single_flag==1)||(robot_ifo.red_single_flag==1))//单项赛启用云台闭环计算
// 		Grap_Cal_2_Speed();
// 	  	PID_Cal_DJI_CAN2_Speed();//对CAN2上的电机进行速度环计算
// 		if(robot_ifo.blue_single_flag==1)
// 		  CAN_Cmd_Chassis_CAN2(send_current_CAN2[0],send_current_CAN2[1],send_current_CAN2[2],0);
// 		else if(robot_ifo.red_single_flag==1)
// 		{
// 			CAN_Cmd_Chassis_CAN2(0,0,0,send_current_CAN2[3]);
// 			CAN_Cmd_Gimbal_CAN2(send_current_CAN2[4],send_current_CAN2[5],0,0);
// 		}
// 		else if(robot_ifo.blue_single_flag==0&&robot_ifo.red_single_flag==0)
// 		{
// 		  CAN_Cmd_Chassis_CAN2(send_current_CAN2[0],send_current_CAN2[1],send_current_CAN2[2],send_current_CAN2[3]);
// 			CAN_Cmd_Gimbal_CAN2(send_current_CAN2[4],send_current_CAN2[5],send_current_CAN2[6],send_current_CAN2[7]);
// 		}
// }