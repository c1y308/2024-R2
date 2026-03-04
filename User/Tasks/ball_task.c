#include "ball_task.h"
#include "stdlib.h"
#include "can.h"
#include "user_config.h"
BallInfo_t ball_ifo;
int8_t ball_temp[3];//当前所在球，目标球，中间辅助

DjiMotorHandle_t *motor2006_front, *motor2006_back, *motor3508_lift;
void seed_motor_init()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    MotorInitConfig_t chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
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
        },
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
		.motor_id = 0,
        .motor_type = DJI_MOTOR_2006,
    };

    chassis_motor_config.motor_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor2006_front = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 5;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor2006_back = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 6;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor3508_lift = djimotor_init(&chassis_motor_config);
}


void ball_init()
{
	ball_ifo.now_ball = 0;
	ball_ifo.target_ball = 0;
	ball_ifo.current_state = BALL_STATE_CHECK;
	ball_temp[0] = 0;
	ball_ifo.line_lr = 0;//如果需要换行,决定从左还是右
	ball_ifo.line_fb = 0;//如果需要换行,是从后往前还是从前往后
	ball_ifo.protect_flag_2 = 1;
}

void ball_task()
{
 	static uint8_t gpio_safe_flag = 1;
	if(gpio_safe_flag == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		gpio_safe_flag = 0;
	}

  	switch(ball_ifo.current_state)
	{
		case BALL_STATE_CHECK:{
			dji_motor_setref(motor2006_front, M2006_GET_BALL_SPEED);
			dji_motor_setref(motor2006_back,  M2006_GET_BALL_SPEED);
			dji_motor_setref(motor3508_lift, 0);


			ball_ifo.confirm_flag = 1;//允许reversal可以增加一次
			ball_ifo.protect_flag = 1;//允许
			if(ball_ifo.target_ball == 0 || ball_ifo.target_ball == ball_temp[0])//没有接受到取球指令,或者目标球是当前球（没有更新目标球）
		    	break;
	   		else
      		{
				ball_temp[2] = ball_temp[0];  //记录移动前的当前球，即使移动了也不改变当前球
				ball_temp[1] = ball_ifo.target_ball;  //遥控器发送目标球到这里
				
				if(ball_temp[0] <= 6)
					ball_ifo.line_now = 1;
				else
					ball_ifo.line_now = 2;
				
				/**************************************************************/
	      		if((ball_temp[1] - ball_temp[0]) == 6 && ball_temp[2] != 0 && ball_ifo.special_flag == 0)//第一行取第二行的特殊情况
				{
					ball_ifo.special_flag = 1;//不允许再次进入
					ball_temp[0] = ball_temp[2];//不改变当前所在位置
					ball_ifo.line_now = 1;
					ball_ifo.current_state = BALL_STATE_GET_BALL_POS_SPECIAL_F;
					break;
				}
				else if((ball_temp[1] - ball_temp[0]) == -6 &&ball_temp[2]!= 0 && ball_ifo.special_flag == 0)
				{
					ball_ifo.special_flag = 1;//不允许再次进入
					ball_temp[0] = ball_temp[2];//不改变当前所在位置
					ball_ifo.line_now = 2;
					ball_ifo.current_state = BALL_STATE_GET_BALL_POS_SPECIAL_B;
					break;
				}
				else if(abs(ball_temp[1] - ball_temp[0]) != 6)
				{	
					ball_ifo.special_flag = 0;//取了其他球就允许进入取特殊球模式
					if(ball_temp[0] <= 6 && ball_temp[1] >= 7 && ball_temp[2] != 0)//需要从第一行换到第二行去
					{
						ball_ifo.line_fb = BALL_FORWARD;
						if(ball_temp[0] <= 3 && 7 <= ball_temp[1] && ball_temp[1] <= 9)//从左边换过去
							ball_ifo.line_lr = BALL_LEFT;
						else if(ball_temp[0] <= 3 && ball_temp[1] >= 10)//从右边换过去
							ball_ifo.line_lr = BALL_RIGHT;
						else if(4 <= ball_temp[0] && ball_temp[0] <= 6 && ball_temp[1] >= 10)//从右边换过去
							ball_ifo.line_lr = BALL_RIGHT;
						else if(4 <= ball_temp[0] && ball_temp[0] <= 6 && ball_temp[1] <= 9)//从左边换过去
							ball_ifo.line_lr = BALL_LEFT;
						ball_ifo.line_now = 2;//预更新当前所在行
						ball_ifo.current_state = BALL_STATE_LINE_CHANGE_F;
					}
		
					else if(ball_temp[0]>=7&&ball_temp[1]<=6&&ball_temp[2]!=0)//需要从第二行换到第一行去
					{
						ball_ifo.line_fb = BALL_BACK;
						if(ball_temp[0]<=9&&ball_temp[1]<=3)//从左边换过去
							ball_ifo.line_lr = BALL_LEFT;
				
						else if(ball_temp[0]<=9&&ball_temp[1]>=4)//从右边换过去
							ball_ifo.line_lr = BALL_RIGHT;
				
						else if(ball_temp[0]>=10 && ball_temp[1]>=4)//从右边换过去
							ball_ifo.line_lr = BALL_RIGHT;
			
						else if(ball_temp[0]>=10 && ball_temp[1]<=3)//从左边换过去
							ball_ifo.line_lr = BALL_LEFT;
						ball_ifo.line_now = 1;//预更新当前所在行
						ball_ifo.current_state = BALL_STATE_LINE_CHANGE_F;
					}
					else//不需要换行（第一次接受命令从此进入）
					{
						ball_ifo.line_lr = ball_ifo.line_fb = 0;//不需要换行
						ball_ifo.current_state = BALL_STATE_GET_BALL_POS_F;
						if(ball_temp[0] == 0 && ball_temp[1]>6)//第一次接受命令如果要前往第二行
						{
								ball_ifo.line_now = 2;
//								ball_ifo.line_fb = BALL_FORWARD;
//								ball_ifo.line_lr = BALL_LEFT;
//								ball_ifo.current_state = line_change_f;
						}
						
					}	
					ball_temp[0] = ball_temp[1];//预更新当前所在球
					break;
				}
				break;
		    }
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_F:{//取第二行的特殊球
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=50)
			{
			  	ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_T;
			}
			else
			{
				dji_motor_setref(motor3508_lift, -0.9);
				//DJI_motor[6].target_speed = -0.9;//减小上球速度
				ball_ifo.disable_rotation_flag = 1;
			  	ball_ifo.bt_check++;
        		if(ball_ifo.bt_check <= 100)//清除到位标志位
          		robot_ifo.chassis_arrive = 0;	
        		ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//调整射球转速
				Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);					
				input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[0]], pos_get_special[0], 0);
				break;
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_B:{//取第二行的特殊球
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=50)
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_T;
			}
			else
			{ 
				dji_motor_setref(motor3508_lift, -0.9);
				//DJI_motor[6].target_speed = -0.9;//减小上球速度
				ball_ifo.disable_rotation_flag = 1;//禁止旋转
			  	ball_ifo.bt_check++;	
				if(ball_ifo.bt_check <= 100)//清除到位标志位
          			robot_ifo.chassis_arrive=0;	
        		ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//调整射球转速
				Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);				
        		input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[0]],pos_get_special[1],0);
			}
		  	break;
		}

	  	case BALL_STATE_LINE_CHANGE_F:{//换行第一个点位
			if(robot_ifo.chassis_arrive==1 && ball_ifo.bt_check>=20)
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_LINE_CHANGE_S;
			}
			else
			{
				ball_ifo.bt_check++;
				if(ball_ifo.line_fb == BALL_FORWARD)//需要前往第二行
			    	robot_ifo.pos_target.pos_y = ball_pos_y[0];//此处的y为第1行的坐标
				else if(ball_ifo.line_fb == BALL_BACK)//需要前往第一行
					robot_ifo.pos_target.pos_y = ball_pos_y[1];//此处的y为第2行的坐标
				if(ball_ifo.line_lr == BALL_LEFT)
				{
           			input_tarpos_chassis(sign_t * change_line_x_left, robot_ifo.pos_target.pos_y, 0);
				}
				else if(ball_ifo.line_lr == BALL_RIGHT)
				{
           			input_tarpos_chassis(sign_t * change_line_x_right, robot_ifo.pos_target.pos_y, 0);
				}
			}
		 	break;
		}
		
	  	case BALL_STATE_LINE_CHANGE_S:{//换行第二个点位
			//if(ball_ifo.bt_check >= CHANGE_LINE_TICK && ball_ifo.bt_check >= 20)//恢复注释
			                          //在user_config第14行（在一区测试，有危险）
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 20)//注释
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_F;
			}
			else
			{
			  ball_ifo.bt_check++;//不操作
				/*********************取消注释*****************************/
//				robot_ifo.chassis_mode = mix_mode_y;		
//			  if(ball_ifo.line_fb == BALL_FORWARD)//需要前往第二行
//				{
//				  robot_ifo.target_vy_direct =  1.4;
//				}
//				else if(ball_ifo.line_fb == BALL_BACK)//需要前往第一行
//				{
//				  robot_ifo.target_vy_direct = -1.4;
//				}
        /*************************以下进行注释*****************************/
				if(ball_ifo.line_lr == BALL_LEFT)
			    robot_ifo.pos_target.pos_x = change_line_x_left;
				else if(ball_ifo.line_lr == BALL_RIGHT)
					robot_ifo.pos_target.pos_x = change_line_x_right;
				if(ball_ifo.line_fb == BALL_FORWARD)//需要前往第二行
				{
				  input_tarpos_chassis(sign_t * robot_ifo.pos_target.pos_x, ball_pos_y[1], 0);
				}
				else if(ball_ifo.line_fb == BALL_BACK)//需要前往第一行
				{
				  input_tarpos_chassis(sign_t * robot_ifo.pos_target.pos_x, ball_pos_y[0], 0);
				}
				/*******************************************************************/
			}
		 	break;
		}
		
		case BALL_STATE_GET_BALL_POS_F:{//先到达目标球的正前方(侧向移动)
			static uint16_t reversal = 0;
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 50)
			{
				ball_ifo.protect_flag = 0;
				if(ball_ifo.confirm_flag == 1)//只允许加一次
				{
					reversal++;
					ball_ifo.confirm_flag = 0;
				}
				if(reversal % 2 == 1 && ball_ifo.ball_confirm % 2 != 0)//只有确认了才能切换
				{
					ball_ifo.current_state = BALL_STATE_GET_BALL_POS_S;//第一次进入切换进行取球(此时需要暂停确认)
					ball_ifo.bt_check = 0;
				}
				else if(reversal % 2 == 0)
				{
				  ball_ifo.current_state = BALL_STATE_GET_BALL_POS_T;//第二次切换直接自动进行移动到射球位置（不需要确认）
					ball_ifo.bt_check = 0;
				}
			}
			else if(ball_ifo.protect_flag == 1)//防止多次进入
			{
				if(reversal % 2 == 1)//后退向发射位置
				{
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],sign_t*ball_launch_angle[ball_temp[1]]);
					robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;
					ball_ifo.bt_check++;	
				}
				else if(reversal % 2 == 0)//此时正向目标球的正前方开（侧向移动）
				{
					robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//地盘移动为高精度移动
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],0);
					if(robot_ifo.stop_flag != 1)//没有停止
					{
						ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//调整射球转速
						Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);
					}
					
					ball_ifo.bt_check++;	
				}
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_S:{//取球动作，向前开
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 50)
			{
			  	ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_F;//回到目标球的正前方
			}
			else
			{
				ball_ifo.confirm_flag = 1;//回来重置标志位，要再次到目标球前方
				ball_ifo.protect_flag = 1;
				robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//地盘移动为高精度移动
			  	ball_ifo.bt_check++;
				if(ball_ifo.line_now == 1)
				{
					input_tarpos_chassis(sign_t * ball_pos_x[ball_temp[1]], pos_get_1, 0);
				}
				else if(ball_ifo.line_now == 2)
				{
					input_tarpos_chassis(sign_t * ball_pos_x[ball_temp[1]], pos_get_2, 0);
				}				
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_T:{//（取到了球地盘回退，并发射）
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 50)
			{
				ball_ifo.disable_rotation_flag = 0;
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_BALL_ACTION;
			}
			else
			{
				robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//地盘移动为高精度移动
			  	ball_ifo.bt_check++;
        		if(ball_ifo.disable_rotation_flag == 0)				
				  	input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]], ball_pos_y[ball_ifo.line_now-1], sign_t*ball_launch_angle[ball_temp[1]]);	
				else if(ball_ifo.disable_rotation_flag == 1)
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]], ball_pos_y[ball_ifo.line_now-1], 0);	
			}
		 	 break;
		}

	  	case BALL_STATE_BALL_ACTION:{//判断是否完成发射可以进行下一个点位取球
		  if((ball_ifo.ball_confirm % 2 == 0 && (ball_ifo.special_flag == 0)) || (ball_ifo.special_flag == 1 && ball_ifo.bt_check >= 300))//
			{
			  	ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_ANGLE_CORRECT;
			}
			else
			{
				ball_ifo.bt_check++;
				input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],sign_t*ball_launch_angle[ball_temp[1]]);					
			}
		  	break;
		}

		case BALL_STATE_ANGLE_CORRECT:{//回正角度准备下一次跑点
		  if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=50)
			{
				ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_CHECK;
			}
			else
			{
				robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//地盘移动为高精度移动
			  	ball_ifo.bt_check++;
        		input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],0);
			}
		  	break;
		}

		default:break;
	}
}


void magnet_control()
{
	static uint8_t tick = 0;
	tick++;
	if(tick<=60)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else
	{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
}
