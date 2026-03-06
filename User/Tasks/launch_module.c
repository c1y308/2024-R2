#include "launch_module.h"
#include "stdlib.h"
#include "can.h"
#include "user_config.h"
BallInfo_t ball_ifo;
int8_t ball_temp[3];

DjiMotorHandle_t *motor2006_front, *motor2006_back, *motor3508_lift;
void seed_motor_init()
{
    MotorInitConfig_t chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
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
            .close_loop_type = SPEED_LOOP,
			.motor_direction = MOTOR_DIRECTION_REVERSE,
        },
		.motor_id = 0,
        .motor_type = DJI_MOTOR_2006,
    };

    chassis_motor_config.motor_id = 4;
    motor2006_front = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 5;
    motor2006_back = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 6;
    motor3508_lift = djimotor_init(&chassis_motor_config);
}


void ball_init()
{
	ball_ifo.now_ball = 0;
	ball_ifo.target_ball = 0;
	ball_ifo.current_state = BALL_STATE_CHECK;
	ball_temp[0] = 0;
	ball_ifo.line_lr = 0;
	ball_ifo.line_fb = 0;
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


			ball_ifo.confirm_flag = 1;//
			ball_ifo.protect_flag = 1;//
			if(ball_ifo.target_ball == 0 || ball_ifo.target_ball == ball_temp[0])//
		    	break;
	   		else
      		{
				ball_temp[2] = ball_temp[0];  //
				ball_temp[1] = ball_ifo.target_ball;  //
				
				if(ball_temp[0] <= 6)
					ball_ifo.line_now = 1;
				else
					ball_ifo.line_now = 2;
				
				/**************************************************************/
	      		if((ball_temp[1] - ball_temp[0]) == 6 && ball_temp[2] != 0 && ball_ifo.special_flag == 0)//
				{
					ball_ifo.special_flag = 1;//
					ball_temp[0] = ball_temp[2];//
					ball_ifo.line_now = 1;
					ball_ifo.current_state = BALL_STATE_GET_BALL_POS_SPECIAL_F;
					break;
				}
				else if((ball_temp[1] - ball_temp[0]) == -6 &&ball_temp[2]!= 0 && ball_ifo.special_flag == 0)
				{
					ball_ifo.special_flag = 1;//
					ball_temp[0] = ball_temp[2];//
					ball_ifo.line_now = 2;
					ball_ifo.current_state = BALL_STATE_GET_BALL_POS_SPECIAL_B;
					break;
				}
				else if(abs(ball_temp[1] - ball_temp[0]) != 6)
				{	
					ball_ifo.special_flag = 0;//ȡȡģʽ
					if(ball_temp[0] <= 6 && ball_temp[1] >= 7 && ball_temp[2] != 0)//
					{
						ball_ifo.line_fb = BALL_FORWARD;
						if(ball_temp[0] <= 3 && 7 <= ball_temp[1] && ball_temp[1] <= 9)//
							ball_ifo.line_lr = BALL_LEFT;
						else if(ball_temp[0] <= 3 && ball_temp[1] >= 10)//
							ball_ifo.line_lr = BALL_RIGHT;
						else if(4 <= ball_temp[0] && ball_temp[0] <= 6 && ball_temp[1] >= 10)//
							ball_ifo.line_lr = BALL_RIGHT;
						else if(4 <= ball_temp[0] && ball_temp[0] <= 6 && ball_temp[1] <= 9)//
							ball_ifo.line_lr = BALL_LEFT;
						ball_ifo.line_now = 2;//Ԥµǰ
						ball_ifo.current_state = BALL_STATE_LINE_CHANGE_F;
					}
		
					else if(ball_temp[0]>=7&&ball_temp[1]<=6&&ball_temp[2]!=0)//
					{
						ball_ifo.line_fb = BALL_BACK;
						if(ball_temp[0]<=9&&ball_temp[1]<=3)//
							ball_ifo.line_lr = BALL_LEFT;
				
						else if(ball_temp[0]<=9&&ball_temp[1]>=4)//
							ball_ifo.line_lr = BALL_RIGHT;
				
						else if(ball_temp[0]>=10 && ball_temp[1]>=4)//
							ball_ifo.line_lr = BALL_RIGHT;
			
						else if(ball_temp[0]>=10 && ball_temp[1]<=3)//
							ball_ifo.line_lr = BALL_LEFT;
						ball_ifo.line_now = 1;//
						ball_ifo.current_state = BALL_STATE_LINE_CHANGE_F;
					}
					else//
					{
						ball_ifo.line_lr = ball_ifo.line_fb = 0;//
						ball_ifo.current_state = BALL_STATE_GET_BALL_POS_F;
						if(ball_temp[0] == 0 && ball_temp[1]>6)//
						{
								ball_ifo.line_now = 2;
//								ball_ifo.line_fb = BALL_FORWARD;
//								ball_ifo.line_lr = BALL_LEFT;
//								ball_ifo.current_state = line_change_f;
						}
						
					}	
					ball_temp[0] = ball_temp[1];//
					break;
				}
				break;
		    }
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_F:{
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=50)
			{
			  	ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_T;
			}
			else
			{
				dji_motor_setref(motor3508_lift, -0.9);
				//DJI_motor[6].target_speed = -0.9;//
				ball_ifo.disable_rotation_flag = 1;
			  	ball_ifo.bt_check++;
        		if(ball_ifo.bt_check <= 100)//
          		robot_ifo.chassis_arrive = 0;	
        		ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//
				Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);					
				input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[0]], pos_get_special[0], 0);
				break;
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_B:{//
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=50)
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_T;
			}
			else
			{ 
				dji_motor_setref(motor3508_lift, -0.9);
				//DJI_motor[6].target_speed = -0.9;//
				ball_ifo.disable_rotation_flag = 1;//ֹ
			  	ball_ifo.bt_check++;	
				if(ball_ifo.bt_check <= 100)//
          			robot_ifo.chassis_arrive=0;	
        		ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//
				Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);				
        		input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[0]],pos_get_special[1],0);
			}
		  	break;
		}

	  	case BALL_STATE_LINE_CHANGE_F:{//
			if(robot_ifo.chassis_arrive==1 && ball_ifo.bt_check>=20)
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_LINE_CHANGE_S;
			}
			else
			{
				ball_ifo.bt_check++;
				if(ball_ifo.line_fb == BALL_FORWARD)//
			    	robot_ifo.pos_target.pos_y = ball_pos_y[0];//
				else if(ball_ifo.line_fb == BALL_BACK)//
					robot_ifo.pos_target.pos_y = ball_pos_y[1];//
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
		
	  	case BALL_STATE_LINE_CHANGE_S:{//

			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 20)//
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_F;
			}
			else
			{
			  ball_ifo.bt_check++;//
				/*************************************************/
//				robot_ifo.chassis_mode = mix_mode_y;		
//			  if(ball_ifo.line_fb == BALL_FORWARD)//Ҫǰڶ
//				{
//				  robot_ifo.target_vy_direct =  1.4;
//				}
//				else if(ball_ifo.line_fb == BALL_BACK)//Ҫǰһ
//				{
//				  robot_ifo.target_vy_direct = -1.4;
//				}
        /*****************************************************/
				if(ball_ifo.line_lr == BALL_LEFT)
			    robot_ifo.pos_target.pos_x = change_line_x_left;
				else if(ball_ifo.line_lr == BALL_RIGHT)
					robot_ifo.pos_target.pos_x = change_line_x_right;
				if(ball_ifo.line_fb == BALL_FORWARD)//
				{
				  input_tarpos_chassis(sign_t * robot_ifo.pos_target.pos_x, ball_pos_y[1], 0);
				}
				else if(ball_ifo.line_fb == BALL_BACK)//
				{
				  input_tarpos_chassis(sign_t * robot_ifo.pos_target.pos_x, ball_pos_y[0], 0);
				}
				/*******************************************************************/
			}
		 	break;
		}
		
		case BALL_STATE_GET_BALL_POS_F:{//
			static uint16_t reversal = 0;
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 50)
			{
				ball_ifo.protect_flag = 0;
				if(ball_ifo.confirm_flag == 1)//ֻ
				{
					reversal++;
					ball_ifo.confirm_flag = 0;
				}
				if(reversal % 2 == 1 && ball_ifo.ball_confirm % 2 != 0)//
				{
					ball_ifo.current_state = BALL_STATE_GET_BALL_POS_S;//
					ball_ifo.bt_check = 0;
				}
				else if(reversal % 2 == 0)
				{
				  ball_ifo.current_state = BALL_STATE_GET_BALL_POS_T;//
					ball_ifo.bt_check = 0;
				}
			}
			else if(ball_ifo.protect_flag == 1)//ֹ
			{
				if(reversal % 2 == 1)//
				{
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],sign_t*ball_launch_angle[ball_temp[1]]);
					robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;
					ball_ifo.bt_check++;	
				}
				else if(reversal % 2 == 0)//
				{
					robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],0);
					if(robot_ifo.stop_flag != 1)//
					{
						ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//
						Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);
					}
					
					ball_ifo.bt_check++;	
				}
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_S:{//
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 50)
			{
			  	ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_GET_BALL_POS_F;//
			}
			else
			{
				ball_ifo.confirm_flag = 1;//
				ball_ifo.protect_flag = 1;
				robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//
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

		case BALL_STATE_GET_BALL_POS_T:{//
			if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 50)
			{
				ball_ifo.disable_rotation_flag = 0;
			  ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_BALL_ACTION;
			}
			else
			{
				robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//
			  	ball_ifo.bt_check++;
        		if(ball_ifo.disable_rotation_flag == 0)				
				  	input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]], ball_pos_y[ball_ifo.line_now-1], sign_t*ball_launch_angle[ball_temp[1]]);	
				else if(ball_ifo.disable_rotation_flag == 1)
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]], ball_pos_y[ball_ifo.line_now-1], 0);	
			}
		 	 break;
		}

	  	case BALL_STATE_BALL_ACTION:{//
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

		case BALL_STATE_ANGLE_CORRECT:{//
		  if(robot_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=50)
			{
				ball_ifo.bt_check = 0;
				ball_ifo.current_state = BALL_STATE_CHECK;
			}
			else
			{
				robot_ifo.tol_state = CHASSIS_MODE_TOL_SMALL;//
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
	if(tick <= 60)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else
	{
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
}
