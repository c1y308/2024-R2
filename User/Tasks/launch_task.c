#include "launch_task.h"
LaunchInfo_t launch_ifo;


void ball_init(LaunchInfo_t *launch_ifo)
{
	launch_ifo->current_ball = 0;
	launch_ifo->target_ball  = 0;
    launch_ifo->last_ball    = 0;

	launch_ifo->current_state = BALL_STATE_IDLE;
	launch_ifo->line_lr = 0;
	launch_ifo->line_fb = 0;
	launch_ifo->protect_flag_2 = 1;
}


void launch_task(LaunchInfo_t *launch_ifo, Robotifo_t *robot_ifo)
{
    /* 设置磁铁的 GPIO, 关闭磁铁 */
 	static uint8_t gpio_safe_flag = 1;
	if(gpio_safe_flag == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		gpio_safe_flag = 0;
	}

  	switch(launch_ifo->current_state)
	{
		case BALL_STATE_START_LAUNCH:{
			launch_motor_speed_normal();

			launch_ifo->confirm_flag = 1;
			launch_ifo->protect_flag = 1;

            /* 确保接受到了拾取新的目标球的指令 */
			if(launch_ifo->target_ball == 0 || launch_ifo->target_ball == launch_ifo->current_ball)
		    	break;
	   		else
      		{		
                /* 需要执行特殊换行 */
	      		if((launch_ifo->target_ball - launch_ifo->current_ball) == 6 && launch_ifo->last_ball != 0 && launch_ifo->special_switch == 0)
				{
					launch_ifo->special_switch = 1;
					// ball_temp[0] = ball_temp[2];
					launch_ifo->current_line  = FIRST_LINE;
					launch_ifo->current_state = BALL_STATE_GET_BALL_POS_SPECIAL_F;
					break;
				}
                /* 需要执行特殊换行 */
				else if((launch_ifo->target_ball - launch_ifo->current_ball) == -6 && launch_ifo->last_ball != 0 && launch_ifo->special_switch == 0)
				{
					launch_ifo->special_switch = 1;
					// ball_temp[0] = ball_temp[2];
					launch_ifo->current_line  = SECOND_LINE;
					launch_ifo->current_state = BALL_STATE_GET_BALL_POS_SPECIAL_B;
					break;
				}
                /* 执行普通换行 */
				else if(abs(launch_ifo->target_ball - launch_ifo->current_ball) != 6)
				{	
					launch_ifo->special_switch = 0;
                    /* 当前在第一行，目标球在第二行 */
					if(launch_ifo->current_ball <= 6 && launch_ifo->target_ball >= 7 && launch_ifo->last_ball != 0)
					{
                        /* 当前在第一行的左半区，目标球在第二行的左半区 */
						if(launch_ifo->current_ball <= 3 && 7 <= launch_ifo->target_ball && launch_ifo->target_ball <= 9)
							launch_ifo->line_lr = SWITCH_POS_ACT_L;
                        /* 当前在第一行的左半区，目标球在第二行的右半区 */
						else if(launch_ifo->current_ball <= 3 && launch_ifo->target_ball >= 10)
							launch_ifo->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第一行的右半区，目标球在第二行的右半区 */
						else if(4 <= launch_ifo->current_ball && launch_ifo->current_ball <= 6 && launch_ifo->target_ball >= 10)
							launch_ifo->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第一行的右半区，目标球在第二行的左半区 */
						else if(4 <= launch_ifo->current_ball && launch_ifo->current_ball <= 6 && launch_ifo->target_ball <= 9)
							launch_ifo->line_lr = SWITCH_POS_ACT_L;

                        launch_ifo->line_fb = SWITCH_POS_ACT_B;
						launch_ifo->current_line = SECOND_LINE;
						launch_ifo->current_state = BALL_STATE_LINE_CHANGE_F;
					}
                    /* 当前在第二行，目标球在第一行 */
					else if(launch_ifo->current_ball >= 7 && launch_ifo->target_ball <= 6 && launch_ifo->last_ball != 0)
					{
                        /* 当前在第二行的左半区，目标球在第一行的左半区 */
						if(launch_ifo->current_ball <= 9 && launch_ifo->target_ball <= 3)
							launch_ifo->line_lr = SWITCH_POS_ACT_L;
                        /* 当前在第二行的左半区，目标球在第一行的右半区 */
						else if(launch_ifo->current_ball <= 9 && launch_ifo->target_ball >= 4)
							launch_ifo->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第二行的右半区，目标球在第一行的右半区 */
						else if(launch_ifo->current_ball >= 10 && launch_ifo->target_ball >= 4)
							launch_ifo->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第二行的右半区，目标球在第一行的左半区 */
						else if(launch_ifo->current_ball >= 10 && launch_ifo->target_ball <= 3)
							launch_ifo->line_lr = SWITCH_POS_ACT_L;

                        launch_ifo->line_fb = SWITCH_POS_ACT_F;
						launch_ifo->current_line = FIRST_LINE;//
						launch_ifo->current_state = BALL_STATE_LINE_CHANGE_F;
					}
                    /* 不用执行换行 */
					else
					{
						launch_ifo->line_lr = launch_ifo->line_fb = 0;
						launch_ifo->current_state = BALL_STATE_MOVE_TO_BALL_POS;				
					}	
					break;
				}
				break;
		    }
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_F:{
			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 50)
			{
			  	launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_READY_TO_LAUNCH;
			}
			else
			{
				dji_motor_setref(motor3508_lift, -0.9);
				//DJI_motor[6].target_speed = -0.9;//
				launch_ifo->disable_rotation_flag = 1;
			  	launch_ifo->run_tick++;
        		if(launch_ifo->run_tick <= 100)//
          		robot_ifo->chassis_arrive = 0;	
        		launch_ifo->storm_speed = ball_launch_speed[launch_ifo->target_ball];//
				Set_PWM_Motor_Speed(&hcan1,launch_ifo->storm_speed, 0, 0, 0);					
				input_tarpos_chassis(robot_ifo, sign_t * ball_pos_x[launch_ifo->current_ball], pos_get_special[0], 0);
				break;
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_B:{//
			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 50)
			{
			  launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_READY_TO_LAUNCH;
			}
			else
			{ 
				dji_motor_setref(motor3508_lift, -0.9);
				//DJI_motor[6].target_speed = -0.9;//
				launch_ifo->disable_rotation_flag = 1;//ֹ
			  	launch_ifo->run_tick++;	
				if(launch_ifo->run_tick <= 100)//
          			robot_ifo->chassis_arrive = 0;	
        		launch_ifo->storm_speed = ball_launch_speed[launch_ifo->target_ball];//
				Set_PWM_Motor_Speed(&hcan1,launch_ifo->storm_speed, 0, 0, 0);				
        		input_tarpos_chassis(robot_ifo, sign_t * ball_pos_x[launch_ifo->current_ball], pos_get_special[1], 0);
			}
		  	break;
		}

	  	case BALL_STATE_LINE_CHANGE_F:{
			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 20)
			{
			    launch_ifo->run_tick = 0;
			    launch_ifo->current_state = BALL_STATE_LINE_CHANGE_S;
			}
			else
			{
				if(launch_ifo->line_fb == SWITCH_POS_ACT_B)
			    	robot_ifo->pos_target.pos_y = ball_pos_y[0];
				else if(launch_ifo->line_fb == SWITCH_POS_ACT_F)
					robot_ifo->pos_target.pos_y = ball_pos_y[1];
				if(launch_ifo->line_lr == SWITCH_POS_ACT_L)
				{
           			input_tarpos_chassis(robot_ifo, sign_t * change_line_x_left, robot_ifo->pos_target.pos_y, 0);
				}
				else if(launch_ifo->line_lr == SWITCH_POS_ACT_R)
				{
           			input_tarpos_chassis(robot_ifo, sign_t * change_line_x_right, robot_ifo->pos_target.pos_y, 0);
				}
			}
		 	break;
		}
		
	  	case BALL_STATE_LINE_CHANGE_S:{

			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 20)//
			{
			  launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_MOVE_TO_BALL_POS;
			}
			else
			{
			  launch_ifo->run_tick++;

				if(launch_ifo->line_lr == SWITCH_POS_ACT_L)
			    robot_ifo->pos_target.pos_x = change_line_x_left;
				else if(launch_ifo->line_lr == SWITCH_POS_ACT_R)
					robot_ifo->pos_target.pos_x = change_line_x_right;
				if(launch_ifo->line_fb == SWITCH_POS_ACT_B)//
				{
				  input_tarpos_chassis(robot_ifo, sign_t * robot_ifo->pos_target.pos_x, ball_pos_y[1], 0);
				}
				else if(launch_ifo->line_fb == SWITCH_POS_ACT_F)//
				{
				  input_tarpos_chassis(robot_ifo, sign_t * robot_ifo->pos_target.pos_x, ball_pos_y[0], 0);
				}
			}
		 	break;
		}
		/* 移动到预备取目标球的位置 */
		case BALL_STATE_MOVE_TO_BALL_POS:{
			static uint16_t reversal = 0;
			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 50)
			{
				launch_ifo->protect_flag = 0;
				if(launch_ifo->confirm_flag == 1)
				{
					reversal++;
					launch_ifo->confirm_flag = 0;
				}
                
				if(reversal % 2 == 1 && launch_ifo->ball_confirm % 2 == 1)
				{
					launch_ifo->current_state = BALL_STATE_GET_BALL_ACTION;
					launch_ifo->run_tick = 0;
				}
				else if(reversal % 2 == 0)
				{
				    launch_ifo->current_state = BALL_STATE_READY_TO_LAUNCH;
					launch_ifo->run_tick = 0;
				}
			}
			else if(launch_ifo->protect_flag == 1)
			{
				if(reversal % 2 == 1)
				{
					input_tarpos_chassis(robot_ifo,
                                         sign_t * ball_pos_x[launch_ifo->target_ball], \
                                         ball_pos_y[launch_ifo->current_line - 1],     \
                                         sign_t * ball_launch_angle[launch_ifo->target_ball]);
					robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;
					launch_ifo->run_tick++;	
				}
                /* 奇次进入此模式移动到预备取球的位置 */
				else if(reversal % 2 == 0)
				{
					robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;
					input_tarpos_chassis(robot_ifo,
                                         sign_t * ball_pos_x[launch_ifo->target_ball],
                                         ball_pos_y[launch_ifo->current_line - 1],
                                         0);

					if(robot_ifo->stop_flag != 1)//
					{
						launch_ifo->storm_speed = ball_launch_speed[launch_ifo->target_ball];
						Set_PWM_Motor_Speed(&hcan1,launch_ifo->storm_speed, 0, 0, 0);
					}
					
					launch_ifo->run_tick++;	
				}
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_ACTION:{//
			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 50)
			{
			  	launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_READY_TO_LAUNCH;//
			}
			else
			{
                /* 可以进行发射 */
				launch_ifo->confirm_flag = 1;

				launch_ifo->protect_flag = 1;
				robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;
			  	launch_ifo->run_tick++;

				if(launch_ifo->current_line == 1)
				{
					input_tarpos_chassis(robot_ifo,
                                         sign_t * ball_pos_x[launch_ifo->target_ball],
                                         pos_get_1,
                                         0);
				}
				else if(launch_ifo->current_line == 2)
				{
					input_tarpos_chassis(robot_ifo,
                                         sign_t * ball_pos_x[launch_ifo->target_ball],
                                         pos_get_2,
                                         0);
				}				
			}
		  	break;
		}

		case BALL_STATE_READY_TO_LAUNCH:{
			if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 50)
			{
				launch_ifo->disable_rotation_flag = 0;
			    launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_LAUNCH_ACTION;
			}
			else
			{
				robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;//
			  	launch_ifo->run_tick++;
        		if(launch_ifo->disable_rotation_flag == 0)				
				  	input_tarpos_chassis(robot_ifo,
                                         sign_t * ball_pos_x[launch_ifo->target_ball],
                                         ball_pos_y[launch_ifo->current_line - 1],
                                         sign_t * ball_launch_angle[launch_ifo->target_ball]);

				else if(launch_ifo->disable_rotation_flag == 1)
					input_tarpos_chassis(robot_ifo,
                                         sign_t * ball_pos_x[launch_ifo->target_ball],
                                         ball_pos_y[launch_ifo->current_line - 1],
                                         0);	
			}
		 	break;
		}

	  	case BALL_STATE_LAUNCH_ACTION:{

		  if(((launch_ifo->special_switch == 0) && (launch_ifo->ball_confirm % 2 == 0)) || (launch_ifo->special_switch == 1 && launch_ifo->run_tick >= 300))
			{
			  	launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_ANGLE_CORRECT;
			}
			else
			{
				launch_ifo->run_tick++;
				input_tarpos_chassis( robot_ifo,
                                      sign_t * ball_pos_x[launch_ifo->target_ball],
                                      ball_pos_y[launch_ifo->current_line - 1],
                                      sign_t * ball_launch_angle[launch_ifo->target_ball]);					
			}
		  	break;
		}

		case BALL_STATE_ANGLE_CORRECT:{
		  if(robot_ifo->chassis_arrive == 1 && launch_ifo->run_tick >= 50)
			{
				launch_ifo->run_tick = 0;
				launch_ifo->current_state = BALL_STATE_START_LAUNCH;
			}
			else
			{
                
				robot_ifo->tol_state = CHASSIS_MODE_TOL_SMALL;
        		input_tarpos_chassis(robot_ifo,
                                     sign_t * ball_pos_x[launch_ifo->target_ball],
                                     ball_pos_y[launch_ifo->current_line - 1],
                                     0);

                /* 更新信息 */
                launch_ifo->last_ball    = launch_ifo->current_ball;
                launch_ifo->current_ball = launch_ifo->target_ball;

				if(launch_ifo->current_ball <= 6)
					launch_ifo->current_line = FIRST_LINE;
				else
					launch_ifo->current_line = SECOND_LINE;


                launch_ifo->run_tick++;
			}
		  	break;
		}
		default:break;
	}
}