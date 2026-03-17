#include "launch_task.h"
LaunchInfo_t launch_info;
static uint8_t move_state_enter_count = 0;

void ball_init(LaunchInfo_t *launch_info)
{
	launch_info->current_ball = 0;
	launch_info->target_ball  = 0;
    launch_info->last_ball    = 0;

	launch_info->current_state = BALL_STATE_IDLE;
	launch_info->line_lr = 0;
	launch_info->line_fb = 0;
}


static LaunchConfirm_e get_launch_confirm_state(LaunchInfo_t *launch_info)
{
    if(launch_info->confirm_2_launch % 2 == 0)
        return LAUNCH_CONFIRM_WAIT;
    else
        return LAUNCH_CONFIRM_READY;
}


void launch_task(LaunchInfo_t *launch_info, ChassisInfo_t *robot_info)
{
    /* 设置磁铁的 GPIO, 关闭磁铁 */
 	static uint8_t gpio_safe_flag = 1;
	if(gpio_safe_flag == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		gpio_safe_flag = 0;
	}

  	switch(launch_info->current_state)
	{
		case BALL_STATE_START_LAUNCH:{
			launch_motor_speed_normal();

            /* 确保接受到了拾取新的目标球的指令 */
			if(launch_info->target_ball == 0 || launch_info->target_ball == launch_info->current_ball)
		    	break;
	   		else
      		{		
                /* 需要执行特殊换行 */
	      		if((launch_info->target_ball - launch_info->current_ball) == 6 && launch_info->last_ball != 0 && launch_info->special_switch == 0)
				{
					launch_info->special_switch = true;
					launch_info->current_line  = FIRST_LINE;
					launch_info->current_state = BALL_STATE_GET_BALL_POS_SPECIAL_F;
					break;
				}
                /* 需要执行特殊换行 */
				else if((launch_info->target_ball - launch_info->current_ball) == -6 && launch_info->last_ball != 0 && launch_info->special_switch == 0)
				{
					launch_info->special_switch = true;
					launch_info->current_line  = SECOND_LINE;
					launch_info->current_state = BALL_STATE_GET_BALL_POS_SPECIAL_B;
					break;
				}
                /* 执行普通换行 */
				else if(abs(launch_info->target_ball - launch_info->current_ball) != 6)
				{	
					launch_info->special_switch = false;

                    /* 当前在第一行，目标球在第二行 */
					if(launch_info->current_ball <= 6 && launch_info->target_ball >= 7 && launch_info->last_ball != 0)
					{
                        /* 当前在第一行的左半区，目标球在第二行的左半区 */
						if(launch_info->current_ball <= 3 && 7 <= launch_info->target_ball && launch_info->target_ball <= 9)
							launch_info->line_lr = SWITCH_POS_ACT_L;
                        /* 当前在第一行的左半区，目标球在第二行的右半区 */
						else if(launch_info->current_ball <= 3 && launch_info->target_ball >= 10)
							launch_info->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第一行的右半区，目标球在第二行的右半区 */
						else if(4 <= launch_info->current_ball && launch_info->current_ball <= 6 && launch_info->target_ball >= 10)
							launch_info->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第一行的右半区，目标球在第二行的左半区 */
						else if(4 <= launch_info->current_ball && launch_info->current_ball <= 6 && launch_info->target_ball <= 9)
							launch_info->line_lr = SWITCH_POS_ACT_L;

                        launch_info->line_fb = SWITCH_POS_ACT_B;
						launch_info->current_line = SECOND_LINE;
						launch_info->current_state = BALL_STATE_LINE_CHANGE_F;
					}
                    /* 当前在第二行，目标球在第一行 */
					else if(launch_info->current_ball >= 7 && launch_info->target_ball <= 6 && launch_info->last_ball != 0)
					{
                        /* 当前在第二行的左半区，目标球在第一行的左半区 */
						if(launch_info->current_ball <= 9 && launch_info->target_ball <= 3)
							launch_info->line_lr = SWITCH_POS_ACT_L;
                        /* 当前在第二行的左半区，目标球在第一行的右半区 */
						else if(launch_info->current_ball <= 9 && launch_info->target_ball >= 4)
							launch_info->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第二行的右半区，目标球在第一行的右半区 */
						else if(launch_info->current_ball >= 10 && launch_info->target_ball >= 4)
							launch_info->line_lr = SWITCH_POS_ACT_R;
                        /* 当前在第二行的右半区，目标球在第一行的左半区 */
						else if(launch_info->current_ball >= 10 && launch_info->target_ball <= 3)
							launch_info->line_lr = SWITCH_POS_ACT_L;

                        launch_info->line_fb = SWITCH_POS_ACT_F;
						launch_info->current_line = FIRST_LINE;//
						launch_info->current_state = BALL_STATE_LINE_CHANGE_F;
					}
                    /* 不用换行 */
					else
					{
						launch_info->line_lr = launch_info->line_fb = 0;
						move_state_enter_count++;
						launch_info->current_state = BALL_STATE_MOVE_TO_BALL_POS;				
					}	
				}
				break;
		    }
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_F:{
			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 50)
			{
			  	launch_info->run_tick = 0;
				launch_info->current_state = BALL_STATE_READY_TO_LAUNCH;
			}
			else
			{
				launch_info->disable_rotation_flag = true;

        		if(launch_info->run_tick <= 100)
          			robot_info->chassis_arrive = 0;	

        		launch_info->storm_speed = ball_launch_speed[launch_info->target_ball];
				launch_motor_speed_change(launch_info->storm_speed);

				input_tarpos_chassis(robot_info, sign_t * ball_pos_x[launch_info->current_ball], pos_get_special[0], 0);

				launch_info->run_tick++;
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_POS_SPECIAL_B:{
			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 50)
			{
			  	launch_info->run_tick = 0;
				launch_info->current_state = BALL_STATE_READY_TO_LAUNCH;
			}
			else
			{ 
				launch_info->disable_rotation_flag = false;	
				
				if(launch_info->run_tick <= 100)
          			robot_info->chassis_arrive = 0;	

        		launch_info->storm_speed = ball_launch_speed[launch_info->target_ball];
				set_pwm_motor_speed(&hcan1,launch_info->storm_speed, 0, 0, 0);	

        		input_tarpos_chassis(robot_info, sign_t * ball_pos_x[launch_info->current_ball], pos_get_special[1], 0);

				launch_info->run_tick++;
			}
		  	break;
		}

	  	case BALL_STATE_LINE_CHANGE_F:{
			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 20)
			{
			    launch_info->run_tick = 0;
			    launch_info->current_state = BALL_STATE_LINE_CHANGE_S;
			}
			else
			{
				if(launch_info->line_fb == SWITCH_POS_ACT_B)
			    	robot_info->pos_target.pos_y = ball_pos_y[0];
				else if(launch_info->line_fb == SWITCH_POS_ACT_F)
					robot_info->pos_target.pos_y = ball_pos_y[1];

				if(launch_info->line_lr == SWITCH_POS_ACT_L)
				{
           			input_tarpos_chassis(robot_info, sign_t * change_line_x_left, robot_info->pos_target.pos_y, 0);
				}
				else if(launch_info->line_lr == SWITCH_POS_ACT_R)
				{
           			input_tarpos_chassis(robot_info, sign_t * change_line_x_right, robot_info->pos_target.pos_y, 0);
				}

				launch_info->run_tick++;
			}
		 	break;
		}
		
	  	case BALL_STATE_LINE_CHANGE_S:{

			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 20)//
			{
			  	launch_info->run_tick = 0;
				move_state_enter_count++;
				launch_info->current_state = BALL_STATE_MOVE_TO_BALL_POS;
			}
			else
			{
				if(launch_info->line_lr == SWITCH_POS_ACT_L)
			    	robot_info->pos_target.pos_x = change_line_x_left;
				else if(launch_info->line_lr == SWITCH_POS_ACT_R)
					robot_info->pos_target.pos_x = change_line_x_right;

				if(launch_info->line_fb == SWITCH_POS_ACT_B)//
				{
				  input_tarpos_chassis(robot_info, sign_t * robot_info->pos_target.pos_x, ball_pos_y[1], 0);
				}
				else if(launch_info->line_fb == SWITCH_POS_ACT_F)//
				{
				  input_tarpos_chassis(robot_info, sign_t * robot_info->pos_target.pos_x, ball_pos_y[0], 0);
				}

				launch_info->run_tick++;
			}
		 	break;
		}
		/* 移动到预备取目标球的位置 */
		case BALL_STATE_MOVE_TO_BALL_POS:{
			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 50)
			{   
				/* 奇数次进入需要切换到取球状态执行取球操作 */          
				if(move_state_enter_count % 2 == 1)
				{
				    launch_info->current_state = BALL_STATE_GET_BALL_ACTION;
					launch_info->run_tick = 0;
				}
				/* 偶数次进入需要切换到发射状态准备发射 */
				else if(move_state_enter_count % 2 == 0 && get_launch_confirm_state(launch_info) == LAUNCH_CONFIRM_READY)
				{
					launch_info->current_state = BALL_STATE_READY_TO_LAUNCH;
					launch_info->run_tick = 0;
				}
			}
			else
			{
                /* 奇次进入此模式移动到预备取球的位置 */
				if(move_state_enter_count % 2 == 1)
				{
					robot_info->tol_state = CHASSIS_MODE_TOL_SMALL;
					input_tarpos_chassis(robot_info,										\
                                         sign_t * ball_pos_x[launch_info->target_ball],	\
                                         ball_pos_y[launch_info->current_line - 1],		\
                                         0);

					launch_info->storm_speed = ball_launch_speed[launch_info->target_ball];
					launch_motor_speed_change(launch_info->storm_speed);
					
					launch_info->run_tick++;	
				}
				else if(move_state_enter_count % 2 == 0)  // 第二次返回到预备取球位置就是要准备发射了
				{
					robot_info->tol_state = CHASSIS_MODE_TOL_SMALL;
					input_tarpos_chassis(robot_info,										\
                                         sign_t * ball_pos_x[launch_info->target_ball],  \
                                         ball_pos_y[launch_info->current_line - 1],      \
                                         sign_t * ball_launch_angle[launch_info->target_ball]);

					launch_info->run_tick++;	
				}
			}
		  	break;
		}

		case BALL_STATE_GET_BALL_ACTION:{
			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 50)
			{
			  	launch_info->run_tick = 0;
				move_state_enter_count++;
				launch_info->current_state = BALL_STATE_MOVE_TO_BALL_POS;  // 拾取完球回到预备取球位置
			}
			else
			{
				robot_info->tol_state = CHASSIS_MODE_TOL_SMALL;

				if(launch_info->current_line == 1)
				{
					input_tarpos_chassis(robot_info,
                                         sign_t * ball_pos_x[launch_info->target_ball],
                                         pos_get_1,
                                         0);
				}
				else if(launch_info->current_line == 2)
				{
					input_tarpos_chassis(robot_info,
                                         sign_t * ball_pos_x[launch_info->target_ball],
                                         pos_get_2,
                                         0);
				}	
				launch_info->run_tick++;			
			}
		  	break;
		}

		case BALL_STATE_READY_TO_LAUNCH:{
			if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 50)
			{
				launch_info->disable_rotation_flag = false;
			    launch_info->run_tick = 0;
				launch_info->current_state = BALL_STATE_LAUNCH_ACTION;
			}
			else
			{
				robot_info->tol_state = CHASSIS_MODE_TOL_SMALL;

        		if(launch_info->disable_rotation_flag == false)				
				  	input_tarpos_chassis(robot_info,
                                         sign_t * ball_pos_x[launch_info->target_ball],
                                         ball_pos_y[launch_info->current_line - 1],
                                         sign_t * ball_launch_angle[launch_info->target_ball]);
				/* 特殊取球模式不需要进行旋转 */
				else if(launch_info->disable_rotation_flag == true)
					input_tarpos_chassis(robot_info,
                                         sign_t * ball_pos_x[launch_info->target_ball],
                                         ball_pos_y[launch_info->current_line - 1],
                                         0);	

				launch_info->run_tick++;
			}
		 	break;
		}

	  	case BALL_STATE_LAUNCH_ACTION:{

		  if(   ((launch_info->special_switch == false) && (get_launch_confirm_state(launch_info) == LAUNCH_CONFIRM_WAIT))
		  	 ||  (launch_info->special_switch == true   && (launch_info->run_tick >= 300)) )
			{
			  	launch_info->run_tick = 0;
				launch_info->current_state = BALL_STATE_ANGLE_CORRECT;
			}
			else
			{
				launch_info->run_tick++;
				input_tarpos_chassis( robot_info,
                                      sign_t * ball_pos_x[launch_info->target_ball],
                                      ball_pos_y[launch_info->current_line - 1],
                                      sign_t * ball_launch_angle[launch_info->target_ball]);					
			}
		  	break;
		}

		case BALL_STATE_ANGLE_CORRECT:{
		  if(robot_info->chassis_arrive == 1 && launch_info->run_tick >= 50)
			{
				launch_info->run_tick = 0;
				launch_info->current_state = BALL_STATE_START_LAUNCH;
			}
			else
			{
                
				robot_info->tol_state = CHASSIS_MODE_TOL_SMALL;
        		input_tarpos_chassis(robot_info,
                                     sign_t * ball_pos_x[launch_info->target_ball],
                                     ball_pos_y[launch_info->current_line - 1],
                                     0);

                /* 更新信息 */
                launch_info->last_ball    = launch_info->current_ball;
                launch_info->current_ball = launch_info->target_ball;

				if(launch_info->current_ball <= 6)
					launch_info->current_line = FIRST_LINE;
				else
					launch_info->current_line = SECOND_LINE;

                launch_info->run_tick++;
			}
		  	break;
		}
		default:break;
	}
}