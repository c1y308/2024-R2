#include "single_task.h"
#include "ball_task.h"
#include "stdlib.h"
void GP_Task_Single()
{
	static uint8_t seed_i = 0;
  switch(seed_ifo.seed_mode)
	{
		case seed_init_mode://�ƶ���Ԥȡ���X��λ��
    {
			DJI_motor_CAN2[0].target_angle =   GET_ANGLE_SINGLE;
			DJI_motor_CAN2[3].target_angle = - GET_ANGLE_SINGLE;
		  if(seed_ifo.gap_check >= INIT_OUT_TICK)
			{
				seed_ifo.gap_check = 0;
				seed_ifo.seed_mode = seed_init_mode_2;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
        Input_TarSpeed_Chassis(0,1,0);
				seed_ifo.gap_check++;
			}
			break;
		}
		case seed_init_mode_2:
    {
			Grap_Init_Single();//צ�ӳ�ʼ��
			all_ifo.error_tol = tol_big;//���Ե��ƶ�
			if((all_ifo.chassis_arrive == 1 && seed_ifo.gap_check>=50))
			{
				seed_ifo.gap_check = 0;
				all_ifo.chassis_arrive = 0;
				seed_ifo.seed_mode = seed_move_mode;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
				input_tarpos_chassis(sign_t*seed_pos_x_single[seed_i],SEED_FORWARD_DES,0);
				seed_ifo.gap_check++;
			}
			break;
		}
	  case seed_move_mode://�ƶ���ȡ��ĵ�λ
    {
			static uint8_t line_flag = 0;
			static uint8_t change_d_flag = 1; 
			all_ifo.error_tol = tol_small;
		  if((seed_ifo.gap_check>=50&&all_ifo.chassis_arrive==1)||seed_ifo.gap_check>=800)//
			{
				line_flag++;
				all_ifo.limit_vy_flag = 0;
				seed_ifo.cnm = 0;
				seed_i++;//ȡ����ͼ�
				seed_ifo.gap_check = 0;
				seed_ifo.seed_mode = seed_get_mode;//ץ��ģʽ
			}
			else
			{
				if(change_d_flag == 1)//��d�Ļ�ȥ
				{
				  for(int i = 0;i<2;i++)//�ֱ�ΪXYλ���⻷�����õ�����Ŀ���ٶ�
	        {
	          PID_Init(&pid_DJI_outer[i],PID_POSITION,DJI_pos_outer_para_cos,RUN_S,M3508_MOTOR_POSITION_PID_IOUT_LIMIT);
	        }
					change_d_flag = 0;
				}
				
				if(all_ifo.blue_single_flag==1)
				  DJI_motor_CAN2[0].target_angle =  GET_ANGLE_SINGLE+seed_i*6.5;//ת����ȡ��ĽǶ�
				else if(all_ifo.red_single_flag==1)
					DJI_motor_CAN2[3].target_angle = -GET_ANGLE_SINGLE-seed_i*6.5;//ת����ȡ��ĽǶ�
				
				if(seed_ifo.gap_check<=300&&seed_ifo.cnm==0)
				  input_tarpos_chassis(sign_t*seed_pos_x_single[seed_i],crack_posY,0);
				else
				{
					all_ifo.chassis_mode = mix_mode_seed;//����ƶ�ģʽ��ֱ��ײ��ȥ
	   			all_ifo.pos_target.target_pos_x = sign_t*seed_pos_x_single[seed_i];
				  all_ifo.pos_target.target_pos_y = crack_posY;
				  all_ifo.pos_target.target_pos_z = 0;
				  all_ifo.target_vy_direct = -1.6;//ֱ�Ӹ�ֵײǽY��Ŀ���ٶ�
				}
				if(line_flag%2==1)
					all_ifo.limit_vy_flag = 1;
				if(seed_ifo.cnm == 1)//��һ��ײǽ����������ֱ�Ӹ����ٶ�
				{
					all_ifo.chassis_mode = mix_mode_seed;//����ƶ�ģʽ��ֱ��ײ��ȥ
	   			all_ifo.pos_target.target_pos_x = sign_t*seed_pos_x_single[seed_i];
				  all_ifo.pos_target.target_pos_y = crack_posY;
				  all_ifo.pos_target.target_pos_z = 0;
					all_ifo.target_vy_direct = -0.3;//ֱ�Ӹ�ֵY��Ŀ���ٶ�
				}
				seed_ifo.gap_check++;
			}
			break;
		}
		case seed_get_mode://ץȡ������
    {
			static uint8_t reset_ops9_y = 1;
			static uint8_t reset_ops9_z = 1;
			static uint16_t seed_tick = 0;
		  if(seed_tick>=120)//ץȡ̧����
			{
				reset_ops9_y = 1;
				reset_ops9_z = 1;
				seed_tick = 0;
				DJI_motor_CAN2[1].target_speed = DJI_motor_CAN2[4].target_speed = 0;
				seed_ifo.seed_mode = seed_pv_mode;
			}
			else
			{
				  seed_tick++;
				  if(seed_tick<60)//צ�ӽ���ץȡ
					{
						if(all_ifo.blue_single_flag == 1)
					    DJI_motor_CAN2[2].target_speed =   GRAP_SPEED;
						else if(all_ifo.red_single_flag == 1)
						  DJI_motor_CAN2[5].target_speed = - GRAP_SPEED;
						break;
					}
					else//����̧��
					{
						if(all_ifo.blue_single_flag == 1)
					  	DJI_motor_CAN2[1].target_speed =   SPEED_LIFT_PLUS - 3;
						else if(all_ifo.red_single_flag == 1)
					  	DJI_motor_CAN2[4].target_speed = - SPEED_LIFT_PLUS + 3;
					}
				if(reset_ops9_y == 1 && seed_tick >= 60)
				{
					CD_SETX(&huart3,-250);
					reset_ops9_y = 0;
				}
				if(reset_ops9_z == 1 && seed_tick >= 90)
				{
					CD_SETZ(&huart3,0);
					reset_ops9_z = 0;
				}
				stop_chassis();
			}
			break;
		}
		case seed_pv_mode://�����ܵ�
    {
			all_ifo.error_tol = tol_small;
			static uint8_t double_move_p = 0;
		  if(seed_ifo.gap_check>=50 && all_ifo.chassis_arrive==1)
			{
				double_move_p++;
				seed_ifo.gap_check = 0;
				seed_ifo.seed_mode = seed_put_mode;
			}
			else
			{
				if(double_move_p%2==0)//��һ�ν������
				{
					input_tarpos_chassis(sign_t*put_pos_x_single[seed_i-1],put_pos_y_single[0],0);
				}
				else//�ڶ��η����ܵ�
				{
					input_tarpos_chassis(sign_t*put_pos_x_single[seed_i-1],put_pos_y_single[1],0);
				}
				if(seed_ifo.gap_check<=160)//����һ��Ԥ���ŵĶ���
				{
					DJI_motor_CAN2[1].target_speed =  2.3;
					DJI_motor_CAN2[4].target_speed = -2.3;
				}
				else
				{
				  DJI_motor_CAN2[1].target_speed =  0;
					DJI_motor_CAN2[4].target_speed =  0;
				}
				if(all_ifo.blue_single_flag==1)//ת��������ĽǶ�
				  DJI_motor_CAN2[0].target_angle =   PUT_ANGLE_SINGLE - double_move_p*8;
				else if(all_ifo.red_single_flag==1)//ת��������ĽǶ�
					DJI_motor_CAN2[3].target_angle = - PUT_ANGLE_SINGLE + double_move_p*8;
				seed_ifo.gap_check++;
			}
			break;
		}
		case seed_put_mode://���綯��
    {
		  if(seed_ifo.gap_check>=60)
			{
				DJI_motor_CAN2[1].target_speed =  2.2;
				DJI_motor_CAN2[4].target_speed = -2.2;
				if(seed_i >= 12)//ȡ���������л�������ģʽ
				{
					seed_ifo.gap_check = 0;
					seed_ifo.seed_mode = transition_f;
					all_ifo.task_type = transition;
				}
				else//û��ȡ����������л���ץȡ�ƶ�ģʽ
				{
					seed_ifo.gap_check = 0;
					DJI_motor_CAN2[2].target_speed = DJI_motor_CAN2[5].target_speed = 0;
					seed_ifo.seed_mode = seed_move_mode;
				}
				break;
			}
			else
			{
				if(all_ifo.blue_single_flag == 1)
				  DJI_motor_CAN2[2].target_speed =   GRAP_SPEED_OPEN;
				else if(all_ifo.red_single_flag == 1)
				  DJI_motor_CAN2[5].target_speed = - GRAP_SPEED_OPEN;
				seed_ifo.gap_check++;
				break;
			}
		}
		default:break;
	}
}
void Ball_Task_Single()
{
	 static uint8_t fast_launch_flag = 1;
   RUN_S  = 1.6;
	 static uint8_t useless_1 = 1;
 	 static uint8_t gpio_safe_flag = 1;
	 static uint8_t tarb = 6;//��ʼ��Ŀ����
	 DJI_motor[4].target_speed = DJI_motor[5].target_speed = M2006_GET_BALL_SPEED;
	 if(fast_launch_flag == 1)
	   DJI_motor[6].target_speed = -7.5;
	 if(gpio_safe_flag == 1)//�ٴ�ȷ�Ϲرյ����
	 {
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		 gpio_safe_flag = 0;
	 }
  switch(ball_ifo.ball_mode)
	{
		case ball_check:{
			if(tarb>5&&useless_1==1)
			  tarb++;//��һ��Ϊ7
			if(tarb == 13)//���Ҫȡ��һ�е�����
			{
				tarb = 6;
				useless_1 = 0;
			}
			else if(tarb <= 6)
				tarb--;
			ball_ifo.confirm_flag = 1;//����reversal��������һ��
			ball_ifo.protect_flag = 1;//����
			
			ball_temp[2] = ball_temp[0];//��¼�ƶ�ǰ�ĵ�ǰ�򣬼�ʹ�ƶ���Ҳ���ı䵱ǰ��
			ball_temp[1] = tarb;//װ��Ŀ����
			
			if(ball_temp[0]<=6&&ball_temp[0]!=0)//�жϵ�ǰ������
				ball_ifo.line_now = 1;
			else
				ball_ifo.line_now = 2;
			/**************************************************************/
			if(ball_temp[0]>=7&&ball_temp[1]<=6&&ball_temp[2]!=0)//��Ҫ�ӵڶ��л�����һ��ȥ
			{
				ball_temp[0] = ball_temp[1];//���µ�ǰ��
				ball_ifo.line_now = 1;//���µ�ǰ������
				ball_ifo.ball_mode = line_change_single_1;
				break;
			}
			else//����Ҫ���У���һ�ν�������Ӵ˽��룩
			{
				ball_ifo.line_lr = ball_ifo.line_fb = 0;//����Ҫ����
				ball_ifo.ball_mode = get_ball_pos_f;
				if(ball_temp[0] == 0 && ball_temp[1]>6)//��һ�ν����������Ҫǰ���ڶ���
				{
						ball_ifo.line_now = 2;
						ball_ifo.ball_mode = get_ball_pos_f;
				}
				ball_temp[0] = ball_temp[1];//���µ�ǰ������
				break;	
			}	
		}
		case line_change_single_1:{//ȡ����滻��
			if((ball_ifo.bt_check>=250))//
			{
			  ball_ifo.bt_check = 0; 
				ball_ifo.ball_mode = line_change_single_2;
			}
			else
			{
				ball_ifo.bt_check++;
				fast_launch_flag = 0;
				DJI_motor[6].target_speed = -1.2;//�л�Ϊ���ٷ���
				ball_ifo.storm_speed = ball_launch_speed[6];//��������ת��
				Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);
			  input_tarpos_chassis(sign_t*ball_pos_x[6],pos_get_special[1],0);//����ȡ���λ
			}
		  break;
		}
		case line_change_single_2:{//ȡ��������еڶ���
		  if((all_ifo.chassis_arrive == 1 && ball_ifo.bt_check >= 20)||ball_ifo.bt_check>=180)
			{
			  ball_ifo.bt_check = 0;
				fast_launch_flag = 1;//�ָ����ٷ���
				ball_ifo.ball_mode = ball_check;//׼���´�����
			}
			else
			{
			  ball_ifo.bt_check++;
				if(ball_ifo.bt_check<=130)
			    input_tarpos_chassis(sign_t*ball_pos_x[6],ball_pos_y[0],0);
				else
					input_tarpos_chassis(sign_t*ball_pos_x[6],ball_pos_y[0],sign_t*ball_launch_angle[6]);
			}
			break;
		}
		case get_ball_pos_f:{//�ȵ���Ŀ�������ǰ��(�����ƶ�)
			static uint16_t reversal = 0;
			if(((ball_ifo.bt_check>=240&&reversal==0))||(ball_ifo.bt_check>=130&&reversal!=0))
			{            //�����ƶ�����һ��ȡ����ʱ��
				ball_ifo.protect_flag = 0;
				if(ball_ifo.confirm_flag == 1)//ֻ������һ��//����ball_check��get_ball_pos_s�лָ�
				{
					reversal++;
					ball_ifo.confirm_flag = 0;
				}
				if(reversal%2 == 1)
				{
					ball_ifo.ball_mode = get_ball_pos_s;//��һ�ν����л�����ȡ��
					ball_ifo.bt_check = 0;
				}
				else if(reversal%2 == 0)
				{
				  ball_ifo.ball_mode = ball_check;//�ڶ����л�ֱ��׼��������һ����
					ball_ifo.bt_check = 0;
				}
			}
			else if(ball_ifo.protect_flag == 1)//��ֹ��ν���//����ball_check��get_ball_pos_s�лָ�
			{
				all_ifo.error_tol = tol_small;//�л�Ϊ�߾����ƶ�
				if(reversal%2 == 1)//�ڶ��ν�������λ���ƶ�
				{
					input_tarpos_chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],sign_t*ball_launch_angle[ball_temp[1]]);
					all_ifo.error_tol = tol_small;
					ball_ifo.bt_check++;	
				}
				else if(reversal%2 == 0)//��һ�ν����ʱ����Ŀ�������ǰ�����������ƶ���
				{
					all_ifo.error_tol = tol_small;//�����ƶ�Ϊ�߾����ƶ�
					Input_TarPos_Chassis(sign_t*ball_pos_x[ball_temp[1]],ball_pos_y[ball_ifo.line_now-1],0);
					
					ball_ifo.storm_speed = ball_launch_speed[ball_temp[1]];//��������ת��
					Set_PWM_Motor_Speed(&hcan1,ball_ifo.storm_speed, 0, 0, 0);
					
					ball_ifo.bt_check++;	
				}
			}
		  break;
		}
		case get_ball_pos_s:{//ȡ����
			if((all_ifo.chassis_arrive == 1 && ball_ifo.bt_check>=20)||(ball_ifo.bt_check>=140&&ball_temp[1]!=5)||(ball_ifo.bt_check>=250&&ball_temp[1]==5))
			{
			  ball_ifo.bt_check = 0;
				ball_ifo.ball_mode = get_ball_pos_f;//�ص�Ŀ�������ǰ��
			}
			else
			{
				ball_ifo.confirm_flag = 1;//�������ñ�־λ��Ҫ�ٴε�Ŀ����ǰ��
				ball_ifo.protect_flag = 1;//��������һ��
				all_ifo.error_tol = tol_small;//�����ƶ�Ϊ�߾����ƶ�
			  ball_ifo.bt_check++;
				if(ball_ifo.line_now == 1)
				{
					Input_TarPos_Chassis(sign_t*ball_pos_x[ball_temp[1]],pos_get_1,0);
				}
				else if(ball_ifo.line_now == 2)
				{
					Input_TarPos_Chassis(sign_t*ball_pos_x[ball_temp[1]],pos_get_2,0);
				}				
			}
		  break;
		}
		default:break;
	}
}
