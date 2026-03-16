#include "chassis_module.h"
#include "Code_Disc.h"
#include "NRF24L01.h"
#include "dji_motor.h"
float controller_vx;
float controller_vy;
float controller_wz;

Robotinfo_t robot_info;

int8_t sign_t = 1;


static DjiMotorHandle_t *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back


/*****************定义位置外环的 PID 参数************/
PIDInitConfig_t chassis_pos_para = {
    .Kp = 0.0032,
    .Ki = 0.0021,
    .Kd = 0.27,
	.IntegralLimit = 0.08,
	.MaxOut = 2.0f
};


PIDInitConfig_t chassis_pos_para_w = {
    .Kp = 0.08,
    .Ki = 0.002,
    .Kd = 1.05,
	.IntegralLimit = 0.02,
	.MaxOut = 1.0f
};


PID_t pid_DJI_outer[3];  // x，y，z speed as outer loop
/**************************初始化PID结构体****************************/
void chassis_PID_init_outer()
{
	 for(int i = 0; i < 2; i++)//分别为XY位置外环，求解得到地盘目标速度
	 {
	    PID_init(&pid_DJI_outer[i], chassis_pos_para);
	 }
	 //初始化Z轴位置外环
	 PID_init(&pid_DJI_outer[2], chassis_pos_para_w);
}


void chassis_motor_init()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    MotorInitConfig_t chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_init_config = {
            .speed_PID = {
                .Kp = M3508_MOTOR_SPEED_PID_KP,
                .Ki = M3508_MOTOR_SPEED_PID_KI,
                .Kd = M3508_MOTOR_SPEED_PID_KD,
                .IntegralLimit = M3508_MOTOR_SPEED_PID_IOUT_LIMIT,
                .MaxOut = M3508_MOTOR_SPEED_PID_POUT_LIMIT,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .MaxOut = 15000,
            },
			.outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
			.motor_direction = MOTOR_DIRECTION_REVERSE,
        },
		.motor_id = 0,
        .motor_type = DJI_MOTOR_3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.motor_id = 0;
    motor_lf = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 1;
    motor_rf = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 2;
    motor_lb = djimotor_init(&chassis_motor_config);

    chassis_motor_config.motor_id = 3;
    motor_rb = djimotor_init(&chassis_motor_config);

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd",  sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}


void All_Init()
{
	PID_Init_All();
	Filter_Init_All();
	CAN_Config();
	Seed_Init();
	Ball_Init();
	
	// sign_t = 1;
	
	// grap_info.grap_arrive = 0;
	// grap_info.start_storage = 0;
	// grap_info.start_storage_out = 0;

	// robot_info.protect_flag = 1;
	// robot_info.disable_rotation_flag = 0;
	
	robot_info.sac = 0;
	robot_info.limit_vy_flag = 0;
	robot_info.red_single_flag = robot_info.blue_single_flag = 0;
	robot_info.init_tick = 0;


	robot_info.task_type = TASK_TYPE_CHASSIS_INIT;
  	robot_info.chassis_state = CHASSIS_MODE_AUTO;
	robot_info.tol_state = CHASSIS_MODE_TOL_SMALL;
	// UART_DMA_Recevie_init(&huart1,buffer_receve_1,100); 
	// UART_DMA_Recevie_init(&huart2,buffer_receve_2,100);
	// UART_DMA_Recevie_init(&huart3,buffer_receve_3,100);

}


void Chassis_Callback()
{
	chassis_feedback_update(&robot_info);
	// Calc_Send_Task();
}


/* 底盘反馈更新:获取当前底盘速度和位置，更新位置/角度的误差信息 */
void chassis_feedback_update(Robotinfo_t *robot_info)
{

	robot_info->speed_now.vx = (motor_lf->measure.speed_rpm + motor_lb->measure.speed_rpm) / 2;
	robot_info->speed_now.vy = (motor_lf->measure.speed_rpm - motor_lb->measure.speed_rpm) / 2;
	robot_info->speed_now.wz = Code_Disc_robot.pal_yaw_rad;
	
	robot_info->pos_now.pos_x = - Code_Disc_robot.x;
	robot_info->pos_now.pos_y = - Code_Disc_robot.y;
	robot_info->pos_now.pos_z =   Code_Disc_robot.yaw_rad * RADIAN_TO_ANGLE;	


	robot_info->pos_error.ErrorposX_temp = robot_info->pos_now.pos_x - robot_info->pos_target.pos_x;
	robot_info->pos_error.ErrorposY_temp = robot_info->pos_now.pos_y - robot_info->pos_target.pos_y;
	
	robot_info->pos_error.ErrorposX = cos(  robot_info->pos_now.pos_z * ANG2RAD) * \
	robot_info->pos_error.ErrorposX_temp - sin(- robot_info->pos_now.pos_z * ANG2RAD) * robot_info->pos_error.ErrorposY_temp;

	robot_info->pos_error.ErrorposY = sin(- robot_info->pos_now.pos_z * ANG2RAD) * \
	robot_info->pos_error.ErrorposX_temp + cos(  robot_info->pos_now.pos_z * ANG2RAD) * robot_info->pos_error.ErrorposY_temp;

	robot_info->pos_error.ErrorposZ = robot_info->pos_now.pos_z - robot_info->pos_target.pos_z;
}


bool chassis_arrive_check(Robotinfo_t *robot_info){
	static int16_t chassis_tol = 0;
	static int16_t check_tick = 0;
	static int16_t cur_tick = 0;

	static int16_t crack_time = 0;
	/* 检查底盘是否到达目标位置的误差容忍度 */
	switch (robot_info->tol_state)
	{
		case CHASSIS_MODE_TOL_BIG:{
			check_tick  = CHECK_TICK_SMALL;
			chassis_tol = CHASSIS_TOL_BIG;
			break;
		}
		case CHASSIS_MODE_TOL_SMALL:{
			check_tick  = CHECK_TICK_BIG;
			chassis_tol = CHASSIS_TOL_SMALL;
			break;
		}
		default:
			break;
	}

	/* 检查地盘控制模式，全自动/半自动/手动 */
	switch (robot_info->chassis_state)
	{
		case CHASSIS_MODE_AUTO:{
			if(fabs(robot_info->pos_error.ErrorposX) <= chassis_tol && fabs(robot_info->pos_error.ErrorposY) <= chassis_tol && \
			   fabs(robot_info->pos_error.ErrorposZ) <= 1 )
			{   
				cur_tick++;
				if(cur_tick >= check_tick)
				{
					cur_tick = 0;
					robot_info->chassis_arrive = 1;
					return true;
				}
			}else{
				robot_info->chassis_arrive = 0;
				return false;
			}
			break;
		}
		case CHASSIS_MODE_MIX_SEED:{
			if(fabs(robot_info->speed_now.vy) <= 0.001 && fabs(robot_info->pos_error.ErrorposY) <= 50)
			{
				crack_time++;
				if(crack_time >= 20)
			  		robot_info->stop_crack = 1;
			}
			else
				crack_time = 0;

			if(fabs(robot_info->pos_error.ErrorposX) <= chassis_tol && robot_info->stop_crack == 1)
			{   
				robot_info->speed_target.vx = robot_info->speed_target.wz = 0;
				cur_tick++;
				if(cur_tick >= check_tick)
				{
					robot_info->stop_crack = 0;
					cur_tick = 0;
					robot_info->chassis_arrive = 1;
					return true;
				}
			}else{
				robot_info->chassis_arrive = 0;
				return false;
			}
			break;
		}
		case CHASSIS_HYBRID_XS:
		{
			if(fabs(robot_info->pos_error.ErrorposY) <= chassis_tol && fabs(robot_info->pos_error.ErrorposZ) <= 1)
			{   
				cur_tick++;
				if(cur_tick >= check_tick)
				{
					cur_tick = 0;
					robot_info->chassis_arrive = 1;
					return true;
				}
			}else{
				robot_info->chassis_arrive = 0;
				return false;
			}
			break;
		}
		case CHASSIS_HYBRID_YS:
		{
			if(fabs(robot_info->pos_error.ErrorposX) <= chassis_tol && fabs(robot_info->pos_error.ErrorposZ) <= 1)
			{   
				cur_tick++;
				if(cur_tick >= check_tick)
				{
					cur_tick = 0;
					robot_info->chassis_arrive = 1;
					return true;
				}
			}else{
				robot_info->chassis_arrive = 0;
				return false;
			}
			break;
		}
		case CHASSIS_HYBRID_XSYS:
		{
			if(fabs(robot_info->pos_error.ErrorposZ) <= 1)
			{   
				cur_tick++;
				if(cur_tick >= check_tick)
				{
					cur_tick = 0;
					robot_info->chassis_arrive = 1;
					return true;
				}
			}else{
				robot_info->chassis_arrive = 0;
				return false;
			}
			break;
		}
		
		default:{
			robot_info->chassis_arrive = 1;
			return true;
			break;
		}
	}
	cur_tick = 0;
}


/* 目的是得到地盘的目标速度 */
void chassis_calc_tarspeed_task(Robotinfo_t *robot_info)
{
	static float vy_limit = 2;
	switch(robot_info->chassis_state)
	{
		case CHASSIS_MODE_AUTO:
		{
			chassis_pos_calc(&robot_info);
			// 此处可以进行优化，进行解耦，当接近目标点时，单独进行一个减速的函数，或者在位置环中加入一个根据误差自动调整输出的函数 
			if(fabs(robot_info->pos_error.ErrorposY) < LOW_DES_COM && seed_info.seed_state == SEED_STATE_MOVE_2_GET)  // &&seed_info.cnm == 0
			{
				 vy_limit = vy_limit - 1.45;
				 if(vy_limit <= LOW_SPEED)
					 vy_limit = LOW_SPEED;
				 abs_limit(&robot_info->speed_target.vy,vy_limit);
			}
			break;
		}
		case CHASSIS_MODE_MANUAL:
		{
			robot_info->speed_target.vx = robot_info->speed_target.target_vx_direct;
			robot_info->speed_target.vy = robot_info->speed_target.target_vy_direct;
			robot_info->speed_target.wz = robot_info->speed_target.target_wz_direct;

			robot_info->chassis_state = CHASSIS_MODE_AUTO;
		}
		case CHASSIS_HYBRID_XS:
		{
			chassis_pos_calc(robot_info);
			robot_info->speed_target.vx = robot_info->speed_target.target_vx_direct;
		}
		case CHASSIS_HYBRID_YS:
		{
			chassis_pos_calc(robot_info);
			robot_info->speed_target.vy = robot_info->speed_target.target_vy_direct;
		}
		case CHASSIS_HYBRID_XSYS:
		{
			chassis_pos_calc(robot_info);
			robot_info->speed_target.vx = robot_info->speed_target.target_vx_direct;
			robot_info->speed_target.vy = robot_info->speed_target.target_vy_direct;
		}
		case CHASSIS_MODE_MIX_SEED:{
			chassis_pos_calc(robot_info);
			if(robot_info->stop_crack == 1)
				robot_info->speed_target.vy = 0;
			else
			{
				robot_info->speed_target.vy = robot_info->speed_target.target_vy_direct;

				if(fabs(robot_info->pos_error.ErrorposY) < LOW_DES_COM &&         \
						robot_info->pos_target.pos_y == crack_posY)  // && seed_info.cnm == 0
				{
          			vy_limit = vy_limit - 1.45;
					if(vy_limit <= LOW_SPEED)
						 vy_limit = LOW_SPEED;
					 abs_limit(&robot_info->speed_target.vy,vy_limit);
				}
				else
          			vy_limit = RUN_S;
		  	}
			/************************************************************/
		}
		case CHASSIS_MODE_STOP:
		{
			robot_info->speed_target.vx = 0;
			robot_info->speed_target.vy = 0;
			robot_info->speed_target.wz = 0;
			robot_info->sac++;
		}
	}

	cal_chassis_speed_2_motor(robot_info);

	if(robot_info->sac >= 400 && robot_info->chassis_state == CHASSIS_MODE_STOP)
	{
		if(robot_info->sac >= 1000)
		{
			Set_PWM_Motor_Speed(&hcan1, 0, 0, 0, 0);
		}
		else
			Set_PWM_Motor_Speed(&hcan1,2000 , 0, 0, 0);

		dji_motor_disable(motor_lf);
		dji_motor_disable(motor_lb);
		dji_motor_disable(motor_rf);
		dji_motor_disable(motor_rb);
	}
}

/* 进行地盘位置环计算，得到目标速度 */
void chassis_pos_calc(Robotinfo_t *robot_info) 
{	
	PID_calc(&pid_DJI_outer[0], robot_info->pos_error.ErrorposX, 0);
	PID_calc(&pid_DJI_outer[1], robot_info->pos_error.ErrorposY, 0);
	PID_calc(&pid_DJI_outer[2], robot_info->pos_error.ErrorposZ, 0);
	
	first_order_filter_cali(&filter_chassis_vx, pid_DJI_outer[0].out);
	first_order_filter_cali(&filter_chassis_vy, pid_DJI_outer[1].out);

	if (fabs(pid_DJI_outer[0].out) <= fabs(filter_chassis_vx.out))
		  robot_info->speed_target.vx = pid_DJI_outer[0].out;
	else  robot_info->speed_target.vx = filter_chassis_vx.out;
    
	if (fabs(pid_DJI_outer[1].out) <= fabs(filter_chassis_vy.out))
		  robot_info->speed_target.vy = pid_DJI_outer[1].out;
	else  robot_info->speed_target.vy = filter_chassis_vy.out;

	if(robot_info->limit_vy_flag == 1)
		abs_limit(&robot_info->speed_target.vy,1.6);	


	if(robot_info->pos_error.ErrorposZ > 180)
		robot_info->speed_target.wz =   pid_DJI_outer[2].out;
	else
		robot_info->speed_target.wz = - pid_DJI_outer[2].out;
}


/* 进行地盘速度环计算，得到目标电机速度 */
void cal_chassis_speed_2_motor(Robotinfo_t *robot_info)
{
	dji_motor_setref(motor_lf, (+robot_info->speed_target.vy + robot_info->speed_target.vx + robot_info->speed_target.wz));
	dji_motor_setref(motor_lb, (-robot_info->speed_target.vy + robot_info->speed_target.vx + robot_info->speed_target.wz));
	dji_motor_setref(motor_rb, (+robot_info->speed_target.vy - robot_info->speed_target.vx + robot_info->speed_target.wz));
	dji_motor_setref(motor_rf, (-robot_info->speed_target.vy - robot_info->speed_target.vx + robot_info->speed_target.wz));
}


void input_tarspeed_chassis(Robotinfo_t *robot_info, float tarx, float tary, float tarz)
{
  	robot_info->chassis_state = CHASSIS_MODE_MANUAL;
	robot_info->speed_target.vx = tarx;
	robot_info->speed_target.vy = tary;
	robot_info->speed_target.wz = tarz;
}


void input_tarpos_chassis(Robotinfo_t *robot_info, float tarpx, float tarpy,float tarpz)
{
  	robot_info->chassis_state = CHASSIS_MODE_AUTO;
	robot_info->pos_target.pos_x = tarpx;
	robot_info->pos_target.pos_y = tarpy;
	robot_info->pos_target.pos_z = tarpz;
}


void Filter_Init_All(void)
{
  	first_order_filter_init(&filter_chassis_vx, 0.0015f, TIMEFORACC);
	first_order_filter_init(&filter_chassis_vy, 0.0015f, TIMEFORACC);
	first_order_filter_init(&filter_chassis_vz, 0.0005f, TIMEFORACC);
}


void stop_chassis(Robotinfo_t *robot_info)
{
   robot_info->chassis_state = CHASSIS_MODE_STOP;
   robot_info->speed_target.vx = robot_info->speed_target.vy = robot_info->speed_target.wz = 0;
}

