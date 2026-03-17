#include "chassis_module.h"
#include "Code_Disc.h"
#include "NRF24L01.h"
#include "dji_motor.h"
float controller_vx;
float controller_vy;
float controller_wz;

ChassisInfo_t chassis_info;

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
	
	// chassis_info.protect_flag = 1;
	// chassis_info.disable_rotation_flag = 0;
	
	chassis_info.sac = 0;
	chassis_info.limit_vy_flag = 0;
	chassis_info.red_single_flag = chassis_info.blue_single_flag = 0;
	chassis_info.init_tick = 0;


	chassis_info.task_type = TASK_TYPE_CHASSIS_INIT;
	chassis_info.tol_state = CHASSIS_MODE_TOL_SMALL;
	// UART_DMA_Recevie_init(&huart1,buffer_receve_1,100); 
	// UART_DMA_Recevie_init(&huart2,buffer_receve_2,100);
	// UART_DMA_Recevie_init(&huart3,buffer_receve_3,100);

}


void Chassis_Callback()
{
	chassis_feedback_update(&chassis_info);
	// Calc_Send_Task();
}


/* 底盘反馈更新:获取当前底盘速度和位置，更新位置/角度的误差信息 */
void chassis_feedback_update(ChassisInfo_t *chassis_info)
{
	chassis_info->speed_now.vx = (motor_lf->measure.speed_rpm + motor_lb->measure.speed_rpm) / 2;
	chassis_info->speed_now.vy = (motor_lf->measure.speed_rpm - motor_lb->measure.speed_rpm) / 2;
	chassis_info->speed_now.wz = Code_Disc_robot.pal_yaw_rad;
	
	chassis_info->pos_now.pos_x = - Code_Disc_robot.x;
	chassis_info->pos_now.pos_y = - Code_Disc_robot.y;
	chassis_info->pos_now.pos_z =   Code_Disc_robot.yaw_rad * RADIAN_TO_ANGLE;	

	chassis_info->pos_error.ErrorposX_temp = chassis_info->pos_now.pos_x - chassis_info->pos_target.pos_x;
	chassis_info->pos_error.ErrorposY_temp = chassis_info->pos_now.pos_y - chassis_info->pos_target.pos_y;
	
	chassis_info->pos_error.ErrorposX = cos(  chassis_info->pos_now.pos_z * ANG2RAD) * \
	chassis_info->pos_error.ErrorposX_temp - sin(- chassis_info->pos_now.pos_z * ANG2RAD) * chassis_info->pos_error.ErrorposY_temp;

	chassis_info->pos_error.ErrorposY = sin(- chassis_info->pos_now.pos_z * ANG2RAD) * \
	chassis_info->pos_error.ErrorposX_temp + cos(  chassis_info->pos_now.pos_z * ANG2RAD) * chassis_info->pos_error.ErrorposY_temp;

	chassis_info->pos_error.ErrorposZ = chassis_info->pos_now.pos_z - chassis_info->pos_target.pos_z;
}


void chassis_task_entry(void *argument)
{
	ChassisInfo_t *chassis_info = (ChassisInfo_t *)argument;
    ChassisCmd_t rx_cmd;
    static uint16_t arrive_tick = 0;
    while(1)
    {
        // 1. 接收队列
        if (xQueueReceive(chassis_cmd_queueHandle, &rx_cmd, 0) == pdPASS) {
            chassis_info->cmd_seq_id = rx_cmd.cmd_seq_id;
            chassis_info->pos_target = rx_cmd.pos_target;
            chassis_info->speed_target = rx_cmd.speed_target;
			chassis_info->mode_x = rx_cmd.mode_x;
			chassis_info->mode_y = rx_cmd.mode_y;
			chassis_info->mode_z = rx_cmd.mode_z;
        }

        // 2. 传感器反馈更新 (获取 pos_now 和 speed_now)
        chassis_feedback_update(chassis_info);

        // ========================================================
        // 3. 核心精髓：三轴解耦计算 (各自算各自的，互不干扰)
        // ========================================================

        // ---- X 轴处理 ----
        if (rx_cmd.mode_x == AXIS_MODE_POS) {
            PID_calc(&pid_DJI_outer[0], chassis_info->pos_error.ErrorposX, 0);
			first_order_filter_cali(&filter_chassis_vx, pid_DJI_outer[0].out);
			if (fabs(pid_DJI_outer[0].out) <= fabs(filter_chassis_vx.out))
		  		chassis_info->speed_target.vx = pid_DJI_outer[0].out;
			else  
				chassis_info->speed_target.vx = filter_chassis_vx.out;
        } 
        else if (rx_cmd.mode_x == AXIS_MODE_VEL) {
            chassis_info->speed_target.vx = rx_cmd.speed_target.vx; // 纯速度模式直通
        } 
        else {
            chassis_info->speed_target.vx = 0; // STOP
        }

        // ---- Y 轴处理 ----
        if (rx_cmd.mode_y == AXIS_MODE_POS) {
            PID_calc(&pid_DJI_outer[1], chassis_info->pos_error.ErrorposY, 0);
			first_order_filter_cali(&filter_chassis_vy, pid_DJI_outer[1].out);
			if (fabs(pid_DJI_outer[1].out) <= fabs(filter_chassis_vy.out))
		  		chassis_info->speed_target.vy = pid_DJI_outer[1].out;
			else  
				chassis_info->speed_target.vy = filter_chassis_vy.out;
        } 
        else if (rx_cmd.mode_y == AXIS_MODE_VEL) {
            chassis_info->speed_target.vy = rx_cmd.speed_target.vy;
        } 
        else {
            chassis_info->speed_target.vy = 0;
        }

        // ---- Z 轴 (Yaw) 处理 ----
        if (rx_cmd.mode_z == AXIS_MODE_POS) {
			PID_calc(&pid_DJI_outer[2], chassis_info->pos_error.ErrorposZ, 0);

			if(chassis_info->pos_error.ErrorposZ > 180)
				chassis_info->speed_target.wz =   pid_DJI_outer[2].out;
			else
				chassis_info->speed_target.wz = - pid_DJI_outer[2].out;
        } 
        else if (rx_cmd.mode_z == AXIS_MODE_VEL) {
            chassis_info->speed_target.wz = rx_cmd.speed_target.wz;
        } 
        else {
            chassis_info->speed_target.wz = 0;
        }

        // ========================================================
        // 4. 到位判断机制 (智能忽略速度轴)
        // ========================================================
        // 只有配置为 POS 的轴，才参与到位判断！如果某轴是 VEL，说明它不需要“到达”


        bool x_arrived = (rx_cmd.mode_x != AXIS_MODE_POS) || (fabs(chassis_info->pos_error.ErrorposX) < CHASSIS_TOL);
        bool y_arrived = (rx_cmd.mode_y != AXIS_MODE_POS) || (fabs(chassis_info->pos_error.ErrorposY) < CHASSIS_TOL);
        bool z_arrived = (rx_cmd.mode_z != AXIS_MODE_POS) || (fabs(chassis_info->pos_error.ErrorposZ) < CHASSIS_TOL_Z);

        if (x_arrived && y_arrived && z_arrived) {
            arrive_tick++;
            if (arrive_tick >= CHASSIS_CHECK_TICK) {
                chassis_info->arrived_seq_id = chassis_info->cmd_seq_id; // 同步 ID，通告上层任务
            }
        } else {
            arrive_tick = 0;
        }

        // 5. 最终运动学逆解 (输出到电机)
        cal_chassis_speed_2_motor(chassis_info);
        
        vTaskDelay(5);
    }
}


/* 进行地盘速度环计算，得到目标电机速度 */
void cal_chassis_speed_2_motor(ChassisInfo_t *chassis_info)
{
	dji_motor_setref(motor_lf, (+chassis_info->speed_target.vy + chassis_info->speed_target.vx + chassis_info->speed_target.wz));
	dji_motor_setref(motor_lb, (-chassis_info->speed_target.vy + chassis_info->speed_target.vx + chassis_info->speed_target.wz));
	dji_motor_setref(motor_rb, (+chassis_info->speed_target.vy - chassis_info->speed_target.vx + chassis_info->speed_target.wz));
	dji_motor_setref(motor_rf, (-chassis_info->speed_target.vy - chassis_info->speed_target.vx + chassis_info->speed_target.wz));
}


void Filter_Init_All(void)
{
  	first_order_filter_init(&filter_chassis_vx, 0.0015f, TIMEFORACC);
	first_order_filter_init(&filter_chassis_vy, 0.0015f, TIMEFORACC);
	first_order_filter_init(&filter_chassis_vz, 0.0005f, TIMEFORACC);
}


// void stop_chassis(ChassisInfo_t *chassis_info)
// {
//    chassis_info->chassis_state = CHASSIS_MODE_STOP;
//    chassis_info->speed_target.vx = chassis_info->speed_target.vy = chassis_info->speed_target.wz = 0;
// }

