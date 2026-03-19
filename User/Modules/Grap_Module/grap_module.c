#include "grap_module.h"
#include "chassis_module.h"
static void set_grap_motor_zero_speed(void);
static void grap_only(void);
static void out_only(void);
static void grap_pos_init(void);
static void lift_seed_from_land(void);
static void restore_rotate_front(void);
static void restore_rotate_back(void);
static void lift_pre_put(void);
static void put_rotate_front(void);
static void put_rotate_back(void);


grap_t grap_ifo; 


DjiMotorHandle_t *motor_yaw_left, *motor_yaw_right, *motor_lift_left, *motor_lift_right, *motor_grap_left, *motor_grap_right;
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
    motor_lift_right = djimotor_init(&chassis_motor_config);

	chassis_motor_config.motor_id = 5;
    motor_grap_right = djimotor_init(&chassis_motor_config);
}


void grap_init(){
	GrapCommand_e cmd = CMD_SEED_INIT;
    osMessageQueuePut(grap_cmd_queueHandle, &cmd, 0, 0);
}

/* 每次抓取命令对应2种状态：抓取场地苗存储在车上、抓取场地上的苗进行种植 */
/* 抓取后存储在车上 */
void grap_seed_2_store(){
	GrapCommand_e cmd = CMD_SEED_GRAP_2_STORE;
    osMessageQueuePut(grap_cmd_queueHandle, &cmd, 0, 0); 

	/* 抓取完毕后需要旋转到存储位置 */
}
/* 抓取后直接去种植 */
void grap_seed_2_put(){
	GrapCommand_e cmd = CMD_SEED_GRAP_2_DIRECT_PUT;
    osMessageQueuePut(grap_cmd_queueHandle, &cmd, 0, 0); 

	/* 抓取完毕后需要降低高度准备种植 */
}


/* 种植存储的苗首先需要进行旋转 */
void put_stored_seed(){
	GrapCommand_e cmd = CMD_SEED_PUT_RESTORED_SEED;
    osMessageQueuePut(grap_cmd_queueHandle, &cmd, 0, 0);
}


static void init_grap_motor(){
	dji_motor_setref(motor_yaw_left, - GRAP_ANGLE_SPEED);
	dji_motor_setref(motor_yaw_right,  GRAP_ANGLE_SPEED);

	dji_motor_setref(motor_lift_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_lift_right, - SAFE_GRAP_SPEED);
	
	dji_motor_setref(motor_grap_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_grap_right, - SAFE_GRAP_SPEED);
}


// void grap_init(grap_t *grap_ifo)
// {   
//     grap_ifo->grap_state = GRAP_STATE_INIT;
//     grap_ifo->grap_last_state = GRAP_STATE_IDLE;
// 	grap_ifo->grap_arrive = 0;


// 	grap_ifo->grap_tick++;

// 	init_grap_motor();

// 	if(grap_ifo->grap_tick >= 300){
// 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
// 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
// 		set_grap_motor_zero_speed();
// 	}

// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
// }


void grap_task(grap_t *grap_ifo){

	GrapCommand_e cmd;
	/* 必须作为全局变量(?)，open的时候还要用 */
	switch (grap_ifo->grap_state)
	{
		case GRAP_STATE_IDLE:{

			set_grap_motor_zero_speed();

			if (osMessageQueueGet(grap_cmd_queueHandle, &cmd, NULL, 0) == osOK) {
				// 根据指令进入不同状态
				switch(cmd) {
				/* 初始化指令 */
				case CMD_SEED_INIT:{
					grap_ifo->grap_state = GRAP_STATE_INIT;
					break;
				}
				/* 抓取苗并存储 */
				case CMD_SEED_GRAP_2_STORE:{
					grap_ifo->grap_state = GRAP_STATE_GRAP_2_STORE;
					break;
				}
				/* 抓取苗直接种植 */
				case CMD_SEED_GRAP_2_DIRECT_PUT:{
					grap_ifo->grap_state = GRAP_STATE_GRAP_2_DIRECT_PUT;
					break;
				}
				/* 种植存储在车上的苗 */
				case CMD_SEED_PUT_RESTORED_SEED:{
					grap_ifo->grap_state = GRAP_STATE_PUT_ROTATE_FRONT;
					break;
				}
				default:
					break;
				}
			}
			break;
		}

		case GRAP_STATE_INIT:{
			grap_pos_init();
			grap_ifo->grap_state = GRAP_STATE_IDLE;
			break;
		}
		/* 抓取后把苗存储在车上 */
		case GRAP_STATE_GRAP_2_STORE:{

			/* 夹爪闭合抓取苗 */
			grap_only();

			/* 抬升把苗从场地拔出来 */
			lift_seed_from_land();

			/* 然后旋转存储在车上 */
			grap_ifo->grap_state = GRAP_STATE_RESTORE_ROTATE_FRONT;
	
			break;
		}
		case GRAP_STATE_RESTORE_ROTATE_FRONT:{

			/* 夹爪旋转到机器人存储苗的位置 */
			restore_rotate_front();

			/* 转到位，打开夹爪 */
			out_only();

			grap_ifo->grap_state = 	GRAP_STATE_RESTORE_ROTATE_BACK;
			break;
		}
		/* 存储完苗，夹爪旋转回去 */
		case GRAP_STATE_RESTORE_ROTATE_BACK:{

			restore_rotate_back();
			
			grap_ifo->grap_arrive = 1;  // 划重点

			/* 完成这一个动作，等待下一个指令 */
			grap_ifo->grap_state = 	GRAP_STATE_IDLE;

			break;
		}
		case GRAP_STATE_GRAP_2_DIRECT_PUT:{
			/* 抓取场地苗 */
			grap_only();

			/* 抬升把苗从场地拔出来 */
			lift_seed_from_land();

			/* 然后降低高度准备种植 */
			lift_pre_put();
			grap_ifo->grap_state = GRAP_STATE_PUT_CORRECT;

			break;
		}
		case GRAP_STATE_PUT_ROTATE_FRONT:{
			put_rotate_front();

			/* 进行抓取 */
			grap_only();

			grap_ifo->grap_state = GRAP_STATE_PUT_ROTATE_BACK;

			break;
		}

		case GRAP_STATE_PUT_ROTATE_BACK:{

			/* 旋转到放苗的位置 */
			put_rotate_back();

			grap_ifo->grap_state = GRAP_STATE_PUT_CORRECT;
			
			break;
		}

        case GRAP_STATE_PUT_CORRECT:{

			// 需要把地盘模式设置为纯手动控制
            // 需要seed_task一个信号量来进行 ack 确认可以打开爪子放苗
			out_only();

			/* 打开夹爪，完成动作，等待下一个命令 */
			grap_ifo->grap_state = GRAP_STATE_IDLE;
			break;
		}
		default:
			break;
	}
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


//左侧爪子负数为抬升，负数为转向存储方向
static void grap_only()//闭合夹爪
{
	dji_motor_setref(motor_yaw_left,  0);
	dji_motor_setref(motor_lift_left, 0);
	dji_motor_setref(motor_grap_left,    GRAP_SPEED);

	dji_motor_setref(motor_yaw_right,  0);
	dji_motor_setref(motor_lift_right, 0);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED);

	osDelay(GRAP_TICK_CLOSE);
}


static void out_only()
{
	dji_motor_setref(motor_yaw_left,  0);
	dji_motor_setref(motor_lift_left, 0);
	dji_motor_setref(motor_grap_left,    GRAP_SPEED_OPEN);

	dji_motor_setref(motor_yaw_right,  0);
	dji_motor_setref(motor_lift_right, 0);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);

	osDelay(GRAP_TICK_OPEN);
}

static void grap_pos_init(){
	dji_motor_setref(motor_yaw_left, - GRAP_ANGLE_SPEED);
	dji_motor_setref(motor_yaw_right,  GRAP_ANGLE_SPEED);

	dji_motor_setref(motor_lift_left,  - SAFE_GRAP_SPEED);
	dji_motor_setref(motor_lift_right,   SAFE_GRAP_SPEED);

	dji_motor_setref(motor_grap_left,    SAFE_GRAP_SPEED);
	dji_motor_setref(motor_grap_right, - SAFE_GRAP_SPEED);

	osDelay(150);

	dji_motor_setref(motor_yaw_left, - LOW_ANGLE_SPEED);
	dji_motor_setref(motor_yaw_right,  LOW_ANGLE_SPEED);

	dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED);
	dji_motor_setref(motor_lift_right,    GRAP_LIFT_SPEED);

	dji_motor_setref(motor_grap_left,    GRAP_SPEED_OPEN);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED_OPEN);

	osDelay(150);

	set_grap_motor_zero_speed();
}


static void lift_seed_from_land(){

	dji_motor_setref(motor_lift_left,    SPEED_LIFT_PLUS);
	dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);

	osDelay(GRAP_LIFT_TICK_PRE);
}

static void restore_rotate_front(){
	dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
	dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
	dji_motor_setref(motor_grap_left,   GRAP_SPEED);

	dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
	dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED);

	osDelay(CAUTION_TICK);

	/* ！！！ */
	// grap_ifo->ready_2_move = 1;//允许车移动到下一个取苗点位（考虑以信号量进行替代，此处给一个信号量）

	dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS);
	dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS);
	dji_motor_setref(motor_grap_left,   GRAP_SPEED);

	dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS);
	dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS);
	dji_motor_setref(motor_grap_right, - GRAP_SPEED);

	osDelay(GRAP_LIFT_TICK_PRE_COM);

	dji_motor_setref(motor_lift_left,   -1);
	dji_motor_setref(motor_lift_right,   1);

	osDelay(CONTINUE_STORAGE_TICK - GRAP_LIFT_TICK_PRE_COM);
}


static void restore_rotate_back(){
	dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED - SPEED_ANGLE_PLUS);
	dji_motor_setref(motor_lift_left,   - GRAP_LIFT_SPEED  - SPEED_LIFT_PLUS);
	
	dji_motor_setref(motor_yaw_right,   GRAP_ANGLE_SPEED  + SPEED_ANGLE_PLUS);
	dji_motor_setref(motor_lift_right,  GRAP_LIFT_SPEED   + SPEED_LIFT_PLUS);

	osDelay(GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 20);

	dji_motor_setref(motor_yaw_left,    -0);
	dji_motor_setref(motor_lift_left,   -LOW_LIFT_SPEED);
	
	dji_motor_setref(motor_yaw_right,   0);
	dji_motor_setref(motor_lift_right,  LOW_LIFT_SPEED);
}


static void lift_pre_put(){
	/* 降低夹爪高度，准备种植苗 */
	dji_motor_setref(motor_lift_left,    2.3);
	dji_motor_setref(motor_lift_right, - 2.3);

	osDelay(280);

	dji_motor_setref(motor_lift_left,   0);
	dji_motor_setref(motor_lift_right,  0);
}


static void put_rotate_front(){
	/* 夹爪旋转到存储苗的位置 */
	dji_motor_setref(motor_yaw_left,    SPEED_ANGLE_PLUS + 0.4);//减速+ 0.1
	dji_motor_setref(motor_lift_left,   SPEED_LIFT_PLUS  - 0.4);//加速- 0.1

	dji_motor_setref(motor_yaw_right,  - SPEED_ANGLE_PLUS - 0.4);
	dji_motor_setref(motor_lift_right, - SPEED_LIFT_PLUS  + 0.4);

	osDelay(GRAP_LIFT_TICK_AFT + GRAP_LIFT_TICK_PRE - GRAP_FAST_TICK + 70);
}


static void put_rotate_back(){
	dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED);//减速+ 0.1
	dji_motor_setref(motor_lift_left,     0);
	dji_motor_setref(motor_grap_left,     GRAP_SPEED);

	dji_motor_setref(motor_yaw_right,     GRAP_ANGLE_SPEED);
	dji_motor_setref(motor_lift_right,    0);		
	dji_motor_setref(motor_grap_right,  - GRAP_SPEED);	

	osDelay(START_DOWN_COM_TICK);

	/* 允许下降高度 */
	dji_motor_setref(motor_yaw_left,    - GRAP_ANGLE_SPEED);//减速+ 0.1
	dji_motor_setref(motor_lift_left,   - SPEED_LIFT_PLUS  + 0.8);//加速- 0.1
	dji_motor_setref(motor_grap_left,     GRAP_SPEED);

	dji_motor_setref(motor_yaw_right,     GRAP_ANGLE_SPEED);
	dji_motor_setref(motor_lift_right,    SPEED_LIFT_PLUS - 0.8);		
	dji_motor_setref(motor_grap_right,  - GRAP_SPEED);	

	osDelay(GRAP_LIFT_TICK_AFT + 50);
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