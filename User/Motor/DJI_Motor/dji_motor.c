#include "dji_motor.h"
#include "can_trx.h"

static uint8_t idx = 0;
static DjiMotorHandle_t *dji_motor[DJI_MOTOR_CNT] = {NULL};//分别为地盘四个电机

// 电机发送的4个CAN实例，比较特殊因此进行手动注册
static UserCANHandle_t sender_assignment[4] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t send_enable_flag[6] = {0};


// 电机初始化,返回一个电机实例
DjiMotorHandle_t *djimotor_init(MotorInitConfig_t *config)
{
    DjiMotorHandle_t *instance = (DjiMotorHandle_t *)malloc( sizeof(DjiMotorHandle_t) );
    memset(instance, 0, sizeof(DjiMotorHandle_t) );

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                   // 2006 or 3508
    instance->settings   = config->controller_init_config; // 正反转,闭环类型,PID参数
    instance->motor_id   = config->motor_id;

    // motor controller init
    PID_init(&instance->motor_controller.current_PID, instance->settings.current_PID);
    PID_init(&instance->motor_controller.speed_PID,   instance->settings.speed_PID);
    PID_init(&instance->motor_controller.angle_PID,   instance->settings.angle_PID);

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    motor_send_grouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = decode_dji_motor; // 设置回调函数
    config->can_init_config.id = instance;                          // 设置当前电机实例的CAN ID为当前实例的地址
    instance->motor_can_handle = CAN_register(&config->can_init_config);

    
    dji_motor_enable(instance);
    dji_motor[idx++] = instance;
    return instance;
}


/**/
static void motor_send_grouping(DjiMotorHandle_t *motor, CANInitConfig_t *config)
{
    uint8_t motor_id = motor->motor_id; // 电调的ID
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case DJI_MOTOR_2006:
    case DJI_MOTOR_3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 2;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 3;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
        motor->motor_can_handle->rx_id =  config->rx_id; // 设置接收id

        send_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->send_group = motor_grouping;

        // 检查电机是否发生id冲突
        for (size_t i = 0; i < idx; i++)
        {
            if (dji_motor[i]->motor_can_handle->can_handle == config->can_handle && dji_motor[i]->motor_can_handle->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) 
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;
    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // 其他电机不应该在这里注册
    }
}


static void decode_dji_motor(UserCANHandle_t *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机DJIMotor_t的地址
    uint8_t *rxbuff = _instance->rx_buff;
    DjiMotorHandle_t *motor = (DjiMotorHandle_t *)_instance->id;
    DjiMotorMeasure_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销


    if(measure->msg_cnt < 50)
    {
        get_dji_motor_offset(measure, rxbuff);
        measure->msg_cnt++;
    }else{
        get_motor_measure(measure, rxbuff);
    }
    angle_calculate(measure);
}


void get_dji_motor_offset(DjiMotorMeasure_t *ptr, uint8_t data[8])
{
	ptr->ecd = (float)(data[0]<<8 | data[1]) ;
	ptr->offset_ecd = ptr->ecd;
	ptr->total_ecd -= ptr->offset_ecd;
}


void get_motor_measure(DjiMotorMeasure_t* ptr,uint8_t data[8])
{
    ptr->last_ecd=ptr->ecd;
    ptr->ecd=(float)(data[0]<<8|data[1]);
    ptr->speed_rpm=(int16_t)(data[2]<<8|data[3]);//*M3508_MOTOR_RPM_TO_VECTOR;//转化为真实速度
    ptr->speed_rpm *= M3508_MOTOR_RPM_TO_VECTOR;
    ptr->current=(int16_t)(data[4]<<8|data[5]);	
}


void angle_calculate(DjiMotorMeasure_t *motor)
{
        if(motor->speed_rpm >= 0)  //正转
        {
            if (motor->last_ecd - motor->ecd >= 4096)
                motor->total_ecd += 8192 - motor->last_ecd + motor->ecd;
            else
                motor->total_ecd += motor->ecd - motor->last_ecd;
        }
        else                                //反转
        {
            if(motor->ecd - motor->last_ecd >= 4096)
                motor->total_ecd -= 8192 - motor->ecd + motor->last_ecd;
            else
                motor->total_ecd -= motor->last_ecd - motor->ecd;
        }
        motor->angle = (float)motor->total_ecd / 8192 * 360 / 19;
}


// 设置参考值
void dji_motor_setref(DjiMotorHandle_t *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}


void dji_motor_disable(DjiMotorHandle_t *motor)
{
    motor->working_state = MOTOR_DISABLE;
}


void dji_motor_enable(DjiMotorHandle_t *motor)
{
    motor->working_state = MOTOR_ENABLE;
}


// 为所有电机实例计算三环PID,发送控制报文
void dji_motor_control()
{
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DjiMotorHandle_t *motor;
    MotorControllerInitConfig_t *motor_setting; // 电机控制参数
    MotorController_t *motor_controller;   // 电机控制器
    DjiMotorMeasure_t *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor[i];
        motor_setting = &motor->settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_direction == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            pid_measure = measure->angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PID_calc(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            pid_measure = measure->speed_rpm;
            // 更新pid_ref进入下一个环
            pid_ref = PID_calc(&motor_controller->speed_PID, pid_measure, pid_ref);
        }
        
        if (motor_setting->motor_direction == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t) pid_ref;

        // 分组填入发送数据
        group = motor->send_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->working_state == MOTOR_DISABLE)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i)
    {
        if (send_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}