#ifndef __MOTOR_DEF_H__
#define __MOTOR_DEF_H__
#include "pid.h"
#include "dji_motor.h"

typedef enum
{
    MOTOR_ENABLE = 0,
    MOTOR_DISABLE = 1,
} MotorState_e;


typedef enum
{
    OPEN_LOOP    = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP   = 0b0010,
    ANGLE_LOOP   = 0b0100,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP   = 0b0110,
    ALL_THREE_LOOP         = 0b0111,
} CloseLoopType_e;


/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL  = 0,
    MOTOR_DIRECTION_REVERSE = 1
} MotorDirection_e;


typedef struct
{
    CloseLoopType_e outer_loop_type;        // 最外层的闭环,未设置时默认为最高级的闭环
    CloseLoopType_e close_loop_type;        // 使用几个闭环(串级)
    MotorDirection_e motor_direction;       // 是否反转
} MotorControlSetting_t;


typedef struct
{
    PID_t current_PID;
    PID_t speed_PID;
    PID_t angle_PID;

    float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
} MotorController_t;


typedef struct
{
    PIDInitConfig_t current_PID;
    PIDInitConfig_t speed_PID;
    PIDInitConfig_t angle_PID;
} MotorControllerInitConfig_t;


typedef struct
{
    MotorControlSetting_t       controller_setting_init_config;
    MotorControllerInitConfig_t controller_param_init_config;
    MotorType_e motor_type;

    UserCANHandle_t *motor_can_handle; // 电机CAN实例
    CANInitConfig_t can_init_config;

    uint8_t motor_id; // 电机ID
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;
} MotorInitConfig_t;
#endif
