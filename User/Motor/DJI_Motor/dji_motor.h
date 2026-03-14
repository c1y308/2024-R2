#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "motor_def.h"


#define DJI_MOTOR_CNT 12
/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF   0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f       // 必须大于0.9
#define ECD_ANGLE_COEF_DJI  0.043945f  // (360/8192),将编码器值转化为角度制
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

typedef struct 
{
	float target_angle;
    float angle;

	
  	float ecd;
	float last_ecd;
	float offset_ecd;
	float total_ecd;
	int16_t round_cnt;
	
	float target_speed;
	float speed_rpm;
	
	int16_t current;
	uint8_t temperate;

	int32_t msg_cnt;
}DjiMotorMeasure_t;


typedef enum
{
    DJI_MOTOR_2006 = 0,
    DJI_MOTOR_3508 = 1,
} MotorType_e;


typedef struct
{
	uint8_t 				motor_id;
	MotorType_e 			motor_type;
	
	MotorControllerInitConfig_t 	settings;
	MotorController_t 				motor_controller;

    DjiMotorMeasure_t 		measure;           // 电机测量值

    MotorState_e   			working_state;
    UserCANHandle_t 	   *motor_can_handle; // 电机CAN实例

	// 分组发送设置
    uint8_t send_group;
    uint8_t message_num;  // 组内编号
} DjiMotorHandle_t;


/**
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DjiMotorHandle_t *djimotor_init(MotorInitConfig_t *config);


void dji_motor_setref(DjiMotorHandle_t *motor, float ref);
/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void dji_motor_control();

void dji_motor_disable(DjiMotorHandle_t *motor);
void dji_motor_enable(DjiMotorHandle_t  *motor);


#endif // !DJI_MOTOR_H
