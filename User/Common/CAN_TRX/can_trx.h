#ifndef CAN_TRX_H
#define CAN_TRX_H
#include "main.h"
#include "chassis_task.h"
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CONTROL_VALUE_2_VECTOR 0.00514f
#define TEST_MODE 1

#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载

typedef struct _
{
    CAN_HandleTypeDef  *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置

    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t  tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8

    uint32_t rx_id;                // 接收id
    uint8_t  rx_len;                // 接收长度,可能为0-8
	uint8_t  rx_buff[8];            // 接收缓存,最大消息长度为8

    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} UserCANHandle_t;


/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    uint32_t rx_id;                             // 接收id 电机的CAN instance的rx_id由电机的ID决定，因此通过电机的ID来设置
    uint32_t tx_id;                             // 发送id 电机的CAN instance只会用来接受，不会用这个instacne来进行发送，因此不进行初始化
    
    void (*can_module_callback)(UserCANHandle_t *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
    CAN_HandleTypeDef *can_handle;              // can句柄
} CANInitConfig_t;


extern int16_t send_current[8];
extern int16_t send_current_CAN2[8];
extern int16_t sbus_channel[16];
extern uint16_t PWM_speed[3];

UserCANHandle_t *CAN_register(CANInitConfig_t *config);

	
void CAN_Cmd_Chassis(int16_t S1,int16_t S2,int16_t S3,int16_t S4);
void CAN_Cmd_Gimbal(int16_t S1,int16_t S2,int16_t S3,int16_t S4);

void CAN_Cmd_Chassis_CAN2(int16_t S1,int16_t S2,int16_t S3,int16_t S4);
void CAN_Cmd_Gimbal_CAN2(int16_t S1,int16_t S2,int16_t S3,int16_t S4);

void Set_PWM_Motor_Speed(CAN_HandleTypeDef *hcan, uint16_t speed1, uint16_t speed2, uint16_t speed3, uint16_t speed4);

#endif
