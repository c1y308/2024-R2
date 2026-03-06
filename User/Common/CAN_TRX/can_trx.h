#ifndef CAN_TRX_H
#define CAN_TRX_H
#include "main.h"
#include "can.h"
//#define CONTROL_VALUE_2_VECTOR 0.00514f
//#define TEST_MODE 1

#define CAN_MAX_REGISTER_CNT 16  // CAN MAX 注册数量    
typedef struct _
{
    CAN_HandleTypeDef  *can_handle; 
    CAN_TxHeaderTypeDef txconf;    

    uint32_t tx_id;               
    uint32_t tx_mailbox;          
    uint8_t  tx_buff[8];            

    uint32_t rx_id;                
    uint8_t  rx_len;                
	uint8_t  rx_buff[8];          

    void (*can_module_callback)(struct _ *); 
    void *id;
} UserCANHandle_t;



typedef struct
{
    uint32_t rx_id;                             
    uint32_t tx_id;                           
    
    void (*can_module_callback)(UserCANHandle_t *); 
    void *id;                                   
    CAN_HandleTypeDef *can_handle;
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
