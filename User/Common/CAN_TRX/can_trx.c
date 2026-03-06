#include "chassis_task.h"
#include "can_trx.h"


static int16_t send_current[8];
static int16_t send_current_CAN2[8];
static int16_t sbus_channel[16];
static uint16_t PWM_speed[3];

static UserCANHandle_t *can_instance[CAN_MAX_REGISTER_CNT] = {NULL};
static uint8_t idx;


static void CAN_add_filter(UserCANHandle_t *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;
	
	can_filter_conf.FilterActivation = ENABLE;   
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       
    can_filter_conf.SlaveStartFilterBank = 14;                                                                
    can_filter_conf.FilterIdLow = _instance->rx_id << 5;    
	can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); 
}


static void CANServiceInit()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}


UserCANHandle_t *CAN_register(CANInitConfig_t *config)
{
    if (idx == 0)
    {
        CANServiceInit(); // ,
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MAX_REGISTER_CNT) // 
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // rxiddjirxid
        if (can_instance[i]->can_handle == config->can_handle && can_instance[i]->rx_id == config->rx_id)
        {
            while (1)
                LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }
    
    UserCANHandle_t *instance = (UserCANHandle_t *)malloc( sizeof(UserCANHandle_t) ); // 
    memset(instance, 0, sizeof(UserCANHandle_t));                                     // 0,

    // 
    instance->txconf.StdId = config->tx_id; // id
    instance->txconf.IDE = CAN_ID_STD;      // id,idCAN_ID_EXT()
    instance->txconf.RTR = CAN_RTR_DATA;    // 
    instance->txconf.DLC = 0x08;            // 8

    // canid
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;  // txconf
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;  // DJIMotor_t

    CANAddFilter(instance);         // CAN
    can_instance[idx++] = instance; 

    return instance;
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 
}


static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO,
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // FIFO
        for (size_t i = 0; i < idx; i++)
        {   // StdID
            if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
            {
                if (can_instance[i]->can_module_callback != NULL) // 
                {
                    can_instance[i]->rx_len = rxconf.DLC;                      // 
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 
                    can_instance[i]->can_module_callback(can_instance[i]);     // 
                }
                return;
            }
        }
    }
}


uint8_t CANTransmit(UserCANHandle_t *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 
        {
            LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    {
        LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1;
}


void Set_PWM_Motor_Speed(CAN_HandleTypeDef *hcan, uint16_t speed1, uint16_t speed2, uint16_t speed3, uint16_t speed4)
{
    CAN_TxHeaderTypeDef  ZDY_tx_message;    //CAN
    uint8_t              can_send_data[8];  //
    uint32_t send_mail_box;
    ZDY_tx_message.StdId = 0xA7;
    ZDY_tx_message.IDE = CAN_ID_STD;
    ZDY_tx_message.RTR = CAN_RTR_DATA;
    ZDY_tx_message.DLC = 0x08;
    can_send_data[0] = speed1>>8;
    can_send_data[1] = speed1;
    can_send_data[2] = speed2>>8;
    can_send_data[3] = speed2;
    can_send_data[4] = speed3>>8;
    can_send_data[5] = speed3;
    can_send_data[6] = speed4>>8;
    can_send_data[7] = speed4;
    HAL_CAN_AddTxMessage(hcan , &ZDY_tx_message, can_send_data, &send_mail_box);
}

