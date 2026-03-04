/**
  ****************************(C)SWJTU_ROBOTCON2022****************************
  * @file       uart_dma.c/h
  * @brief      这里是UART_DMA接收函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-12-2022       ZDYukino        1. done
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C)SWJTU_ROBOTCON2024****************************
  **/
#include "vofa.h"
#include "vofa_setting.h"
#include "main.h"
#include "usart.h"
#include "uart_dma.h"
#include "string.h"
#include "chassis_task.h"
#include "user_lib.h"
#include "Code_Disc.h"
/*缓存数组预定义*/

uint8_t buffer_receve_1[128];
uint8_t buffer_receve_2[128];
uint8_t buffer_receve_3[128];
uint8_t buffer_receve_4[128];
uint8_t buffer_receve_5[128];
uint8_t buffer_receve_6[128];
uint16_t ld_distance;
/**
  * @brief          UART1-6中断接收服务函数
  * @param[in]      接收数组
  * @param[in]      长度值【0-128】
  * @retval         none
  */
static void UART1_Receive_Serve(uint8_t *buffer, uint8_t lenth);
static void UART2_Receive_Serve(uint8_t *buffer, uint8_t lenth);
static void UART3_Receive_Serve(uint8_t *buffer, uint8_t lenth);
static void UART4_Receive_Serve(uint8_t *buffer, uint8_t lenth);
static void UART5_Receive_Serve(uint8_t *buffer, uint8_t lenth);
static void UART6_Receive_Serve(uint8_t *buffer, uint8_t lenth);



/**
  * @brief          初始化串口DMA接收
  * @param[in]      UART接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
void UART_DMA_Recevie_init(UART_HandleTypeDef *usart, uint8_t *buffer, uint8_t lenth)
{
  __HAL_UART_ENABLE_IT(usart, UART_IT_IDLE);  //使能UART设备的空闲中断，即一帧（协议帧）数据接收完成后由于总线产生空闲导致中断
  HAL_UART_Receive_DMA(usart, buffer, lenth); //打开DMA接收
}


/**
  * @brief          串口DMA接收中断函数->放入《USER CODE BEGIN USARTX_IRQn 1》 中
  * @param[in]      UART接口
  * @param[in]      UART DMA接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
void UART_DMA_Recevie_IT(UART_HandleTypeDef *usart, DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t lenth)
{
  if(0 != __HAL_UART_GET_FLAG(usart, UART_FLAG_IDLE))  //是否是空闲中断
  {
    uint8_t temp = 0;
    uint8_t len = 0;     
    __HAL_UART_CLEAR_IDLEFLAG(usart);     //清除空闲中断标志位
    HAL_UART_DMAStop(usart);              //停止DMA接收
    temp  =  __HAL_DMA_GET_COUNTER(DMA);  //获取DMA剩余传输计数器的值
    len =    lenth - temp;                //计算已经接收到的数据长度
    
    if(usart == &huart1)      UART1_Receive_Serve(buffer, len);     //选择解码程序
    else if(usart == &huart2) UART2_Receive_Serve(buffer, len);     //选择解码程序
    else if(usart == &huart3) UART3_Receive_Serve(buffer, len);     //选择解码程序

    memset(buffer, 0, len);//清除缓存	
    HAL_UART_Receive_DMA(usart,buffer,lenth);//重新打开DMA接收  
  }
}


/**
  * @brief          串口异常的处理
  * @param[in]      UART接口
  * @retval         none
  */
void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)
{
	uint32_t data;
  __HAL_UNLOCK(huart);
 
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)//判断是否发生了溢出错误
	{
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);      		//清除溢出中断
		data = huart->Instance->SR;//读取UART设备的状态寄存器的值，以清除其中的错误标志位
		data = huart->Instance->DR;//读取UART设备的数据寄存器的值，以清除其中的错误标志位
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//重新使能UART设备的空闲中断
	}
}



//UART1中断接收函数
static void UART1_Receive_Serve(uint8_t *buffer, uint8_t lenth)
{
	static uint8_t temp[4];
	static uint16_t result_temp;
  if(buffer[0] == 'D')
	{
	  temp[0] = buffer[2]-'0';//米
		temp[1] = buffer[4]-'0';//小数点后第一位
		temp[2] = buffer[5]-'0';//第二位
		temp[3] = buffer[6]-'0';//第三位
		result_temp = temp[0]*1000 + temp[1]*100 + temp[2]*10 + temp[3];
		if(result_temp<=4000)
		{
			ld_distance = result_temp;
			real_lv100 = CORRECT_DISTACNE - ld_distance;
		}
	}	
}


//UART2中断接收函数
static void UART2_Receive_Serve(uint8_t *buffer, uint8_t lenth)
{
//	Vofa_UART_Receive(buffer,20);
}


//UART3中断接收函数
static void UART3_Receive_Serve(uint8_t *buffer, uint8_t lenth)
{
	  CD_get_measer(buffer, lenth);
}
