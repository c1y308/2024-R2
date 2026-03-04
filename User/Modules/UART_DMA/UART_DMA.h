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
  ****************************(C)SWJTU_ROBOTCON2023****************************
  **/

#ifndef UART_DMA_H
#define UART_DMA_H

#include "main.h"
#include "usart.h"

extern uint8_t buffer_receve_1[128];//缓存数组全局定义
extern uint8_t buffer_receve_2[128];//缓存数组全局定义
extern uint8_t buffer_receve_3[128];//缓存数组全局定义
extern uint8_t buffer_receve_4[128];//缓存数组全局定义
extern uint8_t buffer_receve_5[128];//缓存数组全局定义
extern uint8_t buffer_receve_6[128];//缓存数组全局定义
extern uint16_t ld_distance;
/**
  * @brief          初始化串口DMA接收
  * @param[in]      UART接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
extern void UART_DMA_Recevie_init(UART_HandleTypeDef *usart,uint8_t *buffer,uint8_t lenth);
/**
  * @brief          串口DMA接收中断函数->放入《USER CODE BEGIN USARTX_IRQn 1》 中
  * @param[in]      UART接口
  * @param[in]      UART DMA接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
extern void UART_DMA_Recevie_IT(UART_HandleTypeDef *usart,DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t lenth);


#endif
