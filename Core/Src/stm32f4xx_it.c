/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_task.h"
#include "vofa_setting.h"
#include "vofa.h"
#include "uart_dma.h"
#include "usart.h"
#include "can_trx.h"
#include "seed_task.h"
#include "Code_Disc.h"
#include "NRF24L01.h"
#include "ball_task.h"
#include "grapping.h"
#include "single_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  UART_DMA_Recevie_IT(&huart1, &hdma_usart1_rx, buffer_receve_1, 100);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  UART_DMA_Recevie_IT(&huart2,&hdma_usart2_rx,buffer_receve_2,100);
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  UART_DMA_Recevie_IT(&huart3,&hdma_usart3_rx,buffer_receve_3,100);
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */
void CAN2_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX1_IRQn 0 */

  /* USER CODE END CAN2_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX1_IRQn 1 */

  /* USER CODE END CAN2_RX1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint8_t rx_buf[8] = {0};
uint8_t tx_back_buf[8] = {0};
int16_t speed = 0;
uint16_t area = 0x01; //ѡ������ 0x01 left 0x02 right
int area_flag = 0; //ѡȡ��־λ

float manual_vx = 0;
float manual_vy = 0;
float manual_wz = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t init_once_pid = 1;
	if (htim == &htim6)    
	{
		if(NRF24L01_RxPacket_ack_data(rx_buf) == 0)
		{
        all_ifo.check_flag_2++;			
				if(rx_buf[0] == 0x0a && rx_buf[7] == 0xff)
				{
					if(rx_buf[1] == 0x00)
					{
						manual_vy = (float) -((rx_buf[2] << 8 | rx_buf[3])-1000)/1000;
						manual_vx = (float) ((rx_buf[4] << 8 | rx_buf[5])-1000)/1000;
						manual_wz = (float) ((rx_buf[6]-100))/100;
					}
					else if (rx_buf[1] == 0x04)
					{
						ball_ifo.target_ball = rx_buf[2];
					}
					else if (rx_buf[1] ==0xA6){
						switch(rx_buf[2]){
							case 0x10:{
							//����1
								all_ifo.sssstar = 1;
								break;
							}
							case 0x11:{
							//����2
								ball_ifo.ball_confirm = ball_ifo.ball_confirm++;
								break;
							}
							case 0x20:{
								//����3
								all_ifo.stop_flag=1;
								break;
							}
							case 0x21:{
								//����4
								all_ifo.sssstar = 1;
								all_ifo.stop_flag=0;
								all_ifo.sac = 0;
								all_ifo.task_type = manual_type;
								break;
							}
							case 0x30:{//����������
								all_ifo.stop_flag=0;
								all_ifo.task_type = single_type;
							  all_ifo.red_single_flag = 1;
								all_ifo.sssstar = 1;
								break;
							}
							case 0x31:{//����������
								all_ifo.stop_flag=0;
								all_ifo.task_type = single_type;
							  all_ifo.blue_single_flag = 1;
								all_ifo.sssstar = 1;
								break;
							}
						}
					}
					else if(rx_buf[1] == 0xA7){
						speed = (int16_t)(rx_buf[2] <<8 | rx_buf[3]);
					}
					else if(rx_buf[1] == 0xA8){
						all_ifo.manual_flag = rx_buf[2];
					}
					else if(rx_buf[1] == 0xf0&&rx_buf[2]!=0) {
						area = rx_buf[2];
						area_flag = rx_buf[3];
						if(area == 1&&area_flag == 0) 
						{
							sign_t = 1;
							tx_back_buf[1]=1;//���� ��
							nRF24L01_Rx_AckPayload(tx_back_buf,1);
						}
						else if(area == 2&&area_flag == 0) 
						{
							sign_t = -1;
							tx_back_buf[1]=2;//���� ��
							nRF24L01_Rx_AckPayload(tx_back_buf,1);
						}
						//if(rx_buf[2] == 2) area_flag = 1;
						
//						nRF24L01_Rx_AckPayload(tx_back_buf,1);
					}	
			  }
    }
		if(all_ifo.init_tick < 10)
		{
			all_ifo.init_tick++;
		}
		else if(all_ifo.init_tick >= 10 && all_ifo.sssstar==1)//
		{
			if(init_once_pid == 1)
			{
				PID_Init_All();
				init_once_pid = 0;
			}
			switch(all_ifo.task_type)
			{
				case get_seed_type:{//ȡ������
					GP_Task_Ultra();
					break;
				}
				case transition:{//��ȡ����ɣ���б�£��������̣�
					Transition_Task();
					break;
				}
				case ball_type:{//ȡ��������
					Ball_Task_Ultra();
					break;
				}
				case remake_type:{
				  Remake_Task();
					break;
				}
				case test_type:{
//					magnet_control();
//					Input_TarSpeed_Chassis(0,0,0);
				  break;
				}
				case grap_test_type:{//������
					if(grap_ifo.test_semophare%2 == 0)
				    Grap_Storage();
					else
				    Out_Storage();				
					break;
				}
				case single_type:{//������
				  GP_Task_Single();
					break;
				}
				case single_ball_type:{
				  Ball_Task_Single();
					break;
				};
				case manual_type:{
					Input_TarSpeed_Chassis(manual_vx,manual_vy,manual_wz);
					break;
				}
				
				default:{
						all_ifo.chassis_mode = check_mode;
						all_ifo.target_vx_direct = all_ifo.target_vy_direct = all_ifo.target_wz_direct = 0;
						break;
				}
			}
			Grapping_Callback();
			Chassis_Callback();
		}
	}
	else if(htim == &htim7)
	{
	 	tempFloat[0] = all_ifo.pos_target.target_pos_x;
		tempFloat[1] = all_ifo.pos_target.target_pos_y;
    tempFloat[2] = all_ifo.pos_target.target_pos_z;
		
		tempFloat[3] = all_ifo.posX;
		tempFloat[4] = all_ifo.posY;
    tempFloat[5] = all_ifo.posZ;
			
		tempFloat[6] = all_ifo.init_tick;
		tempFloat[7] = all_ifo.red_single_flag;
    tempFloat[8] = all_ifo.blue_single_flag;
		
		tempFloat[9] =  sign_t ;
		tempFloat[10] = DJI_motor_CAN2[0].angle;
    tempFloat[11] = DJI_motor_CAN2[1].angle;
		
		tempFloat[12] =  DJI_motor_CAN2[2].angle;
		tempFloat[13] =  DJI_motor_CAN2[3].angle;;
		tempFloat[14] =  grap_ifo.ErrorAngle[1];
		tempFloat[15] =  send_current_CAN2[3];
		tempFloat[16] =  all_ifo.check_flag_2;
		tempFloat[17] =  all_ifo.check_flag;
		tempFloat[19] =  all_ifo.check_flag_3;
		Vofa_Transmit(&huart2,20);
	}
}
/* USER CODE END 1 */
