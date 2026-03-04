/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_task.h"
#include "seed_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for seed_task */
osThreadId_t seed_taskHandle;
const osThreadAttr_t seed_task_attributes = {
  .name = "seed_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for grap_task */
osThreadId_t grap_taskHandle;
const osThreadAttr_t grap_task_attributes = {
  .name = "grap_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dji_motor_cntl_task */
osThreadId_t dji_motor_cntl_taskHandle;
const osThreadAttr_t dji_motor_cntl_task_attributes = {
  .name = "dji_motor_cntl_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void chassis_task_entry(void *argument);
void seed_task_entry(void *argument);
void grap_task_entry(void *argument);
void dji_motor_cntl_task_entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of chassis_task */
  chassis_taskHandle = osThreadNew(chassis_task_entry, NULL, &chassis_task_attributes);

  /* creation of seed_task */
  seed_taskHandle = osThreadNew(seed_task_entry, NULL, &seed_task_attributes);

  /* creation of grap_task */
  grap_taskHandle = osThreadNew(grap_task_entry, NULL, &grap_task_attributes);

  /* creation of dji_motor_cntl_task */
  dji_motor_cntl_taskHandle = osThreadNew(dji_motor_cntl_task_entry, NULL, &dji_motor_cntl_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chassis_task_entry */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task_entry */
void chassis_task_entry(void *argument)
{
  /* USER CODE BEGIN chassis_task_entry */
  /* Infinite loop */
  for(;;)
  {
    chassis_feedback_update();
    chassis_arrive_check();
    chassis_calc_tarspeed_task();
    
    osDelay(1);
  }
  /* USER CODE END chassis_task_entry */
}

/* USER CODE BEGIN Header_seed_task_entry */
/**
* @brief Function implementing the seed_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_seed_task_entry */
void seed_task_entry(void *argument)
{
  /* USER CODE BEGIN seed_task_entry */
  /* Infinite loop */
  for(;;)
  {
    plant_task();
    osDelay(1);
  }
  /* USER CODE END seed_task_entry */
}

/* USER CODE BEGIN Header_grap_task_entry */
/**
* @brief Function implementing the grap_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_grap_task_entry */
void grap_task_entry(void *argument)
{
  /* USER CODE BEGIN grap_task_entry */
  /* Infinite loop */
  for(;;)
  {
    grap_task();
    osDelay(1);
  }
  /* USER CODE END grap_task_entry */
}

/* USER CODE BEGIN Header_dji_motor_cntl_task_entry */
/**
* @brief Function implementing the dji_motor_cntl_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dji_motor_cntl_task_entry */
void dji_motor_cntl_task_entry(void *argument)
{
  /* USER CODE BEGIN dji_motor_cntl_task_entry */
  /* Infinite loop */
  for(;;)
  {
    dji_motor_control();
    osDelay(1);
  }
  /* USER CODE END dji_motor_cntl_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

