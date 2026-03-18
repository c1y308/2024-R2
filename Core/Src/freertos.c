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
#include "chassis_module.h"
#include "grap_module.h"
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
/* Definitions for motor_task */
osThreadId_t motor_taskHandle;
const osThreadAttr_t motor_task_attributes = {
  .name = "motor_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for plant_task */
osThreadId_t plant_taskHandle;
const osThreadAttr_t plant_task_attributes = {
  .name = "plant_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for launch_task */
osThreadId_t launch_taskHandle;
const osThreadAttr_t launch_task_attributes = {
  .name = "launch_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for chassis_cmd_queue */
osMessageQueueId_t chassis_cmd_queueHandle;
const osMessageQueueAttr_t chassis_cmd_queue_attributes = {
  .name = "chassis_cmd_queue"
};
/* Definitions for grap_cmd_queue */
osMessageQueueId_t grap_cmd_queueHandle;
const osMessageQueueAttr_t grap_cmd_queue_attributes = {
  .name = "grap_cmd_queue"
};
/* Definitions for motion_arrive_event */
osEventFlagsId_t motion_arrive_eventHandle;
const osEventFlagsAttr_t motion_arrive_event_attributes = {
  .name = "motion_arrive_event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void motor_task_entry(void *argument);
extern void chassis_task_entry(void *argument);
void plant_task_entry(void *argument);
void launch_task_entry(void *argument);

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

  /* Create the queue(s) */
  /* creation of chassis_cmd_queue */
  chassis_cmd_queueHandle = osMessageQueueNew (16, sizeof(ChassisCmd_t), &chassis_cmd_queue_attributes);

  /* creation of grap_cmd_queue */
  grap_cmd_queueHandle = osMessageQueueNew (16, sizeof(GrapCommand_e), &grap_cmd_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motor_task */
  motor_taskHandle = osThreadNew(motor_task_entry, NULL, &motor_task_attributes);

  /* creation of chassis_task */
  chassis_taskHandle = osThreadNew(chassis_task_entry, NULL, &chassis_task_attributes);

  /* creation of plant_task */
  plant_taskHandle = osThreadNew(plant_task_entry, NULL, &plant_task_attributes);

  /* creation of launch_task */
  launch_taskHandle = osThreadNew(launch_task_entry, NULL, &launch_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of motion_arrive_event */
  motion_arrive_eventHandle = osEventFlagsNew(&motion_arrive_event_attributes);

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

/* USER CODE BEGIN Header_motor_task_entry */
/**
* @brief Function implementing the motor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_task_entry */
void motor_task_entry(void *argument)
{
  /* USER CODE BEGIN motor_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_task_entry */
}

/* USER CODE BEGIN Header_plant_task_entry */
/**
* @brief Function implementing the plant_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_plant_task_entry */
void plant_task_entry(void *argument)
{
  /* USER CODE BEGIN plant_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END plant_task_entry */
}

/* USER CODE BEGIN Header_launch_task_entry */
/**
* @brief Function implementing the launch_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_launch_task_entry */
void launch_task_entry(void *argument)
{
  /* USER CODE BEGIN launch_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END launch_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

