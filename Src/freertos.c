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
#include "ins_task.h"
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
osThreadId GIMBAL_TaskHandle;
osThreadId INSTaskHandle;
osThreadId current_sendHandle;
osThreadId MODE_TASKHandle;
osThreadId VisionTaskHandle;
osThreadId REFEREETaskHandle;
osThreadId detect_taskHandle;
osThreadId VOFATaskHandle;
osThreadId SHOOT_TASKHandle;
osThreadId TransmitTaskHandle;
osThreadId TRIGGERTASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void gimbal_task(void const * argument);
void StartINSTask(void const * argument);
void send_current_task(void const * argument);
void mode_task(void const * argument);
void vision_task(void const * argument);
void REFEREE_Task(void const * argument);
void DETECT_task(void const * argument);
void StartVOFATask(void const * argument);
void shoot_task(void const * argument);
void Transmit_Data_Task(void const * argument);
void trigger_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of GIMBAL_Task */
  osThreadDef(GIMBAL_Task, gimbal_task, osPriorityHigh, 0, 1024);
  GIMBAL_TaskHandle = osThreadCreate(osThread(GIMBAL_Task), NULL);

  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityHigh, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of current_send */
  osThreadDef(current_send, send_current_task, osPriorityHigh, 0, 512);
  current_sendHandle = osThreadCreate(osThread(current_send), NULL);

  /* definition and creation of MODE_TASK */
  osThreadDef(MODE_TASK, mode_task, osPriorityAboveNormal, 0, 512);
  MODE_TASKHandle = osThreadCreate(osThread(MODE_TASK), NULL);

  /* definition and creation of VisionTask */
  osThreadDef(VisionTask, vision_task, osPriorityAboveNormal, 0, 512);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  /* definition and creation of REFEREETask */
  osThreadDef(REFEREETask, REFEREE_Task, osPriorityAboveNormal, 0, 512);
  REFEREETaskHandle = osThreadCreate(osThread(REFEREETask), NULL);

  /* definition and creation of detect_task */
  osThreadDef(detect_task, DETECT_task, osPriorityAboveNormal, 0, 256);
  detect_taskHandle = osThreadCreate(osThread(detect_task), NULL);

  /* definition and creation of VOFATask */
  osThreadDef(VOFATask, StartVOFATask, osPriorityAboveNormal, 0, 256);
  VOFATaskHandle = osThreadCreate(osThread(VOFATask), NULL);

  /* definition and creation of SHOOT_TASK */
  osThreadDef(SHOOT_TASK, shoot_task, osPriorityNormal, 0, 512);
  SHOOT_TASKHandle = osThreadCreate(osThread(SHOOT_TASK), NULL);

  /* definition and creation of TransmitTask */
  osThreadDef(TransmitTask, Transmit_Data_Task, osPriorityAboveNormal, 0, 512);
  TransmitTaskHandle = osThreadCreate(osThread(TransmitTask), NULL);

  /* definition and creation of TRIGGERTASK */
  osThreadDef(TRIGGERTASK, trigger_task, osPriorityAboveNormal, 0, 512);
  TRIGGERTASKHandle = osThreadCreate(osThread(TRIGGERTASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_gimbal_task */
/**
  * @brief  Function implementing the GIMBAL_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the INSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* USER CODE BEGIN StartINSTask */
  /* Infinite loop */
	INS_Init();
  for(;;)
  {
		INS_Task();
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_send_current_task */
/**
* @brief Function implementing the current_send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_current_task */
__weak void send_current_task(void const * argument)
{
  /* USER CODE BEGIN send_current_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END send_current_task */
}

/* USER CODE BEGIN Header_mode_task */
/**
* @brief Function implementing the MODE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mode_task */
__weak void mode_task(void const * argument)
{
  /* USER CODE BEGIN mode_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END mode_task */
}

/* USER CODE BEGIN Header_vision_task */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vision_task */
__weak void vision_task(void const * argument)
{
  /* USER CODE BEGIN vision_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vision_task */
}

/* USER CODE BEGIN Header_REFEREE_Task */
/**
* @brief Function implementing the REFEREETask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_REFEREE_Task */
__weak void REFEREE_Task(void const * argument)
{
  /* USER CODE BEGIN REFEREE_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END REFEREE_Task */
}

/* USER CODE BEGIN Header_DETECT_task */
/**
* @brief Function implementing the detect_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DETECT_task */
__weak void DETECT_task(void const * argument)
{
  /* USER CODE BEGIN DETECT_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DETECT_task */
}

/* USER CODE BEGIN Header_StartVOFATask */
/**
* @brief Function implementing the VOFATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVOFATask */
__weak void StartVOFATask(void const * argument)
{
  /* USER CODE BEGIN StartVOFATask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartVOFATask */
}

/* USER CODE BEGIN Header_shoot_task */
/**
* @brief Function implementing the SHOOT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot_task */
__weak void shoot_task(void const * argument)
{
  /* USER CODE BEGIN shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END shoot_task */
}

/* USER CODE BEGIN Header_Transmit_Data_Task */
/**
* @brief Function implementing the TransmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Transmit_Data_Task */
__weak void Transmit_Data_Task(void const * argument)
{
  /* USER CODE BEGIN Transmit_Data_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Transmit_Data_Task */
}

/* USER CODE BEGIN Header_trigger_task */
/**
* @brief Function implementing the TRIGGERTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_trigger_task */
__weak void trigger_task(void const * argument)
{
  /* USER CODE BEGIN trigger_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END trigger_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
