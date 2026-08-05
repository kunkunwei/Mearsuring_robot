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
osThreadId StartINSTaskHandle;
osThreadId StartUserTaskHandle;
osThreadId StartCanTaskHandle;
osThreadId StartObserveTasHandle;
osThreadId StartRosTaskHandle;
osThreadId StartChassisTasHandle;
osThreadId StartUltrasonicHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void User_Task(void const * argument);
void Can_Task(void const * argument);
void ObserveTask(void const * argument);
void Ros_Task(void const * argument);
void Chassis_Task(void const * argument);
void Ultrasonic_Task(void const * argument);

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
  /* definition and creation of StartINSTask */
  osThreadDef(StartINSTask, INS_Task, osPriorityRealtime, 0, 256);
  StartINSTaskHandle = osThreadCreate(osThread(StartINSTask), NULL);

  /* definition and creation of StartUserTask */
  osThreadDef(StartUserTask, User_Task, osPriorityIdle, 0, 256);
  StartUserTaskHandle = osThreadCreate(osThread(StartUserTask), NULL);

  /* definition and creation of StartCanTask */
  osThreadDef(StartCanTask, Can_Task, osPriorityHigh, 0, 256);
  StartCanTaskHandle = osThreadCreate(osThread(StartCanTask), NULL);

  /* definition and creation of StartObserveTas */
  osThreadDef(StartObserveTas, ObserveTask, osPriorityIdle, 0, 256);
  StartObserveTasHandle = osThreadCreate(osThread(StartObserveTas), NULL);

  /* definition and creation of StartRosTask */
  osThreadDef(StartRosTask, Ros_Task, osPriorityNormal, 0, 256);
  StartRosTaskHandle = osThreadCreate(osThread(StartRosTask), NULL);

  /* definition and creation of StartChassisTas */
  osThreadDef(StartChassisTas, Chassis_Task, osPriorityAboveNormal, 0, 512);
  StartChassisTasHandle = osThreadCreate(osThread(StartChassisTas), NULL);

  /* definition and creation of StartUltrasonic */
  osThreadDef(StartUltrasonic, Ultrasonic_Task, osPriorityHigh, 0, 256);
  StartUltrasonicHandle = osThreadCreate(osThread(StartUltrasonic), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_User_Task */
/**
* @brief Function implementing the StartUserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_User_Task */
__weak void User_Task(void const * argument)
{
  /* USER CODE BEGIN User_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END User_Task */
}

/* USER CODE BEGIN Header_Can_Task */
/**
* @brief Function implementing the StartCanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_Task */
__weak void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Can_Task */
}

/* USER CODE BEGIN Header_ObserveTask */
/**
* @brief Function implementing the StartObserveTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ObserveTask */
__weak void ObserveTask(void const * argument)
{
  /* USER CODE BEGIN ObserveTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ObserveTask */
}

/* USER CODE BEGIN Header_Ros_Task */
/**
* @brief Function implementing the StartRosTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ros_Task */
__weak void Ros_Task(void const * argument)
{
  /* USER CODE BEGIN Ros_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ros_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the StartChassisTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Ultrasonic_Task */
/**
* @brief Function implementing the StartUltrasonic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ultrasonic_Task */
__weak void Ultrasonic_Task(void const * argument)
{
  /* USER CODE BEGIN Ultrasonic_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ultrasonic_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
