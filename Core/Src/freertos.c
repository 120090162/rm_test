/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "Music_Task.h"
#include "Led_Flow_Task.h"
#include "Detect_Task.h"
#include "INS_Task.h"
#include "Observe_Task.h"

#include "CAN_Task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_SYSVIEW_FreeRTOS.h"

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
// ins task
osThreadId Start_INS_TaskHandle;
uint32_t Start_INS_TaskBuffer[2048];
osStaticThreadDef_t Start_INS_TaskControlBlock;
// observe task
osThreadId Start_Observe_TaskHandle;
uint32_t Start_Observe_TaskBuffer[2048];
osStaticThreadDef_t Start_Observe_TaskControlBlock;
// control task
osThreadId Start_Control_TaskHandle;
uint32_t Start_Control_TaskBuffer[2048];
osStaticThreadDef_t Start_Control_TaskControlBlock;
osThreadId Start_CAN_TaskHandle;
uint32_t Start_CAN_TaskBuffer[2048];
osStaticThreadDef_t Start_CAN_TaskControlBlock;
// detect task
osThreadId Start_Detect_TaskHandle;
uint32_t Start_Detect_TaskBuffer[2048];
osStaticThreadDef_t Start_Detect_TaskControlBlock;
// music task
osThreadId Start_Music_TaskHandle;
uint32_t Start_Music_TaskBuffer[256];
osStaticThreadDef_t Start_Music_TaskControlBlock;
// LED RGB flow task
osThreadId Start_Led_RGB_Flow_TaskHandle;
uint32_t Start_Led_RGB_Flow_TaskBuffer[256];
osStaticThreadDef_t Start_Led_RGB_Flow_TaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_TASK(void const *argument);
void Control_Task(void const *argument);
void CAN_TASK(void const *argument);
void Detect_TASK(void const *argument);
void Music_TASK(void const *argument);
void Observe_TASK(void const *argument);
void Led_RGB_Flow_TASK(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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
	/* definition and creation of Start_Detect_Task */
	osThreadStaticDef(Start_Detect_Task, Detect_TASK, osPriorityBelowNormal, 0, 2048, Start_Detect_TaskBuffer, &Start_Detect_TaskControlBlock);
	Start_Detect_TaskHandle = osThreadCreate(osThread(Start_Detect_Task), NULL);

	/* definition and creation of Start_INS_Task */
	osThreadStaticDef(Start_INS_Task, INS_TASK, osPriorityRealtime, 0, 2048, Start_INS_TaskBuffer, &Start_INS_TaskControlBlock);
	Start_INS_TaskHandle = osThreadCreate(osThread(Start_INS_Task), NULL);

	/* definition and creation of Start_Observe_Task */
	osThreadStaticDef(Start_Observe_Task, Observe_TASK, osPriorityHigh, 0, 2048, Start_Observe_TaskBuffer, &Start_Observe_TaskControlBlock);
	Start_Observe_TaskHandle = osThreadCreate(osThread(Start_Observe_Task), NULL);

	// /* definition and creation of Start_Control_Task */
	// osThreadStaticDef(Start_Control_Task, Control_TASK, osPriorityAboveNormal, 0, 2048, Start_Control_TaskBuffer, &Start_Control_TaskControlBlock);
	// Start_Control_TaskHandle = osThreadCreate(osThread(Start_Control_Task), NULL);

	// /* definition and creation of Start_CAN_Task */
	osThreadStaticDef(Start_CAN_Task, CAN_TASK, osPriorityNormal, 0, 2048, Start_CAN_TaskBuffer, &Start_CAN_TaskControlBlock);
	Start_CAN_TaskHandle = osThreadCreate(osThread(Start_CAN_Task), NULL);

	/* definition and creation of Start_Music_Task */
	osThreadStaticDef(Start_Music_Task, Music_TASK, osPriorityNormal, 0, 256, Start_Music_TaskBuffer, &Start_Music_TaskControlBlock);
	Start_Music_TaskHandle = osThreadCreate(osThread(Start_Music_Task), NULL);

	/* definition and creation of Start_Led_RGB_Flow_Task */
	osThreadStaticDef(Start_Led_RGB_Flow_Task, Led_RGB_Flow_TASK, osPriorityLow, 0, 256, Start_Led_RGB_Flow_TaskBuffer, &Start_Led_RGB_Flow_TaskControlBlock);
	Start_Led_RGB_Flow_TaskHandle = osThreadCreate(osThread(Start_Led_RGB_Flow_Task), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_INS_Task */
/**
 * @brief  Function implementing the StartINS thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_INS_Task */
void INS_TASK(void const *argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN INS_Task */
	/* Infinite loop */
	for (;;)
	{
		INS_task();
	}
	/* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
 * @brief Function implementing the StartControl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Control_Task */
void Control_TASK(void const *argument)
{
	/* USER CODE BEGIN Control_Task */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
 * @brief Function implementing the StartCAN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_Task */
void CAN_TASK(void const *argument)
{
	/* USER CODE BEGIN CAN_Task */
	/* Infinite loop */
	for (;;)
	{
		CAN_Task();
	}
	/* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
 * @brief Function implementing the Start_Detect_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Detect_Task */
void Detect_TASK(void const *argument)
{
	/* USER CODE BEGIN Detect_Task */
	/* Infinite loop */
	for (;;)
	{
		Detect_Task();
	}
	/* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_Music_Task */
/**
 * @brief Function implementing the Start_Music_Task thread.
 * @param argument: Not used
 * @retval None
 */
void Music_TASK(void const *argument)
{
	/* USER CODE BEGIN Music_Task */
	/* Infinite loop */
	for (;;)
	{
		Music_Task();
	}
	/* USER CODE END Music_Task */
}

/* USER CODE BEGIN Header_Led_RGB_Flow_Task */
/**
 * @brief Function implementing the Led_RGB_Flow_Task thread.
 * @param argument: Not used
 * @retval None
 */
void Led_RGB_Flow_TASK(void const *argument)
{
	/* USER CODE BEGIN Led_RGB_Flow_Task */
	/* Infinite loop */
	for (;;)
	{
		led_RGB_flow_task();
	}
	/* USER CODE END Led_RGB_Flow_Task */
}

/* USER CODE BEGIN Header_Observe_TASK */
/**
 * @brief Function implementing the Observe_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Observe_TASK */
void Observe_TASK(void const *argument)
{
	/* USER CODE BEGIN Observe_TASK */
	/* Infinite loop */
	for (;;)
	{
		Observe_task();
	}
	/* USER CODE END Observe_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
