/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "CAR_TASK.h"
#include "stdio.h"
#include "echo.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "tim.h"
#include "dma.h"
#include "oled.h"
#include "contrl.h"
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
osThreadId Task200hzHandle;
osThreadId Task100hzHandle;
osThreadId Task10hzHandle;
osThreadId Task_interactioHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start200hzTask(void const * argument);
void StartTask100hz(void const * argument);
void StartTask10hz(void const * argument);
void Start_interaction(void const * argument);

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
  /* definition and creation of Task200hz */
  osThreadDef(Task200hz, Start200hzTask, osPriorityAboveNormal, 0, 128);
  Task200hzHandle = osThreadCreate(osThread(Task200hz), NULL);

  /* definition and creation of Task100hz */
  osThreadDef(Task100hz, StartTask100hz, osPriorityNormal, 0, 128);
  Task100hzHandle = osThreadCreate(osThread(Task100hz), NULL);

  /* definition and creation of Task10hz */
  osThreadDef(Task10hz, StartTask10hz, osPriorityIdle, 0, 128);
  Task10hzHandle = osThreadCreate(osThread(Task10hz), NULL);

  /* definition and creation of Task_interactio */
  osThreadDef(Task_interactio, Start_interaction, osPriorityIdle, 0, 128);
  Task_interactioHandle = osThreadCreate(osThread(Task_interactio), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start200hzTask */
/**
  * @brief  Function implementing the Task200hz thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start200hzTask */
__weak void Start200hzTask(void const * argument)
{
  /* USER CODE BEGIN Start200hzTask */
		vTaskSuspendAll() ;
		
		OLED_Init();
	  OLED_CLS();
		OLED_ShowStr (1,1,"mpu init--",2);
		int cnt = 100;
    while(mpu_dmp_init()&&cnt>0) {//
			printf("mpu init %d\n",cnt);	
			cnt--;
		}
		if(cnt == 0){//mpu初始化失败
			OLED_ShowStr (1,3,"mpu err",2);
			while(1){//死循环
				printf("mpu init err");
			}
		}
		OLED_ShowStr (1,3,"mpu ok",2);
		printf("200HZ start\n");	
		delay_ms(50);	
		OLED_ShowStr (1,3,"m4car start",2);
		OLED_CLS();
		OLED_ShowStr (1,1,"powered by",2);	
		OLED_ShowStr (35,3,"HF",2);
		delay_ms(200);
	
			
		xTaskResumeAll() ;
    /* Infinite loop */
    for(;;)
    {
			vTaskSuspendAll() ;//临界代码保护开始
			Car_Task_200HZ();
			xTaskResumeAll() ; //临界代码保护结束
        osDelay(5);
    }
  /* USER CODE END Start200hzTask */
}

/* USER CODE BEGIN Header_StartTask100hz */
/**
* @brief Function implementing the Task100hz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask100hz */
__weak void StartTask100hz(void const * argument)
{
  /* USER CODE BEGIN StartTask100hz */
    /* Infinite loop */
    for(;;)
    {
			  Car_Task_100HZ();
        osDelay(10);
    }
  /* USER CODE END StartTask100hz */
}

/* USER CODE BEGIN Header_StartTask10hz */
/**
* @brief Function implementing the Task10hz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask10hz */
__weak void StartTask10hz(void const * argument)
{
  /* USER CODE BEGIN StartTask10hz */
    /* Infinite loop */
    for(;;)
    {
			 Car_Task_10HZ();
        osDelay(100); 
    }
  /* USER CODE END StartTask10hz */
}

/* USER CODE BEGIN Header_Start_interaction */
/**
* @brief Function implementing the Task_interactio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_interaction */
__weak void Start_interaction(void const * argument)
{
  /* USER CODE BEGIN Start_interaction */
	vTaskSuspendAll() ;
	Car_Task_Interaction_begin();
	xTaskResumeAll() ; //临界代码保护结束
    /* Infinite loop */
    for(;;)
    {
			vTaskSuspendAll() ;//临界代码保护开始
			Car_Task_Interaction();
			xTaskResumeAll() ; //临界代码保护结束
        osDelay(100);
    }
  /* USER CODE END Start_interaction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

