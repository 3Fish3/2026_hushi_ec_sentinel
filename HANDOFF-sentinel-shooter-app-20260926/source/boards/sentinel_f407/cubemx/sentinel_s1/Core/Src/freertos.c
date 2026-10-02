/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "s1_diagnostics.h"
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
/* Definitions for s1Task */
osThreadId_t s1TaskHandle;
const osThreadAttr_t s1Task_attributes = {
  .name = "s1Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#if EC_SENTINEL_APP
/* 正式 APP 的 1 kHz 控制任务: 静态栈与控制块, 不占用 FreeRTOS heap, 也不依赖运行期分配.
   优先级高于 10 ms 的 s1Task, 使 CAN 收帧/控制/提交不被监控与串口服务拖延.
   栈大小是静态预算 (4096 B); 实测高水位由 g_ec_s1.stack_free_words 记录, 待上板核对. */
#define EC_APP_CONTROL_STACK_WORDS 1024
static StackType_t appControlStack[EC_APP_CONTROL_STACK_WORDS];
static StaticTask_t appControlTaskBuffer;
osThreadId_t appControlTaskHandle;
const osThreadAttr_t appControlTask_attributes = {
  .name = "appControl",
  .cb_mem = &appControlTaskBuffer,
  .cb_size = sizeof(appControlTaskBuffer),
  .stack_mem = appControlStack,
  .stack_size = sizeof(appControlStack),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
void StartAppControlTask(void *argument);
#endif

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#if EC_SENTINEL_APP
/* 正式 APP 的控制任务入口: 1 ms 周期与每周期有界预算由 board_app.cpp 实现侧保持
   (C++ 接口不能直接出现在本文件里). */
void ec_app_control_task_entry(void);
#endif
/* USER CODE END FunctionPrototypes */

void StartS1Task(void *argument);

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
  /* creation of s1Task */
  s1TaskHandle = osThreadNew(StartS1Task, NULL, &s1Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (s1TaskHandle == NULL) {
    ec_s1_fail(EC_S1_TASK_CREATE, __FILE__, __LINE__);
  }
#if EC_SENTINEL_APP
  /* 正式 APP: 控制任务必须在调度器启动前创建, 应用对象已在 ec_board_io_init() 构造完毕. */
  appControlTaskHandle = osThreadNew(StartAppControlTask, NULL, &appControlTask_attributes);
  if (appControlTaskHandle == NULL) {
    ec_s1_fail(EC_S1_TASK_CREATE, __FILE__, __LINE__);
  }
#endif
  ec_s1_stage(EC_S1_TASKS_READY);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartS1Task */
/**
  * @brief  Function implementing the s1Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartS1Task */
void StartS1Task(void *argument)
{
  /* USER CODE BEGIN StartS1Task */
  ec_s1_task(argument);
  /* USER CODE END StartS1Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#if EC_SENTINEL_APP
/* 控制任务: 不等待外部事件, 周期与工作量全部由实现侧的有界预算决定. */
void StartAppControlTask(void *argument)
{
  (void)argument;
  ec_app_control_task_entry();
}
#endif
/* USER CODE END Application */

