/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os2.h"
#include "dma.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FourWheelMecanumControllerTasks.h"
#include "DiffDriveControllerTasks.h"
#include "ros_interface.h"
#include "rtos_config.h"
#include "robot_config.h"

using namespace imsl;
using namespace imsl::vehiclecontrol;

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

/* USER CODE BEGIN PV */
FourWheelMecanumControllerTasks<real_t> myController; // controller for 4 wheel Mecanum robot
ControllerTasksInterfaces<real_t> *controllerTasks = &myController;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_TIM9_Init();
	MX_DMA_Init();
	/* USER CODE BEGIN 2 */

	logger_init();   // init the screen logger before the other components

	if (hal_init (&fz)) { // init PWM and encoders
		log_message(log_info, "HAL: initialization ok");
	} else {
		log_message(log_error, "HAL: initialization failed");
	}

	if (fz.type == MRC_VEHICLETYPE_MECANUM) {
		// controller for 4 wheel Mecanum robot like OmniRob, Nexus or FILU
		controllerTasks = new FourWheelMecanumControllerTasks<real_t>; // myController;
	} else {
		// differential drive robot like ADRZ D4
		controllerTasks = new DiffDriveControllerTasks<real_t>; // myController;
	}

	controllerTasks->Init(&fz, Regler, Ta); // init controllers

	// init Ros Task
	log_message(log_info, "starting ROS");

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();
	BaseType_t xReturned;
	xReturned = xTaskCreate(rosInit, "executor", 3000, controllerTasks, MICRO_ROS_TASK_PRIORITY, NULL);
	if (xReturned != pdPASS) {
		//    Serial.println("Error: logger_init(), xTaskCreate()");
		printf("Error: logger_init(), xTaskCreate()\n");
	}
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	size_t free_heap = xPortGetMinimumEverFreeHeapSize(); //ESP.getMinFreeHeap(); //lowest level of free heap since boot
	uint32_t free_stack = RT_Task::thisTaskGetStackHighWaterMark();
	log_message(log_info,
			"running into main loop, free heap: %d, free stack: %d", free_heap,
			free_stack);
	//    RT_PeriodicTimer loopTimer(500); // wait period 500 ms = 2 Hz loop frequency
	RT_PeriodicTimer loopTimer(Ta.FzLage * 1000); // wait period is pose controller period
	controllerTasks->SetControllerMode(vehiclecontrol::CtrlMode::OFF);
	loopTimer.init();
	int loopCounter = 0;

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		char msg_buffer[100];
		// output to terminal
		if (loopCounter++ > 1.0 / Ta.FzLage) {
			loopCounter = 0;
			PoseV_t p;
			controllerTasks->GetPose(&p);
			CtrlMode mode = controllerTasks->GetControllerMode();
			const char *mode_str = "";

			switch (mode) {
			case CtrlMode::ESTOP:
				mode_str = "ESTOP";
				break;
			case CtrlMode::OFF:
				mode_str = "OFF";
				break;
			case CtrlMode::TWIST:
				mode_str = "TWIST";
				break;
			case CtrlMode::POSE:
				mode_str = "POSE";
				break;
			}

			sprintf(msg_buffer, "x: %f, y: %f, theta: %f", p.x, p.y, p.theta);

			log_message(log_debug, msg_buffer); // write pose to logger

			log_message(log_info, "ROS: Mode: %s", mode_str);

		}
		loopTimer.wait();

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM12 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM12) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

