/*
 * main.cc
 *
 *  Created on: 15.11.2021
 *      Author: Fabia
 */
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdio.h>

#include <main.h>
#include <gpio.h>
#include <tim.h>
#include <dma.h>
#include <usart.h>
#include <cmsis_os2.h>
#include <stm32f7xx_hal.h>
#include <rtos_config.h>

#include <mecarover/robot_config.h>
#include <mecarover/retarget.h>
#include <mecarover/lidar/lidar.h>
#include <mecarover/hal/stm_hal.h>
#include <mecarover/microros_init/microros_init.h>
#include <mecarover/controls/FourWheelMecanumControllerTasks.h>
#include <mecarover/controls/DiffDriveControllerTasks.h>

real_t test;
LaserScanner ls;

using namespace imsl;
using namespace imsl::vehiclecontrol;

FourWheelMecanumControllerTasks<real_t> myController; // controller for 4 wheel Mecanum robot
ControllerTasksInterfaces<real_t> *controllerTasks = &myController;
bool hal_is_init = false;

extern "C"
{
extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

int main()
{

	logger_init(); // init the screen logger before the other components
	HAL_Init(); //init aus main-Methode
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_TIM9_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_TIM11_Init();
	MX_TIM13_Init();

	retarget_init(&huart3);

	if (hal_init(&fz)) { // init PWM and encoders
		hal_is_init = true;
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

	//init LaserScanner
	ls.init_LaserScanner(&ls);

	log_message(log_info, "starting ROS");
	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */

	BaseType_t xReturned;
	xReturned = xTaskCreate(rosInit, "executor", STACK_SIZE, controllerTasks,
			(osPriority_t)MICRO_ROS_TASK_PRIORITY, NULL);
	if (xReturned != pdPASS) {
		printf("Error: logger_init(), xTaskCreate()\n");
	}

	size_t free_heap = xPortGetMinimumEverFreeHeapSize(); //ESP.getMinFreeHeap(); //lowest level of free heap since boot
	uint32_t free_stack = RT_Task::thisTaskGetStackHighWaterMark();
	log_message(log_info,"running into main loop, free heap: %d, free stack: %lu\r\n",
			free_heap, free_stack);
	//    RT_PeriodicTimer loopTimer(500); // wait period 500 ms = 2 Hz loop frequency
	RT_PeriodicTimer loopTimer(Ta.FzLage * 1000); // wait period is pose controller period
	controllerTasks->SetControllerMode(vehiclecontrol::CtrlMode::OFF);
	loopTimer.init();
	int loopCounter = 0;

	/* Start scheduler */
	osKernelStart();
	while (true) {
		log_message(log_info, "main while Log Message Test\r\n");

		hal_encoder_read(&test); //Test der Encoder

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

			printf("main while loop \r\n");


			log_message(log_debug, "%s", msg_buffer); // write pose to logger
			log_message(log_info, "ROS: Mode: %s", mode_str);
		}
		loopTimer.wait();
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	*/
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
}


/* USER CODE BEGIN 4 */
volatile unsigned long ulHighFrequencyTimerTicks;

void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0;
	HAL_TIM_Base_Start_IT(&htim13);
}

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
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
}
