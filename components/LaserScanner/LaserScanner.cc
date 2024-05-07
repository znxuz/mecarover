/*
 * LaserScanner.cc
 *
 *  Created on: Feb 27, 2023
 *      Author: fabian
 */

#include "LaserScanner.h"
#include <time.h>
#include <math.h>

#define Startflag 0x01
#define InStartflag 0x02
#define Qualitiy 0x03 & 0x04 & 0x05 & 0x06 & 0x07 & 0x08
#define Checkbit 0x01

extern UART_HandleTypeDef huart2;
extern LaserScanner ls;


uint8_t start[2] = { 0xA5, 0x20};
uint8_t stop[2] = { 0xA5, 0x25};
uint8_t reset[2] = { 0xA5, 0x40};
uint8_t getHealth[2] = { 0xA5, 0x52};
uint8_t legacy_scan[9] = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };


bool shouldReadData = false;
int zahl = 0;
int y = 0;


void LaserScannerTaskFunction(void *arg) {
	LaserScanner *scan = (LaserScanner*) arg;
	  log_message(log_info, "LaserScanner Task");
	  scan->LaserScannerTask(scan);
}

void LaserScanner::LaserScannerTask(LaserScanner *scan) {

	while (true) {

		char receivedBytes[1800] = {0};

		int bytesRead;

		unsigned int checkbit;

		unsigned int startflag;
		unsigned int Instartflag;

		uint8_t temp1, temp2;

		//when the RPLidar has been started, the data can be read
		while (shouldReadData) {
			//Read bytes of laser data
			bytesRead = HAL_UART_Receive_DMA(&huart2, (uint8_t*) receivedBytes,	sizeof(receivedBytes));

			int i = 0;
			while(i<1800){
			//Assign values ​​for the check bits
			checkbit = receivedBytes[1 + i] & 0x01;
			startflag = receivedBytes[0 + i] & 0x01;
			Instartflag = receivedBytes[0 + i] & 0x02;
			//If the counter is at the beginning of the data packet, the check bit and start or the inverted start flag must be set to 1.
			if(checkbit == 1 && ((startflag == 1 && Instartflag != 2) || (startflag != 1 && Instartflag == 2))){
				//Data can be read and stored in arrays.
				float quality = (receivedBytes[0 + i]) >> 2;

				temp1 = receivedBytes[1 + i];
				temp2 = receivedBytes[2 + i];
				float actualAngle = (((temp2) << 7) | (temp1 >> 1)) / 64.0f;

				temp1 = receivedBytes[3 + i];
				temp2 = receivedBytes[4 + i];
				float distanceInMeters =	(((temp2) << 8) + (temp1)) / 4.0f / 1000.0f;


					int angleInt = round(actualAngle);

					if (angleInt >= 360) {
						angleInt -= 360;
					} else if (angleInt < 0) {
						angleInt += 360;
					}

					ls.adjustedScan[angleInt] = distanceInMeters;

					if (zahl == 360) {
						zahl = 0;
					}

					ls.quality[zahl] = quality;
//					ls.dist[zahl] = distanceInMeters;
//					ls.ang[zahl++] = actualAngle;
					i += 5;
				} else {
					i += 1;
				}


			}

			osDelay(1);

		}

		osDelay(1);

	}

}

void LaserScanner::init_LaserScanner(LaserScanner *scanner) {

	log_message(log_info, "Create LaserScanner");

	//start the engine of the laser scanner
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);


	if (!laserscanner_thread.create(LaserScannerTaskFunction, "LaserScanner",
			3000, this, (osPriority_t) LaserScanner_TASK_PRIORITY)) {

	} else {
		log_message(log_info, "Create LaserScanner failed");

	}
}

void LaserScanner::Stop(){
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)stop, 2);
	shouldReadData = false;

}

void LaserScanner::Start(){
	//clear the overrun and noise interrupt flag
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 5000);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)start, 2);
	shouldReadData = true;

}

void LaserScanner::invert(float arr[], float erg[], int n){
	int j = n;
	for(int i = 0; i <= n; i++){
		erg[i] = arr[j--];
	}
}
