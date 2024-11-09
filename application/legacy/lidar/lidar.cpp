/*
 * LaserScanner.cc
 *
 *  Created on: Feb 27, 2023
 *      Author: fabian
 */

#include "lidar.h"

#include <math.h>
#include <tim.h>
#include <ulog.h>
#include <usart.h>

#define Startflag 0x01
#define InStartflag 0x02
#define Qualitiy 0x03 & 0x04 & 0x05 & 0x06 & 0x07 & 0x08
#define Checkbit 0x01

extern UART_HandleTypeDef huart2;
extern LaserScanner lidar;

uint8_t start_cmd[2] = {0xA5, 0x20};
uint8_t stop_cmd[2] = {0xA5, 0x25};
uint8_t reset_cmd[2] = {0xA5, 0x40};
uint8_t get_health_cmd[2] = {0xA5, 0x52};
uint8_t legacy_scan[9] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};

bool shouldReadData = false;
int zahl = 0;
int y = 0;

void call_laser_scanner_task(void *arg) {
  LaserScanner *scan = (LaserScanner *)arg;
  scan->laser_scanner_task();
}

void LaserScanner::laser_scanner_task() {
  ULOG_INFO("LaserScanner Task");
  while (true) {
    uint8_t received_bytes[1800] = {0};

    unsigned int checkbit;

    unsigned int startflag;
    unsigned int Instartflag;

    uint8_t temp1, temp2;

    while (shouldReadData) {
      HAL_UART_Receive_DMA(&huart2, (uint8_t *)received_bytes,
                           sizeof(received_bytes));

      int i = 0;
      while (i < 1800) {
        // Assign values ​​for the check bits
        checkbit = received_bytes[1 + i] & 0x01;
        startflag = received_bytes[0 + i] & 0x01;
        Instartflag = received_bytes[0 + i] & 0x02;
        // If the counter is at the beginning of the data packet, the check bit
        // and start or the inverted start flag must be set to 1.
        if (checkbit == 1 && ((startflag == 1 && Instartflag != 2) ||
                              (startflag != 1 && Instartflag == 2))) {
          // Data can be read and stored in arrays.
          float quality = (received_bytes[0 + i]) >> 2;

          temp1 = received_bytes[1 + i];
          temp2 = received_bytes[2 + i];
          float actualAngle = (((temp2) << 7) | (temp1 >> 1)) / 64.0f;

          temp1 = received_bytes[3 + i];
          temp2 = received_bytes[4 + i];
          float distanceInMeters = (((temp2) << 8) + (temp1)) / 4.0f / 1000.0f;

          int angleInt = round(actualAngle);

          if (angleInt >= 360) {
            angleInt -= 360;
          } else if (angleInt < 0) {
            angleInt += 360;
          }

          lidar.adjustedScan[angleInt] = distanceInMeters;

          if (zahl == 360) {
            zahl = 0;
          }

          lidar.quality[zahl] = quality;
          //					ls.dist[zahl] =
          // distanceInMeters; 					ls.ang[zahl++] =
          // actualAngle;
          i += 5;
        } else {
          i += 1;
        }
      }

      vTaskDelay(1);
    }

    vTaskDelay(1);
  }
}

void LaserScanner::init() {
  // start the engine of the laser scanner
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

void LaserScanner::stop() {
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)stop_cmd, 2);
  shouldReadData = false;
}

void LaserScanner::Start() {
  // clear the overrun and noise interrupt flag
  __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 5000);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)start_cmd, 2);
  shouldReadData = true;
}

void LaserScanner::invert(float arr[], float result[], int n) {
  int j = n;
  for (int i = 0; i <= n; i++) {
    result[i] = arr[j--];
  }
}
