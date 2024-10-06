/*
 * LaserScanner.h
 *
 *  Created on: Feb 27, 2023
 *      Author: fabian
 */

#pragma once

#include <stdint.h>

#include <mecarover/rtos_config.h>
#include <mecarover/rtos_utils.h>

#include <mecarover/mrlogger/mrlogger.h>

extern "C"
{
void LaserScannerTaskFunction(void *arg);
};

class LaserScanner {
private:
	RT_Mutex sensMut;
	RT_Task laserscanner_thread{LaserScannerTaskFunction, "LaserScanner",
								SCAN_STACK_SIZE, this, SCAN_TASK_PRIORITY};

public:
	float dist[360];
	float ang[360];
	float quality[360];
	float adjustedScan[360];
	float erg[360];

	uint8_t rplidar_reset[2] = { 0xA5, 0x40 };
	uint8_t rplidar_legacy_scan[9] = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };
	uint8_t rplidar_start_scan[2] = { 0xA5, 0x20 };
	uint8_t rplidar_RPM[6] = { 0xA5, 0xA8, 0x02, 0x00, 0x00, 0x00 };
	uint8_t rplidar_stop_scan[2] = { 0xA5, 0x25 };
	uint8_t rplidar_get_Health[2] = { 0xA5, 0x52 };

	void LaserScannerTask();
	void init_LaserScanner();
	void Stop();
	void Start();
	void invert(float arr[], float erg[], int n);
};
