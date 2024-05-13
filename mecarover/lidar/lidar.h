/*
 * LaserScanner.h
 *
 *  Created on: Feb 27, 2023
 *      Author: fabian
 */

#ifndef LASERSCANNER_LASERSCANNER_H_
#define LASERSCANNER_LASERSCANNER_H_


#include <stdint.h>

#include <RTOS.h>
#include <rtos_config.h>

#include <mecarover/mrlogger/mrlogger.h>

class LaserScanner {
private:
	RT_Mutex sensMut;
	RT_Task laserscanner_thread;

public:
	float dist[360];
	float ang[360];
	float quality[360];
	float adjustedScan[360];
	float erg[360];

	uint8_t rplidar_reset[2] = { 0xA5, 0x40 };
	uint8_t rplidar_legacy_scan[9] = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };
	uint8_t rplidar_start_scan[2] = { 0xA5, 0x20};
	uint8_t rplidar_RPM[6] = { 0xA5, 0xA8, 0x02, 0x00, 0x00, 0x00};
	uint8_t rplidar_stop_scan[2] = { 0xA5, 0x25 };
	uint8_t rplidar_get_Health[2] = { 0xA5, 0x52 };

	void LaserScannerTask(LaserScanner *scan);
	void init_LaserScanner(LaserScanner *scanner);
	void Stop();
	void Start();
	void invert(float arr[], float erg[], int n);
};
#endif /* LASERSCANNER_LASERSCANNER_H_ */
