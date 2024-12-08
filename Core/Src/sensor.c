/*
 * sensor.c
 *
 *  Created on: Dec 7, 2024
 *      Author: samog
 */


#include "sensor.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "vl53l1_platform.h"

int continuous_ranging() {
	uint8_t byteData, sensorState=1;
	uint16_t wordData;
	uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
	uint16_t distance;
	uint16_t signalRate;
	uint16_t ambientRate;
	uint16_t spadNum;
	uint8_t rangeStatus;
	uint8_t dataReady;
	uint8_t status;

//	while(sensorState) {
//		status = VL53L1X_BootState(MAIN_SENSOR_ADDRESS, &sensorState);
//		LL_mDelay(2);
//	}

	status = VL53L1X_SensorInit(MAIN_SENSOR_ADDRESS);
	status = VL53L1X_StartRanging(MAIN_SENSOR_ADDRESS);
	while(1) {
		while(dataReady == 0) {
			status = VL53L1X_CheckForDataReady(MAIN_SENSOR_ADDRESS, &dataReady);
		}
		dataReady = 0;
		status = VL53L1X_GetRangeStatus(MAIN_SENSOR_ADDRESS, &rangeStatus);
		status = VL53L1X_GetDistance(MAIN_SENSOR_ADDRESS, &distance);
		status = VL53L1X_ClearInterrupt(MAIN_SENSOR_ADDRESS);
	}
}
