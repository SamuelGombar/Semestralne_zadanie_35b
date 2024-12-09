/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "sensor.h"

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return 0; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	return 0; // to be implemented
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	return 0; // to be implemented
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	return 0; // to be implemented
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	return 0; // to be implemented
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	int status = 0;
	uint8_t data_read = 0;
	data_read = i2c_read(dev, index, 1);
	if (data_read == 0) {
		status = 1;
	}
	*data = data_read;
	return status; // to be implemented
}

//int _I2CRead(uint16_t Dev, uint8_t *pdata, uint32_t count) {
//    int status;
//    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;
//
//    status = HAL_I2C_Master_Receive(&XNUCLEO53L1A1_hi2c, Dev|1, pdata, count, i2c_time_out);
//    if (status) {
//        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
//        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
//    }
//    return status;
//}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	return 0; // to be implemented
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	return 0; // to be implemented
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	return 0; // to be implemented
}
