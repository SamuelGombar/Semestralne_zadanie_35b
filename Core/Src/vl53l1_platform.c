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
#include "stm32f3xx_ll_utils.h"

//WrWord
//dev - addresa slave
//index - addresa registra
//*data je pamapť a nie array - toto zapisovať

//Read
//dev - addresa slave
//index - addresa registra
//*data je pamapť a nie array - tu zapisovať

void RegisterCallback_i2c_mread_single(void *callback) {
        if(callback != 0) i2c_mread_single = callback;
}
void RegisterCallback_i2c_mread_multi(void *callback) {
        if(callback != 0) i2c_mread_multi = callback;
}
void RegisterCallback_i2c_mwrite(void *callback) {
        if(callback != 0) i2c_mwrite = callback;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	int8_t status = 1;
	status = i2c_mwrite(pdata, count, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	int8_t status = 1;
	status = i2c_mread_multi(pdata, count, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	int8_t status = 1;
	status = i2c_mwrite((uint8_t*)(&data), 1, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	int8_t status = 1;
	status = i2c_mwrite((uint8_t*)(&data), 2, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	int8_t status = 1;
	status = i2c_mwrite((uint8_t*)(&data), 4, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	int8_t status = 1;
	status = i2c_mread_single(data, index, dev);
	return status;
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
	int8_t status = 1;

	status = i2c_mread_multi((uint8_t*)data, 2, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	int8_t status = 1;
	status = i2c_mread_multi((uint8_t*)data, 4, index, dev);
	return status; // to be implemented
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
	LL_mDelay(wait_ms);
	return 0; // to be implemented
}
