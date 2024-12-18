/*
 * VL53L0X_ToF.c
 *
 *  Created on: Dec 18, 2024
 *      Author: Mikasnik
 */

#include <stdint.h>
#include "VL53L0X_ToF.h"
#include "stm32f3xx_ll_utils.h"
const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xcc, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : Intermeasurement period */
0x0f, /* 0x6e : Intermeasurement period */
0x89, /* 0x6f : Intermeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

//static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,	255, 255, 11, 12};


int8_t VL53L0X_GetSingleMeasurement(uint8_t dev, uint16_t* distance){
	int8_t status = 0;
	uint8_t tmp;;
	while(tmp==0){
		status |= VL53L0X_CheckForDataReady(dev, &tmp);
	}
	status |= VL53L0X_GetDistance(dev, distance);
	status |= VL53L0X_ClearInterrupt(dev);

	return status;
}


int8_t VL53L0X_SetI2CAddress(uint8_t dev, uint8_t new_address){
	int8_t status = VL53L0X_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);  // New address to check
	return status;
}

int8_t VL53L0X_BootState(uint8_t dev, uint8_t *state){
	int8_t status = 0;
	uint8_t tmp = 0;

	status |= VL53L0X_RdByte(dev,VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
	*state = tmp;
	return status;
}

int8_t VL53L0X_GetSensorId(uint8_t dev, uint16_t *sensorId){
	int8_t status = 0;
	uint16_t tmp = 0;
	status |= VL53L0X_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
	*sensorId = tmp;
	return status;
}

int8_t VL53L0X_SensorInit(uint8_t dev){
	int8_t status = 0;
	uint8_t Addr = 0x00, tmp;

	for (Addr = 0x2D; Addr <= 0x87; Addr++){
		status |= VL53L0X_WrByte(dev, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
	}

	status |= VL53L0X_SetInterruptPolarity(dev, 0);
	status |= VL53L0X_StartRanging(dev);
	tmp  = 0;
	while(tmp==0){
			status |= VL53L0X_CheckForDataReady(dev, &tmp);
	}
	status |= VL53L0X_ClearInterrupt(dev);
	status |= VL53L0X_StopRanging(dev);
	// This may need check up
	status |= VL53L0X_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	status |= VL53L0X_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
	return status;
}

int8_t VL53L0X_SetInterruptPolarity(uint8_t dev, uint8_t NewPolarity){
	uint8_t Temp;
	int8_t status = 0;

	status |= VL53L0X_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0xEF; // to make definitely bit 5 -> 0
	status |= VL53L0X_WrByte(dev, GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
	return status;
}

int8_t VL53L0X_StartRanging(uint8_t dev){
	int8_t status = 0;
	status |= VL53L0X_WrByte(dev, SYSTEM__MODE_START, 0x40);	/* Enable VL53L1X */
	return status;
}

int8_t VL53L1X_StopRanging(uint16_t dev){
	int8_t status = 0;
	status |= VL53L0X_WrByte(dev, SYSTEM__MODE_START, 0x00);	/* Disable VL53L1X */
	return status;
}

int8_t VL53L0X_CheckForDataReady(uint8_t dev, uint8_t *isDataReady){
	uint8_t Temp;
	uint8_t IntPol;
	int8_t status = 0;

	status |= VL53L0X_GetInterruptPolarity(dev, &IntPol);
	status |= VL53L0X_RdByte(dev, GPIO__TIO_HV_STATUS, &Temp);
	/* Read in the register to check if a new value is available */
	if (status == 0){
		if ((Temp & 1) == IntPol)
			*isDataReady = 1;
		else
			*isDataReady = 0;
	}
	return status;
}

int8_t VL53L0X_GetInterruptPolarity(uint8_t dev, uint8_t *pInterruptPolarity){
	uint8_t Temp;
	int8_t status = 0;

	status |= VL53L0X_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0x10;
	*pInterruptPolarity = !(Temp>>4);
	return status;
}

int8_t VL53L0X_ClearInterrupt(uint8_t dev){
	int8_t status = 0;
	status |= VL53L0X_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
	return status;
}

int8_t VL53L0X_GetDistance(uint8_t dev, uint16_t *distance){
	int8_t status = 0;
	uint16_t tmp;
	status |= (VL53L0X_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
	*distance = tmp;
	return status;
}







void VL53L0X_RegisterCallback_i2c_mread_single(void *callback) {
        if(callback != 0) VL53L0X_i2c_mread_single = callback;
}
void VL53L0X_RegisterCallback_i2c_mread_multi(void *callback) {
        if(callback != 0) VL53L0X_i2c_mread_multi = callback;
}
void VL53L0X_RegisterCallback_i2c_mwrite(void *callback) {
        if(callback != 0) VL53L0X_i2c_mwrite = callback;
}

int8_t VL53L0X_WriteMulti(uint8_t dev, uint16_t index, uint8_t *pdata, uint8_t count) {
	int8_t status = 1;
	status = VL53L0X_i2c_mwrite(pdata, count, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_ReadMulti(uint8_t dev, uint16_t index, uint8_t *pdata, uint8_t count) {
	int8_t status = 1;
	status = VL53L0X_i2c_mread_multi(pdata, count, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_WrByte(uint8_t dev, uint16_t index, uint8_t data) {
	int8_t status = 1;
	status = VL53L0X_i2c_mwrite((uint8_t*)(&data), 1, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_WrWord(uint8_t dev, uint16_t index, uint16_t data) {
	int8_t status = 1;
	status = VL53L0X_i2c_mwrite((uint8_t*)(&data), 2, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_WrDWord(uint8_t dev, uint16_t index, uint32_t data) {
	int8_t status = 1;
	status = VL53L0X_i2c_mwrite((uint8_t*)(&data), 4, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_RdByte(uint8_t dev, uint16_t index, uint8_t *data) {
	int8_t status = 1;
	status = VL53L0X_i2c_mread_single(data, index, dev);
	return status;
}

int8_t VL53L0X_RdWord(uint8_t dev, uint16_t index, uint16_t *data) {
	int8_t status = 1;

	status = VL53L0X_i2c_mread_multi((uint8_t*)data, 2, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_RdDWord(uint8_t dev, uint16_t index, uint32_t *data) {
	int8_t status = 1;
	status = VL53L0X_i2c_mread_multi((uint8_t*)data, 4, index, dev);
	return status; // to be implemented
}

int8_t VL53L0X_WaitMs(uint8_t dev, uint32_t wait_ms) {
	LL_mDelay(wait_ms);
	return 0; // to be implemented
}

