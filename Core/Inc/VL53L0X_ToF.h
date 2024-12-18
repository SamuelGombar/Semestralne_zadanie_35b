/*
 * VL53L0X_ToF.h
 *
 *  Created on: Dec 18, 2024
 *      Author: Mikasnik
 */

#ifndef INC_VL53L0X_TOF_H_
#define INC_VL53L0X_TOF_H_

#define SENSOR_ADDR_DEFAULT  	0x29





#define SOFT_RESET											0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS					0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					0x001E
#define MM_CONFIG__INNER_OFFSET_MM							0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							0x0022
#define GPIO_HV_MUX__CTRL									0x0030
#define GPIO__TIO_HV_STATUS       							0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD				0x006C
#define SYSTEM__THRESH_HIGH 								0x0072
#define SYSTEM__THRESH_LOW 									0x0074
#define SD_CONFIG__WOI_SD0                  				0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		0x0080
#define SYSTEM__SEQUENCE_CONFIG								0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 				0x0082
#define SYSTEM__INTERRUPT_CLEAR       						0x0086
#define SYSTEM__MODE_START                 					0x0087
#define VL53L1_RESULT__RANGE_STATUS							0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL					0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD				0x013E


static int8_t (* VL53L0X_i2c_mread_single)(uint8_t *pdata, uint16_t register_addr, uint8_t slave_addr) = 0;
static int8_t (* VL53L0X_i2c_mread_multi)(uint8_t *pdata, uint8_t len, uint16_t register_addr, uint8_t slave_addr) = 0;
static int8_t (* VL53L0X_i2c_mwrite)(uint8_t *pdata, uint8_t data_len, uint16_t register_addr, uint8_t slave_addr) = 0;

void VL53L0X_RegisterCallback_i2c_mread_single(void *callback);
void VL53L0X_RegisterCallback_i2c_mread_multi(void *callback);
void VL53L0X_pRegisterCallback_i2c_mwrite(void *callback);

// VL53L1X_api.h
int8_t VL53L0X_CalibrateOffset(uint8_t dev, uint16_t TargetDistInMm, int16_t *offset);
int8_t VL53L0X_CalibrateXtalk(uint8_t dev, uint16_t TargetDistInMm, uint16_t *xtalk);
int8_t VL53L0X_SetI2CAddress(uint8_t, uint8_t new_address);
int8_t VL53L0X_SensorInit(uint8_t dev);
int8_t VL53L0X_ClearInterrupt(uint8_t dev);
int8_t VL53L0X_SetInterruptPolarity(uint8_t dev, uint8_t IntPol);
int8_t VL53L0X_GetInterruptPolarity(uint8_t dev, uint8_t *pInterruptPolarity);
int8_t VL53L0X_StartRanging(uint8_t dev);
int8_t VL53L0X_StopRanging(uint8_t dev);
int8_t VL53L0X_CheckForDataReady(uint8_t dev, uint8_t *isDataReady);
int8_t VL53L0X_SetTimingBudgetInMs(uint8_t dev, uint16_t TimingBudgetInMs);
int8_t VL53L0X_SetDistanceMode(uint8_t dev, uint8_t DistanceMode);
int8_t VL53L0X_SetInterMeasurementInMs(uint8_t dev, uint32_t InterMeasurementInMs);
int8_t VL53L0X_BootState(uint8_t dev, uint8_t *state);
int8_t VL53L0X_GetSensorId(uint8_t dev, uint16_t *id);
int8_t VL53L0X_GetDistance(uint8_t dev, uint16_t *distance);
int8_t VL53L0X_GetSignalPerSpad(uint8_t dev, uint16_t *signalPerSp);
int8_t VL53L0X_GetAmbientPerSpad(uint8_t dev, uint16_t *amb);
int8_t VL53L0X_GetSignalRate(uint8_t dev, uint16_t *signalRate);
int8_t VL53L0X_GetSpadNb(uint8_t dev, uint16_t *spNb);
int8_t VL53L0X_GetAmbientRate(uint8_t dev, uint16_t *ambRate);
int8_t VL53L0X_GetRangeStatus(uint8_t dev, uint8_t *rangeStatus);
int8_t VL53L0X_SetOffset(uint8_t dev, int16_t OffsetValue);
int8_t VL53L0X_SetXtalk(uint8_t dev, uint16_t XtalkValue);
int8_t VL53L0X_SetROICenter(uint8_t dev, uint8_t ROICenter);
int8_t VL53L0X_GetROICenter(uint8_t dev, uint8_t *ROICenter);



int8_t VL53L0X_WriteMulti(
		uint8_t 	  dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint8_t      count);
/** @brief DISP_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_ReadMulti(
		uint8_t 	  dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint8_t      count);
/** @brief DISP_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_WrByte(
		uint8_t 	  dev,
		uint16_t      index,
		uint8_t       data);
/** @brief DISP_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_WrWord(
		uint8_t 	  dev,
		uint16_t      index,
		uint16_t      data);
/** @brief DISP_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_WrDWord(
		uint8_t 	  dev,
		uint16_t      index,
		uint32_t      data);
/** @brief DISP_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_RdByte(
		uint8_t 	  dev,
		uint16_t      index,
		uint8_t      *pdata);
/** @brief DISP_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_RdWord(
		uint8_t 	  dev,
		uint16_t      index,
		uint16_t     *pdata);
/** @brief DISP_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_RdDWord(
		uint8_t 	  dev,
		uint16_t      index,
		uint32_t     *pdata);
/** @brief DISP_WaitMs() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L0X_WaitMs(
		uint8_t      dev,
		uint32_t     wait_ms);



#endif /* INC_VL53L0X_TOF_H_ */

