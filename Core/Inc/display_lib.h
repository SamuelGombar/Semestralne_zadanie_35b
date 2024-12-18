/*
 * display_lib.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Mikasnik
 */

#ifndef DISPLAY_LIB_H_
	#define DISPLAY_LIB_H_

	#ifdef __cplusplus
		extern "C"
		{
	#endif

	#define EXPANDER1_ADD				0x42
	#define EXPANDER2_ADD				0x43

	#define GPDR_REG 					0x14
	#define GPSR_REG					0x12

	#define PIN0	(1<<0)
	#define PIN1	(1<<1)
	#define PIN2	(1<<2)
	#define PIN3	(1<<3)
	#define PIN4	(1<<4)
	#define PIN5	(1<<5)
	#define PIN6	(1<<6)
	#define PIN7	(1<<7)
	#define PIN8	(1<<8)
	#define PIN9	(1<<9)
	#define PIN10	(1<<10)
	#define PIN11	(1<<11)
	#define PIN12	(1<<12)
	#define PIN13	(1<<13)
	#define PIN14	(1<<14)
	#define PIN15	(1<<15)


	#include <stdint.h>

	//typedef DISP_Dev_t *DISP_DEV;

	static int8_t (* disp_i2c_mread_single)(uint8_t *pdata, uint8_t register_addr, uint8_t slave_addr) = 0;
	static int8_t (* disp_i2c_mread_multi)(uint8_t *pdata, uint8_t len, uint8_t register_addr, uint8_t slave_addr) = 0;
	static int8_t (* disp_i2c_mwrite)(uint8_t *pdata, uint8_t data_len, uint8_t register_addr, uint8_t slave_addr) = 0;

	/** @brief DISP_WriteMulti() definition.\n
	 * To be implemented by the developer
	 */

	void DispRegisterCallback_i2c_mread_single(void *callback);
	void DispRegisterCallback_i2c_mread_multi(void *callback);
	void DispRegisterCallback_i2c_mwrite(void *callback);

	int8_t display_strongLeft();

	int8_t DISP_WriteMulti(
			uint8_t 	  dev,
			uint8_t      index,
			uint8_t      *pdata,
			uint8_t      count);
	/** @brief DISP_ReadMulti() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_ReadMulti(
			uint8_t 	  dev,
			uint8_t      index,
			uint8_t      *pdata,
			uint8_t      count);
	/** @brief DISP_WrByte() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_WrByte(
			uint8_t 	  dev,
			uint8_t      index,
			uint8_t       data);
	/** @brief DISP_WrWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_WrWord(
			uint8_t 	  dev,
			uint8_t      index,
			uint16_t      data);
	/** @brief DISP_WrDWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_WrDWord(
			uint8_t 	  dev,
			uint8_t      index,
			uint32_t      data);
	/** @brief DISP_RdByte() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_RdByte(
			uint8_t 	  dev,
			uint8_t      index,
			uint8_t      *pdata);
	/** @brief DISP_RdWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_RdWord(
			uint8_t 	  dev,
			uint8_t      index,
			uint16_t     *pdata);
	/** @brief DISP_RdDWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_RdDWord(
			uint8_t 	  dev,
			uint8_t      index,
			uint32_t     *pdata);
	/** @brief DISP_WaitMs() definition.\n
	 * To be implemented by the developer
	 */
	int8_t DISP_WaitMs(
			uint8_t      dev,
			uint32_t       wait_ms);

	#ifdef __cplusplus
	}
	#endif



#endif /* INC_SENSOR_H_ */
