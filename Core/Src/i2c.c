/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
uint8_t *i2c_rx_data; //buffer dat
uint8_t r_index; //incrementovanie pri citani
uint8_t line_ready = 1; //zatial nie je implementovany
uint8_t multi_read;   //switch, ci jeden alebo viac bytov
uint8_t i2c_rx_byte; //len jeden vycitany byte
/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x2000090E;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 2;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */


/*int8_t i2c_master_write(uint8_t *buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr) {
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1 + len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_mDelay(10);
	if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
		LL_I2C_ClearFlag_NACK(I2C1);
		LL_I2C_ClearFlag_STOP(I2C1);
		return 1;
	}
	LL_I2C_TransmitData8(I2C1, register_addr);

	uint8_t index = 0;
	while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {	// a Stop condition, which is the end of a communication sequence
	    if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {	// Transmit Interrupt Status (TXIS) flag indicates that the data register is empty and ready for the next byte of data to be transmitted
	        if (index < len) {
	            LL_I2C_TransmitData8(I2C1, buff[index++]);
	        }
	    }
	}

	LL_I2C_ClearFlag_STOP(I2C1);	// the stop condition was processed and now it is time to liberate the flag
	return 0;
}*/

int8_t i2c_master_write(uint8_t *pdata, uint8_t data_len, uint16_t register_addr, uint8_t slave_addr){
	//if (read_flag) register_addr |= (1 << 7);		// activate PD : hts221 pg. 22

	LL_I2C_HandleTransfer(I2C1, slave_addr<<1, LL_I2C_ADDRSLAVE_7BIT, 1 + data_len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(I2C1, register_addr);

	uint8_t index = 0;
	while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {	// a Stop condition, which is the end of a communication sequence
	    if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {	// Transmit Interrupt Status (TXIS) flag indicates that the data register is empty and ready for the next byte of data to be transmitted
	        if (index < data_len) {
	            LL_I2C_TransmitData8(I2C1, pdata[index++]);
	        }
	    }
	}
	if (LL_I2C_IsActiveFlag_NACK(I2C1)){
			LL_I2C_ClearFlag_STOP(I2C1);
			LL_I2C_ClearFlag_NACK(I2C1);
			return 1;
		}
	LL_I2C_ClearFlag_STOP(I2C1);	// the stop condition was processed and now it is time to liberate the flag
	return 0;
}

int8_t i2c_master_read_single(uint8_t* pdata, uint16_t register_addr, uint8_t slave_addr) {
	register_addr |= (1 << 7); 	// activate PD : hts221 pg. 22
	line_ready = 0;
	multi_read = 0;

	// Enable It from I2C
	LL_I2C_EnableIT_RX(I2C1); // enable the RXNE (Receive Data Register Not Empty) interrupt (triggered when the data register contains new data that has been received and is ready to be read)

	// Initialise communication
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_mDelay(10);
	if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
		LL_I2C_ClearFlag_NACK(I2C1);
		LL_I2C_ClearFlag_STOP(I2C1);
		return 1;
	}
	while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {    // a Stop condition, which is the end of a communication sequence
		if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {    // Transmit Interrupt Status (TXIS) flag indicates that the data register is empty and ready for the next byte of data to be transmitted
			LL_I2C_TransmitData8(I2C1, register_addr);
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);    // the stop condition was processed and now it is time to liberate the flag

	while (LL_I2C_IsActiveFlag_STOP(I2C1)) ;    // to ensure liberated stop flag

	// Receive data from slave device and read them per interrupt handler
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	while (!LL_I2C_IsActiveFlag_STOP(I2C1)) ;

	//End of transfer
	LL_I2C_DisableIT_RX(I2C1);
	LL_I2C_ClearFlag_STOP(I2C1);
	LL_I2C_ClearFlag_NACK(I2C1);
	line_ready = 1;

	*pdata = i2c_rx_byte;
	return 0;
}


int8_t i2c_master_read_multi(uint8_t* buff, uint8_t len, uint16_t register_addr, uint8_t slave_addr) {
	register_addr |= (1 << 7); 	// activate PD : hts221 pg. 22
	r_index = 0;
	i2c_rx_data = buff;
	line_ready = 0;
	multi_read = 1;

	// Enable It from I2C
	LL_I2C_EnableIT_RX(I2C1); // enable the RXNE (Receive Data Register Not Empty) interrupt (triggered when the data register contains new data that has been received and is ready to be read)

	// Initialise communication
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_mDelay(10);
	if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
		LL_I2C_ClearFlag_NACK(I2C1);
		LL_I2C_ClearFlag_STOP(I2C1);
		return 1;
	}
	while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {	// a Stop condition, which is the end of a communication sequence
		if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {	// Transmit Interrupt Status (TXIS) flag indicates that the data register is empty and ready for the next byte of data to be transmitted
			LL_I2C_TransmitData8(I2C1, register_addr);
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);	// the stop condition was processed and now it is time to liberate the flag

	while (LL_I2C_IsActiveFlag_STOP(I2C1)) ;	// to ensure liberated stop flag

	// Receive data from slave device and read them per interrupt handler
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	while (!LL_I2C_IsActiveFlag_STOP(I2C1)) ;

	//End of transfer
	LL_I2C_DisableIT_RX(I2C1);
	LL_I2C_ClearFlag_STOP(I2C1);
	LL_I2C_ClearFlag_NACK(I2C1);
	line_ready = 1;

	buff = i2c_rx_data;
	return 0;
}


void I2C1_EV_IRQHandler(void){
    // Check RXNE flag value in ISR register
    if(LL_I2C_IsActiveFlag_RXNE(I2C1)) {
    	if (multi_read){
    		i2c_rx_data[r_index++] = LL_I2C_ReceiveData8(I2C1); // Call function Master Reception Callback
    		//if (r_index >= I2C_MAX_BYTES_TO_READ){
    		r_index %= I2C_MAX_BYTES_TO_READ; // to prevent overflow of data
        	//line_ready = 1;
    		//}
    	} else {
    		i2c_rx_byte = LL_I2C_ReceiveData8(I2C1);
    	}

    }
}
/* USER CODE END 1 */
