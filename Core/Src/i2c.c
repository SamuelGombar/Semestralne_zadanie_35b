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
uint8_t data_recive=0;

uint8_t data_recive_8=0;
uint16_t data_recive_16=0;
uint32_t data_recive_32=0;

uint8_t data_recive_multy[3];

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
//ă?­ă?­ă?­ Notes for Read and Write  ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­
//Read Note - if reading multiple registers you set number_of_registers how much, but if reading only one you can put 0 or 1
//Read Important note - outputs only uint32_t
//Write Note - cant write multiple things
//Write Important note - need to check and make sure to not write to reserved registers positions
//ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­ă?­

//Read function
uint32_t i2c_read(uint8_t slave_address, uint8_t register_address, uint8_t number_of_registers){
	//Note
	//Read Note - if reading multiple registers you set number_of_registers how much, but if reading only one you can put 0 or 1
	//Read Important note - outputs only uint32_t

	data_recive=0;

	if(number_of_registers == 2){
		data_recive_16=0;
		register_address |= 0x80;
	}
	if(number_of_registers == 3){
		data_recive_32=0;
		register_address |= 0x80;
	}


	LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
	while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}


	LL_I2C_TransmitData8(I2C1, register_address);
	while (!LL_I2C_IsActiveFlag_TC(I2C1)) {}


	if(number_of_registers <= 1){
		LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	}
	if(number_of_registers > 1){
		LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, number_of_registers, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_READ);
	}
	while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {}


	if(number_of_registers > 1){
	for(int i=0; i<number_of_registers;i++){
			while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
				if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
					LL_I2C_ClearFlag_STOP(I2C1);
				}
			}
			data_recive_multy[i] = LL_I2C_ReceiveData8(I2C1);
			//while (!LL_I2C_IsActiveFlag_(I2C1)) {}
		}
	}else{
		data_recive = LL_I2C_ReceiveData8(I2C1);
		while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {}
	}
	LL_I2C_ClearFlag_STOP(I2C1);


	if(number_of_registers == 1){
		data_recive_8 = data_recive;
		return data_recive_8;
	}
	if(number_of_registers == 2){
		data_recive_16 = (data_recive_multy[1] << 8) | data_recive_multy[0];
		return data_recive_16;
	}
	if(number_of_registers == 3){
		data_recive_32 = (data_recive_multy[2] << 16) | (data_recive_multy[1] << 8) | data_recive_multy[0];
		return data_recive_32;
	}
	if(number_of_registers == 0){
		return data_recive;
	}

	/* Maybe more safe option of returning the value
	if(number_of_registers == 1){
		data_recive_8 = data_recive;
		return (uint32_t)data_recive_8;
	}
	if(number_of_registers == 2){
		data_recive_16 = (data_recive_multy[1] << 8) | data_recive_multy[0];
		return (uint32_t)data_recive_16;
	}
	if(number_of_registers == 3){
		data_recive_32 = (data_recive_multy[2] << 16) | (data_recive_multy[1] << 8) | data_recive_multy[0];
		return data_recive_32;
	}
	if(number_of_registers == 0){
		return (uint32_t)data_recive;
	}
 	*/

}



//Write function
uint32_t i2c_write(uint8_t slave_address, uint8_t register_address, uint8_t data, uint8_t number_of_registers){
	//Note
	//Write Note - cant write multiple things
	//Write Important note - need to check and make sure to not write to reserved registers positions

	if(number_of_registers == 2){
		register_address |= 0x80;
	}
	if(number_of_registers == 3){
		register_address |= 0x80;
	}

																			//zmenene z 2
	//LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, number_of_registers + 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
	//while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}

	LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
	while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}

	LL_I2C_TransmitData8(I2C1, register_address);
	while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}

	//pridane viacbytove zapisovanie (moze to byt takto len v cykle?) nope, I dont realy know
	/*
	for (int i = 0; i < number_of_registers; i++) {
		LL_I2C_TransmitData8(I2C1, data[i]); //treba implementovat multiwrite (toto je pravdepodobne zle)
		while (!LL_I2C_IsActiveFlag_TC(I2C1)) {}
	}
 	*/
	LL_I2C_TransmitData8(I2C1, data);
	while (!LL_I2C_IsActiveFlag_TC(I2C1)) {}

	LL_I2C_GenerateStopCondition(I2C1);
	while (LL_I2C_IsActiveFlag_STOP(I2C1) == 0) {}

	LL_I2C_ClearFlag_STOP(I2C1);

	return 0;
}


// Write function that can do multy write
void i2c_master_write(uint8_t slave_addr, uint8_t register_addr, uint8_t *data, uint8_t len, uint8_t read_flag){
    //slave_addr - addres of the sensor
    //register_addr - into what register to write data
    //*data - array of data to be writen to the specified resiger addres 
    //len - number of bytes that you want to write
    //read_flag - *note*
    //Note: dont know what tha flag douing but ok - best guess activates 8. bit to enable auto indexing - how tha huk the sensor know how many to expect is to be discoverede
    if (read_flag) register_addr |= (1 << 7);        // activate PD : hts221 pg. 22

    LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1 + len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, register_addr);

    uint8_t index = 0;
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {    // a Stop condition, which is the end of a communication sequence
        if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {    // Transmit Interrupt Status (TXIS) flag indicates that the data register is empty and ready for the next byte of data to be transmitted
            if (index < len) {
                LL_I2C_TransmitData8(I2C1, data[index++]);
            }
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);    // the stop condition was processed and now it is time to liberate the flag
    return;
}
/* USER CODE END 1 */
