/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define I2C_MAX_BYTES_TO_READ 4
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
// x: uint8_t i2c_master_read_byte(uint8_t slave_address, uint8_t register_address);
int8_t i2c_master_read_multi(uint8_t *buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr);
int8_t i2c_master_read_single(uint8_t *pdata, uint8_t register_addr, uint8_t slave_addr);

int8_t i2c_master_write(uint8_t *buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

