///*
// * sensor.c
// *
// *  Created on: Dec 7, 2024
// *      Author: samog
// */
//
//
///**
//  *
//  * Copyright (c) 2023 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
//
//
///* Includes ------------------------------------------------------------------*/
//#include "stm32f3xx_it.h"
//
///* USER CODE BEGIN Includes */
//#include "main.h"
//#include "VL53L1X_API.h"
//#include "VL53l1X_calibration.h"
//#include "X-NUCLEO-53L1A1.h"
///* USER CODE END Includes */
///* Private variables ---------------------------------------------------------*/
///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//UART_HandleTypeDef huart2;
//uint16_t	dev=0x52;
//int status=0;
//volatile int IntCount;
//#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
///* USER CODE BEGIN PFP */
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void Error_Handler(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_I2C1_Init(void);
///* USER CODE END PFP */
//
///* USER CODE BEGIN 0 */
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//
//PUTCHAR_PROTOTYPE
//{
//	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
//  return ch;
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//		if (GPIO_Pin==VL53L1X_INT_Pin)
//		{
//			IntCount++;
//		}
//}
//
///* USER CODE END 0 */
//
//int main(void)
//{
//
//  uint8_t byteData, sensorState=0;
//  uint16_t wordData;
//  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
//  uint16_t Distance;
//  uint16_t SignalRate;
//  uint16_t AmbientRate;
//  uint16_t SpadNum;
//  uint8_t RangeStatus;
//  uint8_t dataReady;
////	int16_t offset;
////	uint16_t xtalk;
//  /* MCU Configuration----------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
//  MX_I2C1_Init();
//  XNUCLEO53L1A1_Init();
//
//  ToFSensor = 1; // Select ToFSensor: 0=Left, 1=Center, 2=Right
//  status = XNUCLEO53L1A1_ResetId(ToFSensor, 0); // Reset ToF sensor
//  HAL_Delay(2);
//  status = XNUCLEO53L1A1_ResetId(ToFSensor, 1); // Reset ToF sensor
//  HAL_Delay(2);
//
///* Those basic I2C read functions can be used to check your own I2C functions */
//  status = VL53L1_RdByte(dev, 0x010F, &byteData);
//  printf("VL53L1X Model_ID: %X\n", byteData);
//  status = VL53L1_RdByte(dev, 0x0110, &byteData);
//  printf("VL53L1X Module_Type: %X\n", byteData);
//  status = VL53L1_RdWord(dev, 0x010F, &wordData);
//  printf("VL53L1X: %X\n", wordData);
//  while(sensorState==0){
//		status = VL53L1X_BootState(dev, &sensorState);
//	HAL_Delay(2);
//  }
//  printf("Chip booted\n");
//
//  /* This function must to be called to initialize the sensor with the default setting  */
//  status = VL53L1X_SensorInit(dev);
//  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 100); /* in ms, IM must be > = TB */
////  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
////  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
////	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
////	status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
//  printf("VL53L1X Ultra Lite Driver Example running ...\n");
//  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
//  while(1){ /* read and display data */
//	  while (dataReady == 0){
//		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
//		  HAL_Delay(2);
//	  }
//	  dataReady = 0;
//	  status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
//	  status = VL53L1X_GetDistance(dev, &Distance);
//	  status = VL53L1X_GetSignalRate(dev, &SignalRate);
//	  status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
//	  status = VL53L1X_GetSpadNb(dev, &SpadNum);
//	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
//	  printf("%u, %u, %u, %u, %u\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
//  }
//}
///** System Clock Configuration
//*/
//
///* USART2 init function */
//
///** Configure pins as
//        * Analog
//        * Input
//        * Output
//        * EVENT_OUT
//        * EXTI
//*/
//
///* USER CODE BEGIN 4 */
//int XNUCLEO53L1A1_Init(void) {
//    int status;
//    uint8_t ExpanderData[2];
//    XNUCLEO53L1A1_USART2_UART_Init();
//    XNUCLEO53L1A1_I2C1Configure();
//
//    status = _ExpanderRd( I2cExpAddr0, 0, ExpanderData, 2);
//    if (status != 0 || ExpanderData[0] != 0x00 || ExpanderData[1] != 0x16) {
//        XNUCLEO53L1A1_ErrLog("I2C Expander @0x%02X not detected",(int)I2cExpAddr0 );
//        goto done_err;
//
//    }
//    status = _ExpanderRd( I2cExpAddr1, 0, ExpanderData, 2);
//    if (status != 0 || ExpanderData[0] != 0x00 || ExpanderData[1] != 0x16) {
//        XNUCLEO53L1A1_ErrLog("I2C Expander @0x%02X not detected",(int)I2cExpAddr1);
//        goto done_err;
//    }
//
//    CurIOVal.u32=0x0;
//    /* setup expender   i/o direction  all output but exp1 bit 14*/
//    ExpanderData[0] = 0xFF;
//    ExpanderData[1] = 0xFF;
//    status = _ExpanderWR(I2cExpAddr0, GPDR, ExpanderData, 2);
//    if (status) {
//        XNUCLEO53L1A1_ErrLog("Set Expander @0x%02X DR", I2cExpAddr0);
//        goto done_err;
//    }
//    ExpanderData[0] = 0xFF;
//    ExpanderData[1] = 0xBF; // all but bit 14-15 that is pb1 and xhurt
//    status = _ExpanderWR(I2cExpAddr1, GPDR, ExpanderData, 2);
//    if (status) {
//        XNUCLEO53L1A1_ErrLog("Set Expander @0x%02X DR", I2cExpAddr1);
//        goto done_err;
//    }
//    /* shut down all segment and all device */
//    CurIOVal.u32=0x7F + (0x7F<<7) + (0x7F<<16)+(0x7F<<(16+7));
//    status= _ExpandersSetAllIO();
//    if( status ){
//        XNUCLEO53L1A1_ErrLog("Set initial i/o ");
//    }
//
//done_err:
//    return status;
//}
//
///* Autonomous ranging loop*/
//void AutonomousLowPowerRangingTest(void)
//{
//}
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @param  None
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler */
//  /* User can add his own implementation to report the HAL error return state */
//  while(1)
//  {
//  }
//  /* USER CODE END Error_Handler */
//}
//
//#ifdef USE_FULL_ASSERT
//
///**
//   * @brief Reports the name of the source file and the source line number
//   * where the assert_param error has occurred.
//   * @param file: pointer to the source file name
//   * @param line: assert_param error line source number
//   * @retval None
//   */
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//
//}
//
//#endif
//
///**
//  * @}
//  */
//
///**
//  * @}
//*/
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
