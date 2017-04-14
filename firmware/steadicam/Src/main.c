/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "mpu9255.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	MPU9255_t MPU9255;
	uint8_t Address, i, SPI_Data=0x00;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
	MPU9255.SPI = &hspi3; 							// set spi connected to mpu9255
	MPU9255.SPI_CS_PORT = GPIOA;				// set CS port
	MPU9255.SPI_CS_PIN = GPIO_PIN_15;		// set CS pin number
	
	MPU9255.Init.ASense = MPU9255_AccelSens_2G; 		// set accel sense
	MPU9255.Init.GSense = MPU9255_GyroSens_250DPS;	// set gyro sense
	MPU9255.Init.MSense = MPU9255_MagSens_16Bit;		// set mag sense
	
	/* Calculate multiplicators */
  MPU9255.MMult = 10.0f * 4912.0f / 32768.0f;
	
	HAL_Delay(100);
	Address = MPU9255_WHO_AM_I;
	MPU9255_GetReg(&MPU9255, &Address, &SPI_Data);
	
	// Accelerometer check
	if (SPI_Data == 0x73) { // if accel is mpu9255
			RED_LED_ON(); BLUE_LED_ON(); GREEN_LED_ON(); YELLOW_LED_ON();				HAL_Delay(1000);
			RED_LED_OFF(); BLUE_LED_OFF(); GREEN_LED_OFF(); YELLOW_LED_OFF();		HAL_Delay(1000);
		} else { 
			i=0;
			while(i<3) {
				RED_LED_ON();BLUE_LED_OFF();GREEN_LED_OFF();YELLOW_LED_OFF();			HAL_Delay(50);
				RED_LED_OFF();BLUE_LED_ON();GREEN_LED_OFF();YELLOW_LED_OFF();			HAL_Delay(50);
				RED_LED_OFF();BLUE_LED_OFF();GREEN_LED_ON();YELLOW_LED_OFF();			HAL_Delay(50);
				RED_LED_OFF();BLUE_LED_OFF();GREEN_LED_OFF();YELLOW_LED_ON();			HAL_Delay(50);
				i++;
				RED_LED_OFF();BLUE_LED_OFF();GREEN_LED_OFF();YELLOW_LED_OFF();
			}
	}
		
	MPU9255_Init(&MPU9255);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		MPU9255_ReadAccel(&MPU9255);
		if (MPU9255.Ax >= 0.5f) {
			RED_LED_ON(); GREEN_LED_OFF();
		} else if (MPU9255.Ax <= -0.5f)   {
			RED_LED_OFF(); GREEN_LED_ON(); 
		} else {
			GREEN_LED_OFF(); RED_LED_OFF();
		}
		
		if (MPU9255.Ay >= 0.5f) {
			YELLOW_LED_ON(); BLUE_LED_OFF();
		} else if (MPU9255.Ay <= -0.5f)   {
			YELLOW_LED_OFF(); BLUE_LED_ON(); 
		} else {
			YELLOW_LED_OFF(); BLUE_LED_OFF();
		}
		HAL_Delay(1);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
