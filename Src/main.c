/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f2xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_custom_hid_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern	USBD_HandleTypeDef  *hUsbDevice_0;

uint8_t USB_RX_Buffer[64];
uint8_t USB_TX_Buffer[64];	//To send usb data to PC

uint8_t STATUSM = 0;

uint32_t KEY_REG;
uint32_t CRC_Rezult;
volatile uint32_t trng;

#define ID_REQUEST_BUFFER									USB_RX_Buffer[1]
#define PASSWORD_LENGTH										USB_RX_Buffer[2]
#define ID_ANSWER_BUFFER									USB_TX_Buffer[1]

#define	PASSWORD_LENGTH_MIN										8
#define	PASSWORD_LENGTH_MAX										24

#define	INSTALL_BIT												0x80	//10000000b
#define	CASE_OPEN_BIT											0x40	//01000000b
#define	RNG_ERROR_BIT											0x01	//00000001b
#define	KEY_1_ERROR_BIT										0x02	//00000010b
#define	KEY_2_ERROR_BIT										0x04	//00000100b
#define	PASSWORD_ERROR_BIT								0x08	//00001000b
#define	FLASH_ERROR_BIT										0x10	//00010000b

#define	INSTALL_BYTE_ADDRESS							0x08060000
#define	PASSWORD_LENGTH_ADDRESS						0x08060002
#define	PASSWORD_CRC_ADDRESS							0x08060004
#define	KEY_1_CRC_ADDRESS									0x08060008
#define	KEY_2_CRC_ADDRESS									0x0806000C

#define	INSTALL_BYTE											0x55	

//************************************************************
//ƒанные дл€ проверки генератора случайных чисел по FIPS-140-2
//************************************************************
//  оличество 1 в каждой из 16 возможных последовательностей
static char abNumMem[16] = {	0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
	#define	MONOBIT_TEST_ERROR_BIT						0x01	//00000001b
	#define	POKER_TEST_ERROR_BIT							0x02	//00000010b
	#define	RUNS_TEST_ERROR_BIT								0x04	//00000100b
	#define	LONG_RUNS_TEST_ERROR_BIT					0x08	//00001000b
	#define	TEST_BIT_SIZE											20000
	#define	LONG_RUNS_MAX											26	
	#define	Test_Monobit_MIN									9725
	#define	Test_Monobit_MAX									10275
	#define	Poker_Test_MIN										10800
	#define	Poker_Test_MAX										230850	
	#define	Runs_Test_1_MIN										2315
	#define	Runs_Test_1_MAX										2685
	#define	Runs_Test_2_MIN										1114
	#define	Runs_Test_2_MAX										1386
	#define	Runs_Test_3_MIN										527
	#define	Runs_Test_3_MAX										723
	#define	Runs_Test_4_MIN										240
	#define	Runs_Test_4_MAX										384
	#define	Runs_Test_5_MIN										103
	#define	Runs_Test_5_MAX										209
	#define	Runs_Test_6_MIN										103
	#define	Runs_Test_6_MAX										209

	volatile uint8_t TEST_RNG_BYTE = 0;
//************************************************************

/* Backup registers table */
uint32_t KEY_1_DataReg[8] =
{
  RTC_BKP_DR0,  RTC_BKP_DR1,  RTC_BKP_DR2, 
  RTC_BKP_DR3,  RTC_BKP_DR4,  RTC_BKP_DR5,
  RTC_BKP_DR6,  RTC_BKP_DR7
};
uint32_t KEY_2_DataReg[8] =
{
  RTC_BKP_DR8,  RTC_BKP_DR9,  RTC_BKP_DR10, 
  RTC_BKP_DR11,  RTC_BKP_DR12,  RTC_BKP_DR13,
  RTC_BKP_DR14,  RTC_BKP_DR15
};

uint32_t KEY_REG_BUF[8];

uint8_t Read_Flash_Byte(uint32_t Data_adr)
{
   return *(uint8_t*) Data_adr;
}

uint32_t Read_Flash_Word(uint32_t Data_adr)
{
   return *(uint32_t*) Data_adr;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void	CLR_USB_TX_RX_Buffers (void);
static void TEST_RNG_FIPS_140 (void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */

	TEST_RNG_FIPS_140 ();
	if (TEST_RNG_BYTE != 0)
	{
		STATUSM = (STATUSM | RNG_ERROR_BIT);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		USB_TX_Buffer[0] = ID_ANSWER;

			if (ID_REQUEST_BUFFER == ID_REQUEST_STATUS)
			{
				STATUSM = (STATUSM | RNG_ERROR_BIT);
				STATUSM = (STATUSM ^ RNG_ERROR_BIT);
				TEST_RNG_FIPS_140 ();
				if (TEST_RNG_BYTE != 0)
				{
					STATUSM = (STATUSM | RNG_ERROR_BIT);
					USB_TX_Buffer[3] = TEST_RNG_BYTE;
				}
				USB_TX_Buffer[1] = ID_STATUS;
				USB_TX_Buffer[2] = STATUSM;
				USBD_CUSTOM_HID_SendReport(hUsbDevice_0,USB_TX_Buffer,ANSWER_LENGTH+1);
				CLR_USB_TX_RX_Buffers();
			}
			if (ID_REQUEST_BUFFER == ID_REQUEST_RNG)
			{
				trng = HAL_RNG_GetRandomNumber(&hrng);
					for (uint8_t i = 2; i < 34; i=i+4)
					{
						trng = HAL_RNG_GetRandomNumber(&hrng);
						USB_TX_Buffer[i] = (trng & 0x000000FF);	//To read usb data from PC
						USB_TX_Buffer[i+1] = (trng & 0x0000FF00) >> 8;
						USB_TX_Buffer[i+2] = (trng & 0x00FF0000) >> 16;
						USB_TX_Buffer[i+3] = (trng & 0xFF000000) >> 24;
					}
				USB_TX_Buffer[1] = ID_RNG;
				USBD_CUSTOM_HID_SendReport(hUsbDevice_0,USB_TX_Buffer,ANSWER_LENGTH+1);
				CLR_USB_TX_RX_Buffers();
			}
			if (ID_REQUEST_BUFFER == ID_REQUEST_TIME)
			{
				RTC_TimeTypeDef sTime;
				RTC_DateTypeDef sDate;
				
				HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BCD);
				HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BCD);

				USB_TX_Buffer[2] = sTime.Seconds;
				USB_TX_Buffer[3] = sTime.Minutes;
				USB_TX_Buffer[4] = sTime.Hours;
				
				USB_TX_Buffer[5] = sDate.Date;
				USB_TX_Buffer[6] = sDate.WeekDay;
				USB_TX_Buffer[7] = sDate.Month;
				USB_TX_Buffer[8] = sDate.Year;
			
				ID_ANSWER_BUFFER = ID_TIME;
				USBD_CUSTOM_HID_SendReport(hUsbDevice_0,USB_TX_Buffer,ANSWER_LENGTH+1);
				
				CLR_USB_TX_RX_Buffers();
			}	
			if (ID_REQUEST_BUFFER == ID_REQUEST_TIME_INSTALL)
			{
					RTC_TimeTypeDef sTime;
					RTC_DateTypeDef sDate;
				
					sTime.Seconds = USB_RX_Buffer[27];
					sTime.Minutes = USB_RX_Buffer[28];
					sTime.Hours = USB_RX_Buffer[29];
					sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					sTime.StoreOperation = RTC_STOREOPERATION_RESET;
					HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

					sDate.Date = USB_RX_Buffer[30];
					sDate.WeekDay = USB_RX_Buffer[31];
					sDate.Month = USB_RX_Buffer[32];
					sDate.Year = USB_RX_Buffer[33];
					HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BCD);
				}					
				USB_TX_Buffer[1] = ID_STATUS;
				USB_TX_Buffer[2] = STATUSM;
				USBD_CUSTOM_HID_SendReport(hUsbDevice_0,USB_TX_Buffer,ANSWER_LENGTH+1);																//and send STATUS
				STATUSM = (STATUSM | PASSWORD_ERROR_BIT);
				STATUSM = (STATUSM ^ PASSWORD_ERROR_BIT);		
				CLR_USB_TX_RX_Buffers();
			}
			
	
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

//}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  HAL_CRC_Init(&hcrc);

}

/* RNG init function */
void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  HAL_RNG_Init(&hrng);

}

/* RTC init function */
void MX_RTC_Init(void)
{

//  RTC_TimeTypeDef sTime;
//  RTC_DateTypeDef sDate;
//  RTC_TamperTypeDef sTamper;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);
/*
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BCD);
*/
    /**Enable the RTC Tamper 
    */
 /* sTamper.Tamper = RTC_TAMPER_1;
  sTamper.PinSelection = RTC_TAMPERPIN_PC13;
  sTamper.Trigger = RTC_TAMPERTRIGGER_RISINGEDGE;
  HAL_RTCEx_SetTamper_IT(&hrtc, &sTamper);
*/
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void	CLR_USB_TX_RX_Buffers (void)
{
	for (uint8_t i = 1; i < 34; i = i++)
	{
		USB_TX_Buffer[i] = 0;
		USB_RX_Buffer[i] = 0;
	}
}

void TEST_RNG_FIPS_140 (void)
{
//**********************************************************************		
//√енерируем 20000 бит блоками по 32 бита и анализируем эти блоки.
//Ѕуфер со случайным числом не накапливаем.
//’раним только анализируемые 32 бита случайности, разбив их на тетрады.
//**********************************************************************
//
//“ест знаков.
//“ест пройден если число битов последовательности, установленных в 1, 
//находитьс€ в промежутке от 9725 до 10275
//
//ѕокер-тест.
//ѕоследовательность разбиваетс€ на 5000 тетрад. “етрады интерпретируютс€ как числа от 0 до 15.
//ќпредел€етс€ статистика S = 16 *sum _{i=0...15} S(i)^2 - (5000)^2,
//где S(i) -- количество по€влений числа i среди тетрад.
//“ест пройден, если 10800 < S < 230850.	
//		
//“ест серий
//ќпредел€ютс€ серии (максимальные последовательности	повтор€ющихс€ соседних битов) различных длин. 
//“ест пройден, если и дл€ серий из нулей, и дл€ серий из единиц выполн€етс€:
//S_1 \in [2315, 2685],
//S_2 \in [1114, 1386],
//S_3 \in [527, 723],
//S_4 \in [240, 384],
//S_5, S_6+ \in [103, 209].
//«десь S_i -- количество серий длины i = 1, 2, ..., S_6+ = S_6 + S_7 + .... 		
//		
//“ест длинных серий
//“ест пройден, если в последовательности отсутствуют серии длины 26 и больше.
//**********************************************************************		

	volatile uint8_t TEST_RNG_Buffer[8];
	volatile uint16_t Test_Monobit_Rezult = 0;
	volatile uint16_t Poker_Test_Buffer[16] = {	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	volatile uint32_t Poker_Test_Rezult = 0;
	volatile uint16_t Runs_Test_Buffer[2][7] = {{0, 0, 0, 0, 0, 0, 0, },{0, 0, 0, 0, 0, 0, 0, }};	
	volatile uint8_t b;
	volatile uint16_t Length_Run = 1;

	TEST_RNG_BYTE = 0;
		
	for (uint16_t i = 0; i < (TEST_BIT_SIZE/32); i = i++)
	{	
		trng = HAL_RNG_GetRandomNumber(&hrng);
	
		TEST_RNG_Buffer[0] = (trng & 0x0000000F);	
		TEST_RNG_Buffer[1] = (trng & 0x000000F0) >> 4;
		TEST_RNG_Buffer[2] = (trng & 0x00000F00) >> 8;
		TEST_RNG_Buffer[3] = (trng & 0x0000F000) >> 12;
		TEST_RNG_Buffer[4] = (trng & 0x000F0000) >> 16;	
		TEST_RNG_Buffer[5] = (trng & 0x00F00000) >> 20;
		TEST_RNG_Buffer[6] = (trng & 0x0F000000) >> 24;
		TEST_RNG_Buffer[7] = (trng & 0xF0000000) >> 28;
	
		for (uint8_t j = 0;  j < 8;  j = j++)
		{
			Test_Monobit_Rezult = (Test_Monobit_Rezult + abNumMem[TEST_RNG_Buffer[j]]);
			Poker_Test_Buffer[TEST_RNG_Buffer[j]]++;
		}

		if (i == 0)
		{
			b = TEST_RNG_Buffer[0] & 1;
		}
		
		for (uint8_t j = 1; j < 32; j = j++)
		{
			if (((TEST_RNG_Buffer[j/4] >> (j%4)) & 1) == b)
			{
				Length_Run++;
			}
			else
			{
				if (Length_Run >= LONG_RUNS_MAX)
				{
					TEST_RNG_BYTE = (TEST_RNG_BYTE | LONG_RUNS_TEST_ERROR_BIT);
				}
				else
				{
					if(Length_Run>6)
					{
						Length_Run=6;
					}
				(Runs_Test_Buffer[b][Length_Run])++;
				b = (b ^ 1);
				Length_Run = 1;
				}
			}
		}
	}
	for (uint8_t i = 0; i < 16; i = i++)
	{
		Poker_Test_Rezult = (Poker_Test_Rezult + (Poker_Test_Buffer[i] * Poker_Test_Buffer[i]));
	}
	Poker_Test_Rezult = ((16 * Poker_Test_Rezult) - (5000 * 5000));

	if ((Test_Monobit_Rezult < Test_Monobit_MIN) || (Test_Monobit_Rezult > Test_Monobit_MAX))
	{
		TEST_RNG_BYTE = (TEST_RNG_BYTE | MONOBIT_TEST_ERROR_BIT);
	}
	if ((Poker_Test_Rezult < Poker_Test_MIN) || (Poker_Test_Rezult > Poker_Test_MAX))
	{
		TEST_RNG_BYTE = (TEST_RNG_BYTE | POKER_TEST_ERROR_BIT);
	}
	if((Runs_Test_Buffer[0][1] < Runs_Test_1_MIN) || (Runs_Test_Buffer[0][1] > Runs_Test_1_MAX) ||
		 (Runs_Test_Buffer[1][1] < Runs_Test_1_MIN) || (Runs_Test_Buffer[1][1] > Runs_Test_1_MAX) ||
		 (Runs_Test_Buffer[0][2] < Runs_Test_2_MIN) || (Runs_Test_Buffer[0][2] > Runs_Test_2_MAX) ||
		 (Runs_Test_Buffer[1][2] < Runs_Test_2_MIN) || (Runs_Test_Buffer[1][2] > Runs_Test_2_MAX) ||
		 (Runs_Test_Buffer[0][3] < Runs_Test_3_MIN) || (Runs_Test_Buffer[0][3] > Runs_Test_3_MAX) ||
		 (Runs_Test_Buffer[1][3] < Runs_Test_3_MIN) || (Runs_Test_Buffer[1][3] > Runs_Test_3_MAX) ||
		 (Runs_Test_Buffer[0][4] < Runs_Test_4_MIN) || (Runs_Test_Buffer[0][4] > Runs_Test_4_MAX) ||
		 (Runs_Test_Buffer[1][4] < Runs_Test_4_MIN) || (Runs_Test_Buffer[1][4] > Runs_Test_4_MAX) ||
		 (Runs_Test_Buffer[0][5] < Runs_Test_5_MIN) || (Runs_Test_Buffer[0][5] > Runs_Test_5_MAX) ||
		 (Runs_Test_Buffer[1][5] < Runs_Test_5_MIN) || (Runs_Test_Buffer[1][5] > Runs_Test_5_MAX) ||
		 (Runs_Test_Buffer[0][6] < Runs_Test_6_MIN) || (Runs_Test_Buffer[0][6] > Runs_Test_6_MAX) ||
		 (Runs_Test_Buffer[1][6] < Runs_Test_6_MIN) || (Runs_Test_Buffer[1][6] > Runs_Test_6_MAX))
	{
		TEST_RNG_BYTE = (TEST_RNG_BYTE | RUNS_TEST_ERROR_BIT);
	}
}
/* USER CODE END 4 */

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
