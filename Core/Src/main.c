/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssd1306.h"
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP_REFESH_RATE 1000 // ms
#define CAN_TX_TIMEOUT 100 // ms
#define DEADBEEF_REFESH_RATE 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char lcd_line[32];

const char *day_of_week[] =
{ "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData101[8];
uint8_t canDatacheck101 = 0;
uint8_t rxData102[8];
uint8_t canDatacheck102 = 0;

CAN_TxHeaderTypeDef txHeader;
uint8_t txData[8];
uint32_t txMailbox;
uint16_t txTimeoutCounter;

//
// BMP280
//
BMP280_t Bmp280;
uint32_t SoftTimerBmp;
uint32_t SoftTimerDeadBeef;
float Pressure, Temperature;

typedef union
{
	struct
	{
		uint8_t LessSignificantByte;
		uint8_t MoreSignificantByte;
	};
	uint16_t TwoBytes;
} twoBytes_t;

twoBytes_t PressureRounded;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SendCanMessage(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	BMP280_Init(&Bmp280, &hi2c1, 0x76);

	HAL_CAN_Start(&hcan1);

	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA; // https://www.kvaser.com/can-protocol-tutorial/
	txHeader.TransmitGlobalTime = DISABLE;
	// Activate the notification
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 5);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(25, 15);
	ssd1306_WriteString("CAN 2.0B demo", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

	SoftTimerBmp = HAL_GetTick();
	SoftTimerDeadBeef = HAL_GetTick();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (1 == canDatacheck101)
		{
			canDatacheck101 = 0;
			ssd1306_SetCursor(2, 35);
			sprintf(lcd_line, "RX FIFO0: %02d:%02d:%02d", rxData101[0],
					rxData101[1], rxData101[2]);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();
		}

		if (1 == canDatacheck102)
		{
			canDatacheck102 = 0;
			ssd1306_SetCursor(2, 45);
			sprintf(lcd_line, "RX FIFO1: %02d.%02d.%04d", rxData102[0],
					rxData102[1], (2000 + rxData102[2]));
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_SetCursor(2, 55);
			sprintf(lcd_line, "RX FIFO1: %s", day_of_week[rxData102[3] - 1]);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();
		}

		if ((HAL_GetTick() - SoftTimerBmp) > BMP_REFESH_RATE)
		{
			SoftTimerBmp = HAL_GetTick();

			if (Bmp280.bmp_i2c->State == HAL_I2C_STATE_READY)
			{
				BMP280_ReadPressureAndTemperature(&Bmp280, &Pressure,
						&Temperature);

				txHeader.DLC = 1;  // data length
				txHeader.StdId = 0x103;  // ID of the message/frame
				txData[0] = (uint8_t) (round(Temperature));  // temperature
				SendCanMessage();

				txHeader.DLC = 2;  // data length
				txHeader.StdId = 0x107;  // ID of the message/frame
				PressureRounded.TwoBytes = (uint16_t) Pressure; // pressure
				txData[0] = PressureRounded.MoreSignificantByte;
				txData[1] = PressureRounded.LessSignificantByte;
				SendCanMessage();
			}
		}

		if ((HAL_GetTick() - SoftTimerDeadBeef) > DEADBEEF_REFESH_RATE)
		{
			SoftTimerDeadBeef = HAL_GetTick();

			txHeader.DLC = 8;  // data length
			txHeader.StdId = 0x104;  // ID of the message/frame
			txData[0] = 0xD;
			txData[1] = 0xE;
			txData[2] = 0xA;
			txData[3] = 0xD;
			txData[4] = 0xB;
			txData[5] = 0xE;
			txData[6] = 0xE;
			txData[7] = 0xF;
			SendCanMessage();
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData101);
	if (rxHeader.DLC == 3)
	{
		canDatacheck101 = 1;
		HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData102);
	if (rxHeader.DLC == 4)
	{
		canDatacheck102 = 1;
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	}
}

void SendCanMessage(void)
{
	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK)
	{
		Error_Handler();
	}
	txTimeoutCounter = 0;
	while ((HAL_CAN_IsTxMessagePending(&hcan1, txMailbox))
			&& (txTimeoutCounter < CAN_TX_TIMEOUT))
	{
		//				__NOP();
		txTimeoutCounter++;
		HAL_Delay(1);
	}
	if (txTimeoutCounter == CAN_TX_TIMEOUT)
	{
		Error_Handler();
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
