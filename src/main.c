/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cube_hal.h"
#include "radio_shield_config.h"
#include "radio_appli.h"
#include "p2p_lib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t tnfType;
  /* for payload length, assume short record up to 255 bytes */
  uint8_t payloadLength;
  /* payload container is only 246 bytes */
  char payloadContainer[0x86];
} ndefInfo_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFFER_SIZE 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char aTransmitBuffer[] = "Hello World";
uint8_t TxLength = sizeof(aTransmitBuffer);
char newline[2] = "\r\n";
sGeoInfo geoS = {"0.0", "0.0", "Info"};
uint16_t status;
volatile uint8_t flagGPO = 0;
/* CCBuffer holds the contents of first 15 bytes of CCfile of M24SR */
uint8_t CCBuffer[15];
extern SM_State_t SM_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#if defined(USE_LOW_POWER_MODE)
static void SystemPower_Config(void);
#endif
void HAL_Delay(uint32_t Delay);
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
  /*
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
	SystemClock_Config();
#endif
	 */

#if defined(MCU_SLEEP_MODE)
  {
    HAL_EnableDBGSleepMode();
  }
#endif
#if defined(MCU_STOP_MODE)
  {
    HAL_EnableDBGStopMode();
  }
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize LEDs*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
//	RadioShieldLedInit(RADIO_SHIELD_LED);
#endif

  //	BSP_LED_Init(LED2);
  HAL_Radio_Init();

  /* Initialize Buttons*/
  //	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  P2P_Init();
  //  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Wait for the GPO interrupt */
    while (flagGPO != 1)
    {
      HAL_Delay(1000);
      LED3_GPIO_Port->ODR ^= LED3_Pin;
    }
    flagGPO = 0;
    ReadGeoL(&geoS);
    LED1_GPIO_Port->ODR ^= LED1_Pin;
    //
    P2P_Transmit((uint8_t *)(&geoS.Information), 20  );
    SM_State = SM_STATE_SEND_DATA;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin | LED3_Pin | LD2_Pin | SP1_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFDIS_GPIO_Port, RFDIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_LED_Pin | CSn_Pin | LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin SP1_GPIO3_Pin GPO_Pin */
  GPIO_InitStruct.Pin = SP1_GPIO3_Pin | GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC7 PC8 PC9 
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |  GPIO_PIN_7 |GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LD2_Pin 
                           SP1_SDN_Pin */
  GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LD2_Pin | SP1_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins :PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_DIS_Pin */
  GPIO_InitStruct.Pin = RFDIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFDIS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin CSn_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin | CSn_Pin | LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */
/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a period to guaranty minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait++;
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}
void M24SR_WaitMs(uint32_t time_ms)
{
  wait_ms(time_ms);
}

/**
 * @brief  This function retrieve current tick
 * @param	ptickstart: pointer on a variable to store current tick value
 */
void M24SR_GetTick(uint32_t *ptickstart)
{
  *ptickstart = HAL_GetTick();
}

/**
 * @brief  This function read the state of the M24SR GPO
 * @param	none
 * @retval GPIO_PinState : state of the M24SR GPO
 */
void M24SR_GPO_ReadPin(GPIO_PinState *pPinState)
{
  *pPinState = HAL_GPIO_ReadPin(M24SR_GPO_PIN_PORT, M24SR_GPO_PIN);
}

/**
 * @brief  This function set the state of the M24SR RF disable pin
 * @param	PinState: put RF disable pin of M24SR in PinState (1 or 0)
 */
void M24SR_RFDIS_WritePin(GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(M24SR_RFDIS_PIN_PORT, M24SR_RFDIS_PIN, PinState);
}

/* Set GPO state for RF and I2C sessions */
uint16_t setGPOState(void)
{
  uint16_t status = ERROR;
  uint16_t trials = 5; /* wait 1sec, driver is configured to let 200ms for command to complete */
  /* which is enough for all commands except GetSession if RF session is already opened */
  /* Smartphone generaly release the session within the second, but customer can modify this value */

  /* Perform HW initialization */
  M24SR_Init();

  /* Read CC file */
  while (status != M24SR_ACTION_COMPLETED && trials)
  {
    status = M24SR_GetSession();
    trials--;
  }
  if (status != M24SR_ACTION_COMPLETED)
    return ERROR;
  /* Write GPO purpose for RF and I2C sessions */
  errorchk(M24SR_ManageRFGPO(SESSION_OPENED));
  errorchk(M24SR_ManageI2CGPO(HIGH_IMPEDANCE));
  /* read the first 15 bytes of the CC file */

  errorchk(M24SR_Deselect());
  return SUCCESS;

Error:
  return ERROR;
}

/* A very simple function to read longitude and latitude from NFC chip memory */
uint16_t ReadGeoL(sGeoInfo *pGeoStruct)
{
  /* File ID for CCFile */
  uint16_t FileId;
  /* Create a data 32 byte data buffer */
  uint8_t ndefBuffer[NDEF_MAX_SIZE] = {0};
  /* Definition of NDEFInfo type variable and its pointer */
  ndefInfo_t NDEFInfo = {0x00, 0x00, {0}}, *pNDEFInfo;
  pNDEFInfo = &NDEFInfo;

  /* Return status */
  uint16_t status = 1;

  /* Define a string to hold the geolocation data */
  char geoString[140] = {"0"};
  char *pGeoString;
  pGeoString = geoString;

  /* Clear *pGeoStruct */
  strncpy(pGeoStruct->Information, "0", 100);
  strncpy(pGeoStruct->Latitude, "0", 20);
  strncpy(pGeoStruct->Longitude, "0", 20);

  while (TagT4Init(CCBuffer, sizeof(CCBuffer)) != SUCCESS)
    ;
  //	__NOP();
  if (SUCCESS == GetNDEFFileId(&FileId))
  {
    if (SUCCESS == OpenNDEFSession(FileId, ASK_FOR_SESSION))
    {
      NDEF_ReadNDEF(ndefBuffer);
      pNDEFInfo->tnfType = ndefBuffer[FIRST_RECORD_OFFSET] & TNF_Mask;
      /* if tnfType=0x01, then proceed} */
      if (pNDEFInfo->tnfType == TNF_WellKnown)
      {
        pNDEFInfo->payloadLength = ndefBuffer[FIRST_RECORD_OFFSET + 2];
        memcpy(pNDEFInfo->payloadContainer, (uint8_t *)(ndefBuffer + FIRST_RECORD_OFFSET + 5), pNDEFInfo->payloadLength);
        /* Check if there is "geo:" string is in the payloadContainer array.
				 * If there is not, just signal the fact.
				 * If there is, split the string using comma character between digits.
				 * We could do more tests but it is enough for our purpose at the moment.
				 * It somehow does not work when there are two records in NDEF. MHE. 9/6/2019.
				 */
        if (strstr(pNDEFInfo->payloadContainer, GEO_TYPE_STRING) == NULL)
        {
          strcpy(pGeoStruct->Information, "Location Info not given");
          strcpy(pGeoStruct->Latitude, "undefined");
          strcpy(pGeoStruct->Longitude, "undefined");
        }
        else
        {
          pGeoString = (char *)(strstr(pNDEFInfo->payloadContainer, GEO_TYPE_STRING) + GEO_TYPE_STRING_LENGTH);
          strcpy(pGeoStruct->Information, "Location");
          strcpy(pGeoStruct->Latitude, strtok(pGeoString, ","));
          strcpy(pGeoStruct->Longitude, strtok(NULL, ","));
        }
      }
    }
    while (M24SR_I2CTokenRelease() != M24SR_STATUS_SUCCESS)
      ;
    status = CloseNDEFSession(FileId);
  }
  return status;
}

/**
 * @brief EXTI line detection callback.
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */

#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
extern uint16_t M2S_GPIO_PIN_IRQ;
#endif
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

#if defined(RF_STANDBY)    /*if S2LP is in standby*/
#if defined(MCU_STOP_MODE) /*if MCU is in stop mode*/

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
	PLL as system clock source (HSE and PLL are disabled in STOP mode) */
  SystemClockConfig_ExitSTOPMode();
#endif
#if defined(MCU_SLEEP_MODE)
  /* Resume Tick interrupt if disabled prior to sleep mode entry*/
  HAL_ResumeTick();
#endif

  /* Initialize LEDs*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  RadioShieldLedInit(RADIO_SHIELD_LED);
#endif
#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
  // RadioShieldLedInit(RADIO_SHIELD_LED);
#endif
  BSP_LED_Init(LED2);

  PushButtonStatusWakeup = SET;
  PushButtonStatusData = RESET;
  wakeupCounter = LPM_WAKEUP_TIME;
  dataSendCounter = DATA_SEND_TIME;
  dataSendCounter++;
#endif
  switch (GPIO_Pin)
  {
    /*   case USER_BUTTON_PIN:
    Set_KeyStatus(SET);
    break; */

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  case RADIO_GPIO_3_EXTI_LINE:
    P2PInterruptHandler();
    break;

#endif
  case M24SR_GPO_PIN:
    flagGPO = 1;
    break;
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
