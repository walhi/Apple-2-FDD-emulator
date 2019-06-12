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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "dsk2nic.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _Error_Handler(x,y)
#define NIC_SECTOR_SIZE 512
#define DSK_SECTOR_SIZE 256
#define SECTORS_COUNT 16
#define TRUE  1
#define FALSE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS fs;
FATFS *pfs;
FIL fil;
FIL fil2;
uint8_t nic_buf[NIC_SECTOR_SIZE];
uint8_t dsk_buf[DSK_SECTOR_SIZE+2];

uint16_t lastTimerValue;

uint8_t writeChangeFlag = FALSE;
uint8_t readChangeFlag = FALSE;

volatile uint changes = 0;

volatile uint8_t track = 0;
volatile uint8_t sector = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void changeSector()
{
	sector = (sector >= 15)?0:sector+1;
	//uint8_t* tmp = currentSectorBuf;
	//currentSectorBuf = nextSectorBuf;
	//nextSectorBuf = tmp;
}


void writeMode(void){
	//memset(currentSectorBuf, 0x00, SECTOR_SIZE);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	writeChangeFlag = FALSE;
}

void readMode(void){
	//prepare buffers
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);

	//start transmit
	//HAL_SPI_Transmit_DMA(&hspi1, buf, sizeof(buf));
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	readChangeFlag = FALSE;
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	//changeSector();
	// get next sector from sd card
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	changeSector();
	// convert dsk to nic
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
}

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint8_t *data;
	static uint8_t pos;
	switch (GPIO_Pin){
	case WRITE_REQUEST_Pin:
		if (HAL_GPIO_ReadPin(WRITE_REQUEST_GPIO_Port, WRITE_REQUEST_Pin) == GPIO_PIN_RESET){
			// Stop read pulse
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_SPI_Abort(&hspi1);
			changes = 0;

			HAL_TIM_Base_Init(&htim1);
			HAL_TIM_Base_Start(&htim1);

			//HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			//data = currentSectorBuf;
			pos = 0;
			writeChangeFlag = TRUE;
		} else {
			//HAL_NVIC_DisableIRQ(EXTI2_IRQn);
			readChangeFlag = TRUE;
		}
		break;
	case PHASE0_Pin:
		if (lastPhase == PHASE1){
			if (track > 0){
				track--;
			}
		} else if (lastPhase == PHASE3) {
			if (track < 34){
				track++;
			}
		}
		lastPhase = PHASE0;
		break;
	case PHASE1_Pin:
		lastPhase = PHASE1;
		break;
	case PHASE2_Pin:
		if (lastPhase == PHASE3){
			if (track > 0){
				track--;
			}
		} else if (lastPhase == PHASE1) {
			if (track < 34){
				track++;
			}
		}
		lastPhase = PHASE2;
		break;
	case PHASE3_Pin:
		lastPhase = PHASE3;
		break;
	case WRITE_SIGNAL_Pin:
		changes++;
		uint16_t ticks = __HAL_TIM_GET_COUNTER(&htim1);
		if (ticks >= 7 && ticks < 14){
			// 1
			*data |= (1 << pos);
			pos++;
			if (pos >= 8){
				data++;
				pos = 0;
			}
		} else if (ticks >= 14 && ticks < 21){
			// 01
			*data &= ~(1 << pos);
			pos++;
			if (pos >= 8){
				data++;
				pos = 0;
			}
			*data |= (1 << pos);
			pos++;
			if (pos >= 8){
				data++;
				pos = 0;
			}
		} else if (ticks >= 21 && ticks < 28){
			// 001
			*data &= ~(1 << pos);
			pos++;
			if (pos >= 8){
				data++;
				pos = 0;
			}
			*data &= ~(1 << pos);
			pos++;
			if (pos >= 8){
				data++;
				pos = 0;
			}
			*data |= (1 << pos);
			pos++;
			if (pos >= 8){
				data++;
				pos = 0;
			}
		} else {
			// ?
		}

		break;
	}
}
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
	uint8_t oldTrack = !track;
	uint8_t oldSector = !sector;
	GPIO_PinState oldDRIVE1_ENABLE;
	volatile FRESULT res;
	char uart_buf[20];

	uint16_t readed;
	memset(uart_buf, 0, sizeof(uart_buf));

	//HAL_HalfDuplex_EnableTransmitter(&huart1);

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
  MX_DMA_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	oldDRIVE1_ENABLE = GPIO_PIN_SET;//HAL_GPIO_ReadPin(DRIVE_ENABLE_GPIO_Port, DRIVE_ENABLE_Pin);

  /* Enable the Half transfer complete interrupt */
  //__HAL_DMA_ENABLE_IT(&hdma_spi1_tx, DMA_IT_HT);
	//HAL_DMA_RegisterCallback(&hdma_spi1_tx, HAL_DMA_XFER_CPLT_CB_ID, changeSector);
	//HAL_DMA_RegisterCallback(&hdma_spi1_tx, HAL_DMA_XFER_HALFCPLT_CB_ID, &changeSector);


	HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 0xFFFF);

  /* Mount SD Card */
  if(f_mount(&fs, "", 0) != FR_OK){
		// SHOW MESSAGE ON APPLE 2
    _Error_Handler(__FILE__, __LINE__);
	}


	  /* Open file to read */
  if(res = f_open(&fil, "BOOT.DSK", FA_READ) != FR_OK){
		// SHOW MESSAGE ON APPLE 2

    _Error_Handler(__FILE__, __LINE__);
	}


  HAL_TIM_Base_Start(&htim3);
	//HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//HAL_TIM_OnePulse_Start(&htim1, TIM_CHANNEL_3);
	//currentSectorBuf = buf[0];
	//nextSectorBuf = buf[1];

	//f_lseek(&fil, 0);


	//res = f_read(&fil, dsk_buf, DSK_SECTOR_SIZE, &readed);
	//dsk2nic(dsk_buf, nic_buf, track, sector);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (HAL_GPIO_ReadPin(DRIVE1_ENABLE_GPIO_Port, DRIVE1_ENABLE_Pin) != oldDRIVE1_ENABLE){
			if (oldDRIVE1_ENABLE){
				// run spi
				HAL_SPI_Transmit_DMA(&hspi1, nic_buf, sizeof(nic_buf));
			} else {
				// stop spi
				HAL_SPI_DMAStop(&hspi1);
			}
			oldDRIVE1_ENABLE = !oldDRIVE1_ENABLE;
		}

		if (oldTrack != track){
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			sprintf(uart_buf, "\n\rtr: %d\n\r", track);
			HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 0xFFFF);
			memset(nic_buf, 0x00, NIC_SECTOR_SIZE);
			oldSector = !sector;
			oldTrack = track;
			//if (track > 34) continue;
		}

		if (oldSector != sector){
			HAL_SPI_DMAStop(&hspi1);
			sprintf(uart_buf, "sec: %d\r", sector);
			HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 0xFFFF);
			oldSector = sector;
			f_lseek(&fil, ((DSK_SECTOR_SIZE * SECTORS_COUNT * track) + (DSK_SECTOR_SIZE * dsk_scramble[sector])));
			//f_lseek(&fil, ((DSK_SECTOR_SIZE * SECTORS_COUNT * track) + (DSK_SECTOR_SIZE * sector)));

			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			f_read(&fil, dsk_buf, DSK_SECTOR_SIZE, &readed);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			dsk2nic(dsk_buf, nic_buf, track, sector);
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_SPI_Transmit_DMA(&hspi1, nic_buf, sizeof(nic_buf));
		}

		if (writeChangeFlag) writeMode();
		if (readChangeFlag) readMode();
  }

  /* Close file */
  if(f_close(&fil) != FR_OK)
    _Error_Handler(__FILE__, __LINE__);

  /* Unmount SDCARD */
  if(f_mount(NULL, "", 1) != FR_OK)
    _Error_Handler(__FILE__, __LINE__);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WRITE_SIGNAL_Pin WRITE_REQUEST_Pin */
  GPIO_InitStruct.Pin = WRITE_SIGNAL_Pin|WRITE_REQUEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PHASE0_Pin PHASE1_Pin */
  GPIO_InitStruct.Pin = PHASE0_Pin|PHASE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PHASE2_Pin PHASE3_Pin */
  GPIO_InitStruct.Pin = PHASE2_Pin|PHASE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_DETECT_Pin SD_WREN_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin|SD_WREN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DRIVE1_ENABLE_Pin DRIVE2_ENABLE_Pin */
  GPIO_InitStruct.Pin = DRIVE1_ENABLE_Pin|DRIVE2_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
