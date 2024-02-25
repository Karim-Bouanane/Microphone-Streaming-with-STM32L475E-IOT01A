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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "wifi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SaturaLH(N, L, H) 				(((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define AUDIO_BUFFER_SIZE							2000
#define AUDIO_WAITING_SEND_SIGNAL			0
#define AUDIO_WAITING_BUFFER					1
#define AUDIO_HALF_BUFFER_AVIALABLE		2
#define AUDIO_FULL_BUFFER_AVAILABLE		3

// WiFi
#define AP_SSID     									"stm32audioserver"
#define AP_PASSWORD 									"12345678"
#define PORT           								5024

#define WIFI_WRITE_TIMEOUT 						10000
#define WIFI_READ_TIMEOUT  						10000
#define SOCKET              					0

// LOG
#define LOG(a)  											printf a


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Audio
uint8_t audio_buffer_status 									= AUDIO_WAITING_SEND_SIGNAL;
int32_t raw_audio_buff[AUDIO_BUFFER_SIZE] 		= {0};
int32_t conv_audio_buff[AUDIO_BUFFER_SIZE] 		= {0};

// WiFi
static  uint8_t 	http[1024]	= {0};
static  uint8_t  	IP_Addr[4]	= {0};
static  int     	LedState 		= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */


static  int wifi_init(void);
static  int wifi_shareAP(void);
static  int wifi_server(void);
static  bool WebServerProcess(void);
static  WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature);

static void start_wifi_cli(void);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  MX_FATFS_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  if (wifi_init() != 0)
  {
  	while(1);
  }

  if (wifi_shareAP() != 0)
	{
  	asm("nop");
	}

  if (wifi_server() != 0)
	{
  	asm("nop");
	}

  start_wifi_cli();

//  http://stm32audioserver:5024

//  /* Wait HTTP request from Clients */
//
//	#define WIFI_PACKET_MAX_SIZE 								1460
//  uint8_t  client_data[WIFI_PACKET_MAX_SIZE] = {0};
//  uint16_t max_packet_size 									 = 1200;			// must not exceed 1460 bytes
//  uint16_t read_timeout 										 = 1000;			// 1 second
//  uint16_t write_timeout 										 = 5000;			// 5 second
//  uint16_t rcvDataLen												 = 0;
//  uint16_t sentDataLen											 = 0;
//  char 		*token_ptr 												 = NULL;
//  WIFI_Status_t ret 												 = WIFI_STATUS_ERROR;
//	char *homePageResponse = "HTTP/1.1 200 OK\r\n "
//											"Content-Type: text/html\r\n"
//											"\r\n"
//											"<html><body><h1>Hello, World!</h1></body></html>\r\n"
//											"\r\n";
//
//	do
//  {
//  		ret = WIFI_ReceiveData(socket, client_data, max_packet_size, &rcvDataLen, read_timeout);
//  		if(ret != WIFI_STATUS_OK)
//  		{
//  				HandleClientError("Failed to Receive Data");
//  		}
//
//  		if(rcvDataLen > 0)
//  		{
//					char datalen_str[100] = {0};
//  				HAL_UART_Transmit(&huart1, (uint8_t *)"-----> New Message \r\n", strlen("-----> New Message \r\n"), HAL_MAX_DELAY);
//					sprintf((char*)datalen_str, "RcvDataLen : %d\r\n", rcvDataLen);
//					HAL_UART_Transmit(&huart1, (uint8_t *)datalen_str, strlen(datalen_str), HAL_MAX_DELAY);
//  				HAL_UART_Transmit(&huart1, (uint8_t *)client_data, rcvDataLen, HAL_MAX_DELAY);
//  				HAL_UART_Transmit(&huart1, (uint8_t *)"-----> End Message \r\n", strlen("-----> End Message \r\n"), HAL_MAX_DELAY);
//
//  				// check if connection is lost
//  				token_ptr = strstr((char *)client_data, "-1\r\nERROR:");
//  				if (token_ptr != NULL)
//  				{
//							//	Response= "-1"
//							//	Any one of the following reasons will cause a "-1" response:
//							//	1. Connection to the network (AP) has been lost
//							//	2. Other side has closed the connection
//							//	3. Other side has lost it connection to the network
//							printf("Connection lost\r\n");
//  				}
//  				else
//  				{
//  						// check if GET request is received
//							token_ptr = strstr((char *)client_data, "GET /");
//							if (token_ptr != NULL)
//							{
//									printf("Received GET Request\r\n");
//									token_ptr = strstr((char *)client_data, "GET / HTTP/1.1");
//									if (token_ptr != NULL)
//									{
//											printf("Home Page Request Received\r\n");
//											ret = WIFI_SendData(socket, (uint8_t *)homePageResponse, strlen(homePageResponse), &sentDataLen, write_timeout);
//											if(ret == WIFI_STATUS_OK)
//											{
//													printf("Home Page well sent\r\n");
//											}
//											else
//											{
//													printf("Error: Data aren't sent to client\r\n");
//											}
//									}
//							}
//  				}
//  		}
//
//  }
//  while(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t preambule[4] = "Send";
  uint8_t rcv_data[4] = {0};

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, raw_audio_buff, AUDIO_BUFFER_SIZE) != HAL_OK)
  {
	  Error_Handler();
  }

  while (1)
  {
		switch (audio_buffer_status)
		{
			case AUDIO_WAITING_SEND_SIGNAL:

				if (HAL_UART_Receive(&huart1, (uint8_t *)rcv_data, 4, HAL_MAX_DELAY) == HAL_OK)
				{
					if (memcmp(rcv_data, preambule, 4) == 0)
					{
						// wait for DMA to fill buffer
						audio_buffer_status = AUDIO_WAITING_BUFFER;
					}
				}
				break;

			case AUDIO_HALF_BUFFER_AVIALABLE:

				for(uint16_t sample_index = 0; sample_index < AUDIO_BUFFER_SIZE / 2; sample_index ++)
				{
					conv_audio_buff[sample_index] = SaturaLH(((int32_t)raw_audio_buff[sample_index] >> 8), -32768, 32767);
					HAL_UART_Transmit(&huart1, (uint8_t *)&conv_audio_buff[sample_index], 2, HAL_MAX_DELAY);
				}
				audio_buffer_status = AUDIO_WAITING_SEND_SIGNAL;
				break;

			case AUDIO_FULL_BUFFER_AVAILABLE:

				for(uint16_t sample_index = AUDIO_BUFFER_SIZE / 2; sample_index < AUDIO_BUFFER_SIZE; sample_index ++)
				{
					conv_audio_buff[sample_index] = SaturaLH(((int32_t)raw_audio_buff[sample_index] >> 8), -32768, 32767);
					HAL_UART_Transmit(&huart1, (uint8_t *)&conv_audio_buff[sample_index], 2, HAL_MAX_DELAY);
				}
				audio_buffer_status = AUDIO_WAITING_SEND_SIGNAL;
				break;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1200000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_I2C2_SCL_Pin INTERNAL_I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = INTERNAL_I2C2_SCL_Pin|INTERNAL_I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if(audio_buffer_status == AUDIO_WAITING_BUFFER)
	{
		audio_buffer_status = AUDIO_HALF_BUFFER_AVIALABLE;
	}
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if(audio_buffer_status == AUDIO_WAITING_BUFFER)
	{
		audio_buffer_status = AUDIO_FULL_BUFFER_AVAILABLE;
	}
}


static int wifi_init(void)
{
  uint8_t  MAC_Addr[6];

  /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
    {
      LOG(("eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]));
    }
    else
    {
      LOG(("> ERROR : CANNOT get MAC address\r\n"));
      return -1;
    }
  }
  else
  {
  	LOG(("> ERROR : ES-WIFI Not Initialized\r\n"));
    return -1;
  }
  return 0;
}


int wifi_shareAP(void)
{
  uint8_t channel = 1;
  uint8_t max_conn = 2;

  if (WIFI_ConfigureAP((const uint8_t *)AP_SSID, (const uint8_t *)AP_PASSWORD, WIFI_ECN_WPA2_PSK, channel, max_conn) == WIFI_STATUS_OK)
  {
    LOG(("Connect to this Access Point -> SSID: %s , PWD: %s\r\n", AP_SSID, AP_PASSWORD));

    if (WIFI_GatewayIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
    {
      LOG(("Gateway IP Address : %d.%d.%d.%d\r\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]));
    }
    else
    {
		  LOG((" ERROR : es-wifi module CANNOT get IP address\r\n"));
      return -1;
    }
  }
  else
  {
		 LOG(("ERROR : es-wifi module NOT connected\r\n"));
     return -1;
  }
  return 0;
}


int wifi_server(void)
{
  bool StopServer = false;

  LOG(("Running HTML Server \r\n"));

  if (WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT) != WIFI_STATUS_OK)
  {
    LOG(("ERROR: Cannot start server.\r\n"));
  }

  LOG(("Server is running and waiting for an HTTP  Client connection to %d.%d.%d.%d\r\n",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]));

  do
  {
    uint8_t RemoteIP[4];
    uint16_t RemotePort;

    while (WIFI_WaitServerConnection(SOCKET, 1000, RemoteIP, sizeof(RemoteIP), &RemotePort) != WIFI_STATUS_OK)
    {
        LOG(("Waiting connection to  %d.%d.%d.%d\r\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]));
    }

    LOG(("Client connected %d.%d.%d.%d:%d\r\n", RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3], RemotePort));

    StopServer = WebServerProcess();

    if(WIFI_CloseServerConnection(SOCKET) != WIFI_STATUS_OK)
    {
      LOG(("ERROR: failed to close current Server connection\r\n"));
      return -1;
    }
  }
  while(StopServer == false);

  if (WIFI_StopServer(SOCKET) != WIFI_STATUS_OK)
  {
    LOG(("ERROR: Cannot stop server.\n"));
  }

  LOG(("Server is stop\n"));
  return 0;
}


static bool WebServerProcess(void)
{
  uint8_t temp;
  uint16_t respLen;
  static uint8_t resp[1024];
  bool stopserver = false;

  if (WIFI_STATUS_OK == WIFI_ReceiveData(SOCKET, resp, 1000, &respLen, WIFI_READ_TIMEOUT))
  {
   LOG(("get %d byte from server\r\n", respLen));

   if( respLen > 0)
   {
      if(strstr((char *)resp, "GET")) /* GET: put web page */
      {
        temp = (int) 20; //BSP_TSENSOR_ReadTemp();
        if(SendWebPage(LedState, temp) != WIFI_STATUS_OK)
        {
          LOG(("> ERROR : Cannot send web page\r\n"));
        }
        else
        {
          LOG(("Send page after  GET command\r\n"));
        }
       }
       else if(strstr((char *)resp, "POST"))/* POST: received info */
       {
         LOG(("Post request\n"));

         if(strstr((char *)resp, "radio"))
         {
           if(strstr((char *)resp, "radio=0"))
           {
             LedState = 0;
             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
             //BSP_LED_Off(LED2);
           }
           else if(strstr((char *)resp, "radio=1"))
           {
             LedState = 1;
             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
             //BSP_LED_On(LED2);
           }
           temp = (int) 20; //BSP_TSENSOR_ReadTemp();
         }
         if(strstr((char *)resp, "stop_server"))
         {
           if(strstr((char *)resp, "stop_server=0"))
           {
             stopserver = false;
           }
           else if(strstr((char *)resp, "stop_server=1"))
           {
             stopserver = true;
           }
         }
         temp = (int) 20; //(int) BSP_TSENSOR_ReadTemp();
         if(SendWebPage(LedState, temp) != WIFI_STATUS_OK)
         {
           LOG(("> ERROR : Cannot send web page\r\n"));
         }
         else
         {
           LOG(("Send Page after POST command\r\n"));
         }
       }
     }
  }
  else
  {
    LOG(("Client close connection\r\n"));
  }

  return stopserver;
}


/**
  * @brief  Send HTML page
  * @param  None
  * @retval None
  */
static WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature)
{
  uint8_t  temp[50];
  uint16_t SentDataLength;
  WIFI_Status_t ret;

  /* construct web page content */
  strcpy((char *)http, (char *)"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n");
  strcat((char *)http, (char *)"<html>\r\n<body>\r\n");
  strcat((char *)http, (char *)"<title>STM32 Web Server</title>\r\n");
  strcat((char *)http, (char *)"<h2>InventekSys : Web Server using Es-Wifi with STM32</h2>\r\n");
  strcat((char *)http, (char *)"<br /><hr>\r\n");
  strcat((char *)http, (char *)"<p><form method=\"POST\"><strong>Temp: <input type=\"text\" value=\"");
  sprintf((char *)temp, "%d", temperature);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\"> <sup>O</sup>C");

  if (ledIsOn)
  {
    strcat((char *)http, (char *)"<p><input type=\"radio\" name=\"radio\" value=\"0\" >LED off");
    strcat((char *)http, (char *)"<br><input type=\"radio\" name=\"radio\" value=\"1\" checked>LED on");
  }
  else
  {
    strcat((char *)http, (char *)"<p><input type=\"radio\" name=\"radio\" value=\"0\" checked>LED off");
    strcat((char *)http, (char *)"<br><input type=\"radio\" name=\"radio\" value=\"1\" >LED on");
  }

  strcat((char *)http, (char *)"</strong><p><input type=\"submit\"></form></span>");
  strcat((char *)http, (char *)"</body>\r\n</html>\r\n");

  ret = WIFI_SendData(0, (uint8_t *)http, strlen((char *)http), &SentDataLength, WIFI_WRITE_TIMEOUT);

  if((ret == WIFI_STATUS_OK) && (SentDataLength != strlen((char *)http)))
  {
    ret = WIFI_STATUS_ERROR;
  }

  return ret;
}


#include "es_wifi.h"

static ES_WIFI_Status_t AT_ExecuteCommand(ES_WIFIObject_t *Obj, const uint8_t *cmd, uint8_t *pdata)
{
  int ret = 0;
  int16_t recv_len = 0;

  LOCK_WIFI();

  if ((Obj->fops.IO_Send != NULL) && (Obj->fops.IO_Receive != NULL)) {

  ret = Obj->fops.IO_Send(cmd, strlen((const char *)cmd), Obj->Timeout);

  if( ret > 0)
  {
    recv_len = Obj->fops.IO_Receive(pdata, ES_WIFI_DATA_SIZE, Obj->Timeout);
    if ((recv_len > 0) && (recv_len <= ES_WIFI_DATA_SIZE))
    {
      if (recv_len == ES_WIFI_DATA_SIZE)
      {
        /* ES_WIFI_DATA_SIZE maybe too small !! */
        recv_len--;
      }
      *(pdata + recv_len) = 0;

      if (strstr((char *)pdata, "\r\nOK\r\n> "))
      {
        UNLOCK_WIFI();
        return ES_WIFI_STATUS_OK;
      }
      else if (strstr((char *)pdata, "\r\nERROR"))
      {
        UNLOCK_WIFI();
        return ES_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
      }
    }
    if (recv_len == ES_WIFI_ERROR_STUFFING_FOREVER)
    {
      UNLOCK_WIFI();
      return ES_WIFI_STATUS_MODULE_CRASH;
    }
   }
  }
  UNLOCK_WIFI();

  return ES_WIFI_STATUS_IO_ERROR;
}

static void start_wifi_cli(void)
{
	/* User CLI */

	  uint8_t wifi_cmd[100] = {0};
	  uint8_t *cmd_byte = wifi_cmd;

		while(1)
		{

			if (HAL_UART_Receive(&huart1, cmd_byte, 1, 500) == HAL_OK)
			{
				if((*cmd_byte) == '\r')
				{
					*(++cmd_byte) = 0; 		// string terminator
					cmd_byte = wifi_cmd;

					extern ES_WIFIObject_t EsWifiObj;

					if (AT_ExecuteCommand(&EsWifiObj, (uint8_t*)wifi_cmd, EsWifiObj.CmdData) == ES_WIFI_STATUS_OK)
					{
						HAL_UART_Transmit(&huart1, (uint8_t *)EsWifiObj.CmdData, strlen((char*)EsWifiObj.CmdData), HAL_MAX_DELAY);
					}
				}
				else
				{
				  cmd_byte++;
				}
			}
			else
			{
				cmd_byte = wifi_cmd;
			}
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
