/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/************************************************ DECAWAVE *******************************/
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Configuration enum */
typedef enum
{
    LongData_Fast,
    ShortData_Fast,
		LongData_Range
} configSel_t;

/* FSM state enum */
typedef enum
{
    IDLE,
    RECEIVE_I,
    WAIT,
    PROCESS,
		INITIATOR
} tag_FSM_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/************************************************ DECAWAVE *******************************/
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500

/* Buffer size to hold ranging measurements */
#define RANGE_BUF_SIZE 2000

/* Number of Measurements to perform in each run */
#define NUM_MEASUREMENTS 200

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
//#define POLL_RX_TO_RESP_TX_DLY_UUS 3500			//ShortData_Fast
//#define POLL_RX_TO_RESP_TX_DLY_UUS 3500				//LongData_Fast
#define POLL_RX_TO_RESP_TX_DLY_UUS 3500
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define RESP_TX_TO_FINAL_RX_DLY_UUS 300			//ShortData_Fast
//#define RESP_TX_TO_FINAL_RX_DLY_UUS 300				//LongData_Fast
#define RESP_TX_TO_FINAL_RX_DLY_UUS 300
/* Receive final timeout. See NOTE 5 below. */
//#define FINAL_RX_TIMEOUT_UUS    5000				//ShortData_Fast
//#define FINAL_RX_TIMEOUT_UUS    10000				//LongData_Fast
//#define FINAL_RX_TIMEOUT_UUS    20000				//LongData_Range
#define FINAL_RX_TIMEOUT_UUS    20000

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 300			//ShortData_Fast
//#define POLL_TX_TO_RESP_RX_DLY_UUS 300				//LongData_Fast
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
//#define RESP_RX_TO_FINAL_TX_DLY_UUS 3500		//ShortData_Fast
//#define RESP_RX_TO_FINAL_TX_DLY_UUS 3500			//LongData_Fast
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3500
/* Receive response timeout. See NOTE 5 below. */
//#define RESP_RX_TIMEOUT_UUS 5000						//ShortData_Fast
//#define RESP_RX_TIMEOUT_UUS 10000						//LongData_Fast
//#define RESP_RX_TIMEOUT_UUS 20000						//LongData_Range
#define RESP_RX_TIMEOUT_UUS 20000

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
//#define PRE_TIMEOUT 8
#define PRE_TIMEOUT 60

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/************************************************ DECAWAVE *******************************/

/*	Configuration modes
1. MODE_SHORTDATA_FAST[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};

2. MODE_LONGDATA_RANGE[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};

3. MODE_LONGDATA_FAST[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
*/

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config_ShortData_Fast = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_config_t config_LongData_Range = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_config_t config_LongData_Fast = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,    /* Preamble length. Used in TX only. */
    DWT_PAC64,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 8 - 64)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb_initiator = 0;
static uint8 frame_seq_nb_responder = 0;
	
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
	
/* timing variables to determine the ranging period */
uint64_t t1 = 0;
uint64_t t2 = 0;

dwt_txconfig_t    configTX;

tag_FSM_state_t state = IDLE;

/* Variable to set and select the configuration mode */
configSel_t ConfigSel = ShortData_Fast;

/* Buffer to save ranging distances */
uint16_t numRanged = 0;
double ranges[RANGE_BUF_SIZE];


uint16_t numMeasure = NUM_MEASUREMENTS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

/************************************************ DECAWAVE *******************************/
uint64_t getRxTimestamp(void);
uint64_t getTxTimestamp(void);

/* Declaration of static functions. */
static void MX_DWM_Init(volatile bool responder);
static void initiator_go (uint16_t numMeasure);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void tx_conf_cb(const dwt_cb_data_t *);
static void rx_ok_cb(const dwt_cb_data_t *);
static void rx_to_cb(const dwt_cb_data_t *);
static void rx_err_cb(const dwt_cb_data_t *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small msg_info (option LD Linker->Libraries->Small msg_info
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

/**
  * @brief Retargets the C library printf function to the USB.
  * @param None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the serial port and Loop until the end of transmission */
  while (USBD_OK != CDC_Transmit_FS((uint8_t *) &ch, 1))
  {
    ;
  }
  return ch;
}
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay (3000);
	printf("Starting the Great Application!!\n");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* State machine of the application, application starts in IDLE mode and configs for responder,
			then it goes to RECEIVE_I which enables RX, it then goes to WAIT state to wait for ranging to complete,
			finally, whenever the ranging ends, the application goes to PROCESS state and sends measured distance to the host.
			If the debug button is pressed, application goes into INITIATOR state and starts ranging for the specified number of times.*/
		switch (state)
		{
			case IDLE: 
				printf ("IDLE state\n");
				/* Initilizing Decawave module for responder configuration */
				MX_DWM_Init(1);
				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
				state = RECEIVE_I;
				break;

			case RECEIVE_I:
				//printf("STATE RECEIVE_I\n");
				HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
				 /* Clear reception timeout to start next ranging process. */
				dwt_setrxtimeout(0);
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				state = WAIT;
				break;

			case WAIT:
				break;

			case PROCESS:
				t2 = HAL_GetTick() - t1;
				ranges[numRanged] = distance;
				printf("Measurement: %d, Distance: %f, Time: %llu\n",numRanged,distance,t2);
				numRanged++;

				state = RECEIVE_I;
				break; 
			
			case INITIATOR:
				/* Initilizing Decawave module for initiator configuration */
				HAL_NVIC_DisableIRQ (EXTI2_IRQn);
				printf("Initiator state\n");
	
				MX_DWM_Init (0);	
			
				initiator_go(numMeasure);
				
				state = IDLE ;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	
  //HAL_TIM_Base_Init(&htim2);
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DW_NSS_Pin|DW_WU_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : DW_IRQn_Pin */
  GPIO_InitStruct.Pin = DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_RESET_Pin */
  GPIO_InitStruct.Pin = DW_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DW_NSS_Pin PB6 DW_WU_Pin */
  GPIO_InitStruct.Pin = DW_NSS_Pin|GPIO_PIN_6|DW_WU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	
  HAL_NVIC_SetPriority(EXTI2_IRQn, 4, 0);
  //HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */


	/************************************************ DECAWAVE *******************************/
/**
  * @brief Decawave Initialization Function
  * @param responder	parameter to select responder or initiator mode, to set callbacks for the responder or set the fixed
						time variables for the initiator
  * @retval None
  */
static void MX_DWM_Init(volatile bool responder)
{
	/* Reduce spi baudrate to send commands to the Decawave module */
	HAL_SPI_DeInit (&hspi1);
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	HAL_SPI_Init (&hspi1);
	
	/* Reset and initializing DWM */
	setup_DW1000RSTnIRQ(0);
	reset_DW1000(); 
	port_set_deca_isr (dwt_isr);
	HAL_Delay (500);
	
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)	{
		printf("initialize NOT ok\n");
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		HAL_Delay(400);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(400);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		HAL_Delay(400);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(400);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		HAL_Delay(400);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	}
	else {
		printf("initialize OK\n");
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		HAL_Delay(400);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(400);
	}
  
	/* Increase spi baudrate to exploit the Decawave module speed*/
	HAL_SPI_DeInit (&hspi1);
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init (&hspi1);
	
	/* Select the desired mode configuration */
	if (ConfigSel == LongData_Fast)
		dwt_configure(&config_LongData_Fast);
	else if (ConfigSel == ShortData_Fast)
		dwt_configure(&config_ShortData_Fast);
	else if (ConfigSel == LongData_Range)
		dwt_configure(&config_LongData_Range);
	else
		Error_Handler();
	
	/* If the application is in the responder mode, it should sets the callbacks */
	if (responder == 1){
		dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
	}
	
	/* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
	
  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  //dwt_setpreambledetecttimeout(PRE_TIMEOUT);
  dwt_setleds(DWT_LEDS_ENABLE);

	/* If the application is in the initiator mode, set expected response's delay and timeout. See NOTE 1 and 5 below.
		* As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
	if (responder ==0){
		dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
		dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	}
	
  dwt_setsmarttxpower(0);
  configTX.power = 0x1F1F1F1F;
  configTX.PGdly = 0xC2;
  dwt_configuretxrf(&configTX);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Set a timestamp value to the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to set
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

// @brief Callback to process TX confirmation events
static void tx_conf_cb(const dwt_cb_data_t *cb_data){
}

// @brief Callback to process RX OK events
static void rx_ok_cb(const dwt_cb_data_t *cb_data){
		//printf("Received a frame\n");

		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
			 * buffer. */
		for (int i = 0 ; i < RX_BUF_LEN; i++ )
		{
			rx_buffer[i] = 0;

		}
		
		/* A frame has been received, read it into the local buffer. */
		if (cb_data->datalength <= RX_BUF_LEN)
		{
				dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
		}

		/* Check that the frame is a first poll sent by "DS TWR initiator" example or the final message.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			  t1 = HAL_GetTick();
				//printf("Received the expected frame\n");
				uint32 resp_tx_time;
				int ret;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb_responder;
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

				/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
				if (ret == DWT_ERROR)
				{
						printf("Sending Error\n");
				}
				//printf("Response sent, waiting for final reception\n");

				
				/* Increment frame sequence number after transmission of the response message (modulo 256). */
				frame_seq_nb_responder++;

				state = WAIT;
		}
		else if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
		{
				dwt_setleds(DWT_LEDS_ENABLE);
				uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
				uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
				double Ra, Rb, Da, Db;
				int64 tof_dtu;

				/* Retrieve response transmission and final reception timestamps. */
				resp_tx_ts = get_tx_timestamp_u64();
				final_rx_ts = get_rx_timestamp_u64();

				/* Get timestamps embedded in the final message. */
				final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
				final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
				final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

				/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
				poll_rx_ts_32 = (uint32)poll_rx_ts;
				resp_tx_ts_32 = (uint32)resp_tx_ts;
				final_rx_ts_32 = (uint32)final_rx_ts;
				Ra = (double)(resp_rx_ts - poll_tx_ts);
				Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
				Da = (double)(final_tx_ts - resp_rx_ts);
				Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
				tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

				tof = tof_dtu * DWT_TIME_UNITS;
				distance = tof * SPEED_OF_LIGHT;

				/* Display computed distance on LCD. */
				//printf("Dist: %1.2f\n",distance);
				
				state = PROCESS;
		}
}
 
// @brief Callback to process RX timeout events
static void rx_to_cb(const dwt_cb_data_t *cb_data){
		//printf ("Not receiving a frame and timeout. Cause:%lx\n",cb_data->status);
		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
	
		state = RECEIVE_I;
}

// @brief Callback to process RX error events
static void rx_err_cb(const dwt_cb_data_t *cb_data){
		//printf ("Not receiving a frame and error. Cause:%lx\n",cb_data->status);
		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
	
		state = RECEIVE_I;
}

/*! ---------------------------------------------------
 * @fn initiator_go()
 *
 * @brief initiator application
 *
 * @param  numMeasure number of measurements try to be done between the responder
 *
 * @return none
 */
static void initiator_go (uint16_t numMeasure)
{
	for(uint16_t numDone=0; numDone<numMeasure;numDone++){
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		t1 = TIM2->CNT;
		/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
		tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb_initiator;
		dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

		/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
		 * set by dwt_setrxaftertxdelay() has elapsed. */
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{ };
		
		/* Increment frame sequence number after transmission of the poll message (modulo 256). */
		frame_seq_nb_initiator++;
		
		//printf ("Transmition started and waited for a reception or timeout\n");
		if (status_reg & SYS_STATUS_RXFCG)
		{
				//printf ("Reception of a frame\n");
				uint32 frame_len;

				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
						dwt_readrxdata(rx_buffer, frame_len, 0);
				}
				/* Check that the frame is the expected response from the companion "DS TWR responder" example.
				 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0;

				// if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
				if (memcmp(rx_buffer, rx_resp_msg, 5) == 0)
				{
						//printf ("Reception of the expected frame, sending final msg\n");
						uint32 final_tx_time;
						int ret;

						/* Retrieve poll transmission and response reception timestamp. */
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						/* Compute final message transmission time. See NOTE 10 below. */
						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);

						/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						/* Write all timestamps in the final message. See NOTE 11 below. */
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						/* Write and send final message. See NOTE 8 below. */
						tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb_initiator;
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
						if (ret == DWT_SUCCESS)
						{
								printf ("Final msg sent correct # %d\n",numDone);
                printf("%d\n", (int)final_tx_ts);
                printf("%d\n", (int)final_tx_time);
								/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
								while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
								{ };

								/* Clear TXFRS event. */
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

								/* Increment frame sequence number after transmission of the final message (modulo 256). */
								frame_seq_nb_initiator++;
						}
				}
		}
		else
		{
				printf ("Not receiving a frame and timeout. Cause:%lx\n",status_reg);
				/* Clear RX error/timeout events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

				/* Reset RX to properly reinitialise LDE operation. */
				dwt_rxreset();
		}

		/* Execute a delay between ranging exchanges. */
		// Sleep(RNG_DELAY_MS);
		HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		while(TIM2->CNT - t1 < RNG_DELAY_MS*100);
	}
}

/* @fn      HAL_GPIO_EXTI_Callback
 * @brief   IRQ HAL call-back for all EXTI configured lines
 *          i.e. DW_RESET_Pin, DW_IRQn_Pin and Button_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DW_RESET_Pin)
    {
        set_signalResetDone();
    }
    else if (GPIO_Pin == DW_IRQn_Pin)
    {
        process_deca_irq();
    }
    else if (GPIO_Pin == Button_Pin)
    {
			if (state != INITIATOR)
				state = INITIATOR;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/