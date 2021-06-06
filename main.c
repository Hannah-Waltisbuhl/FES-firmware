/* USER CODE BEGIN Header */
/**
  *********************************************************************
  * @file           : main.c
  * @brief          : Stimulator Program 
  *********************************************************************
  */
/* USER CODE END Header */
/* Includes ---------------------------------------------------------*/
#include "main.h"

/* Private includes -------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef --------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ---------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro ----------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
const uint16_t SW_ALL = 0xFFFF;
const uint16_t SW_OFF = 0x0000;
const uint16_t SW_EVEN = 0xAAAA;
const uint16_t SW_ODD = 0x5555;


// STIMULATION SIGNAL VARIABLES:
const bool HOLD = false;				// true will hold amplitude (no flip)
uint16_t pulseWidth = 0.5 * 1000;		// in us (>= 1ms)
float amplitude = 13; 					// stimulation current in mA (0.5mA to 30mA)
uint16_t nodeA = 1;						// valid 1-8
uint16_t nodeB = 8;						// valid 1-8
uint32_t burst = 50 * 1000; 			// burst duration in us (e.g. 50us)
uint32_t gap = 0 * 1000; 				// inter-burst gap in us (e.g. 50us)

// COUNTERS:
const uint16_t TIMER_MIN = 10; 	// us
uint16_t countTick = 0; 		// counts every 10us
uint16_t countPulse = 0;		// counts every pulse
uint16_t countGap = 0;			// counts every pulse during a gap

/* USER CODE END PV */

/* Private function prototypes --------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Turns on the H-Bridge
  * @retval void
  */
void bridgeOn(void) {

	HAL_GPIO_WritePin(GPIOB, ALI_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BHI_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, AHI_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BLI_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, BRIDGE_EN_Pin, GPIO_PIN_SET);

}

/**
  * @brief  Turns off the H-Bridge
  * @retval void
  */
void bridgeOff(void) {

	HAL_GPIO_WritePin(GPIOB, ALI_Pin|AHI_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BLI_Pin|BHI_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, BRIDGE_EN_Pin, GPIO_PIN_RESET);

}

/**
  * @brief  Set an output at the DAC
  * @param	valueDAC - the value to pass to the DAC
  * @retval void
  */
void setDAC(uint32_t valueDAC) {

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, valueDAC);

}

/**
  * @brief  Generates a square wave of stimulation
  * @param	currentAmp - the desired stimulation amplitude in mA
  * 		pulseWidth - the desired stimulation pulse width in ms
  * @retval void
  */
void squareWave(float currentAmp, uint32_t pulseWidth) {
	uint32_t valueDAC;
	float voltageAmp = 0;
	int resistor = 100;

	voltageAmp = currentAmp/1000 * resistor;

	valueDAC = voltageAmp*(0xFFF+1)/3.3;
	setDAC(valueDAC);

	// flip the H-Bridge:
	HAL_GPIO_TogglePin(GPIOB, ALI_Pin|AHI_Pin);
	HAL_GPIO_TogglePin(GPIOA, BLI_Pin|BHI_Pin);

	// flash the LED:
	HAL_GPIO_TogglePin(GPIOC, LED_Pin);

	// delay:
	HAL_Delay(pulseWidth);

}

/**
  * @brief	Flips the H-Bridge to generate rectangular pulse
  * @param	currentAmp - the desired stimulation amplitude in mA
  * 		pulseWidth - the desired stimulation pulse width in ms
  *
  * @retval void
  */
uint32_t pulse(uint32_t pulseWidth, uint32_t previousTime) {
	uint32_t currentTime = 0;

	// Timing Stuff
	currentTime = HAL_GetTick();

	if ((currentTime - previousTime) >= pulseWidth) {
		previousTime = currentTime;

		// flip the H-Bridge:
		HAL_GPIO_TogglePin(GPIOB, ALI_Pin|AHI_Pin);
		HAL_GPIO_TogglePin(GPIOA, BLI_Pin|BHI_Pin);

		// flash the LED:
		HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	}

	return previousTime;
}

/**
  * @brief	Sets the amplitude of the stimulation current
  * @param	currentAmp - the desired stimulation amplitude in mA
  *
  * @retval void
  */
void setAmplitude(float currentAmp) {
	uint32_t valueDAC;
	float voltageAmp = 0;
	int resistor = 100;

	voltageAmp = currentAmp/1000 * resistor;

	valueDAC = voltageAmp*(0xFFF+1)/3.3;
	setDAC(valueDAC);
}

/**
  * @brief  Sets switch clear pin high, turns all switches off
  * @retval void
  */
void switchClear(void) {

	HAL_GPIO_WritePin(GPIOC, SW_CLR_Pin, GPIO_PIN_SET);

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&SW_OFF, 1, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
}

/**
  * @brief  Pulls switch clear pin low to enable switching
  * @retval void
  */
void switchEnable(void) {

	HAL_GPIO_WritePin(GPIOC, SW_CLR_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Sets the nodes for stimulation by connecting to the output of the H-bridge,
  * 		nodeA will be connected to H+, nodeB connected to H-.
  * @param	int nodeA (valid 1-8)
  * 		int nodeB (valid 1-8)
  * @retval void
  */
void connectNodes(uint16_t nodeA, uint16_t nodeB) {
	bool valid = false;
	uint16_t switchState = 0;

	// check if the chosen nodes are valid
	if (nodeA > 8 || nodeB > 8) { // not valid if node > 8
		valid = false;
	} else if (nodeA < 1 || nodeB < 1) { // not if node < 1
		valid = false;
	} else if (nodeA == nodeB) { // not valid if the same node
		valid = false;
	} else {
		valid = true;
	}

	// close the corresponding switches
	if (valid) {
		switchState = 0 | (1 << (nodeA-1)) | (1 << (nodeB+7));

		// send SPI command
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&switchState, 1, HAL_MAX_DELAY);
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	}

	if (!valid) {
		switchClear();
	}
}

void allSwitchesOn (void) {

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&SW_ALL, 1, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration-----------------------------------------------*/

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
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); // connect dac output to pin PA5
  HAL_TIM_Base_Start_IT(&htim16);		// run timer in interrupt mode

  bridgeOn(); 							// enable the h-bridge driver
  switchEnable(); 						// set switch clear pin low
  connectNodes(nodeA,nodeB); 			// connect stimulation nodes
  setAmplitude(amplitude);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SW_LE_Pin|SW_CLR_Pin|LED_Pin|BRIDGE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AHI_Pin|ALI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLI_Pin|BHI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW_LE_Pin SW_CLR_Pin LED_Pin BRIDGE_EN_Pin */
  GPIO_InitStruct.Pin = SW_LE_Pin|SW_CLR_Pin|LED_Pin|BRIDGE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AHI_Pin ALI_Pin */
  GPIO_InitStruct.Pin = AHI_Pin|ALI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BLI_Pin BHI_Pin */
  GPIO_InitStruct.Pin = BLI_Pin|BHI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	// check which version of the timer triggered this callback
	if ((htim == &htim16) && (HOLD == false)) {

		// count the tick
		countTick++;

		if (countTick >= (pulseWidth/TIMER_MIN)) {
			// count the pulse
			countPulse++;

			// gap?
			if (((countPulse)*pulseWidth >= burst) && gap!=0) {
				countGap++;

				setAmplitude(0);
				bridgeOff();

				if ((countGap)*pulseWidth >= gap) {
					setAmplitude(amplitude);
					bridgeOn();
					countGap = 0;
					countPulse = 0;
				}

			}

			else {
				// flip the H-Bridge:
				HAL_GPIO_TogglePin(GPIOB, ALI_Pin|AHI_Pin);
				HAL_GPIO_TogglePin(GPIOA, BLI_Pin|BHI_Pin);

				HAL_GPIO_TogglePin(GPIOC, LED_Pin);
			}

			countTick = 0;
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
