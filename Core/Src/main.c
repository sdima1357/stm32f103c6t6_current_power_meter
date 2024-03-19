/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "st7789.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void retarget_put_char(char p)
{
	HAL_UART_Transmit(&huart1, &p, 1, 0xffffff); // send message via UART
}
void crput(char r)
{
	retarget_put_char(r);
}
int _write(int fd, char* ptr, int len)
{
    (void)fd;
    int i = 0;
    while (ptr[i] && (i < len))
    {
    	if (ptr[i] == '\r')
    	{

    	}
    	else
    	{
    		crput(ptr[i]);
			//retarget_put_char((int)ptr[i]);
			if (ptr[i] == '\n')
			{
				crput('\r');
				//retarget_put_char((int)'\r');
			}
    	}
        i++;
    }
    return len;
}
#define HSAMPLES 64
#define NCHAN 6
struct sADC_BUFF
{
	uint16_t V_1; // 1.0/2.0
	uint16_t V_2; // 1.0/2.0
	uint16_t V_3; // 1.0/2.0
	uint16_t V_4; // 1.0/2.0
	uint16_t V_TEMP;
	uint16_t V_REF;
};


uint16_t ADC_BUFF[HSAMPLES*NCHAN];

struct sADC_BUFFAcc
{
	int32_t V_1; // 1.0/2.0
	int32_t V_2; // 1.0/2.0
	int32_t V_3; // 1.0/2.0
	int32_t V_4; // 1.0/2.0
	int32_t V_TEMP;
	int32_t V_REF;
	int32_t counter;
}Acc;

#define VDD_APPLI                      ((uint32_t) 3300)   /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t) 4095)   /* Max value with a full range of 12 bits */


/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */
/* For more accurate values, device should be calibrated on offset and slope  */
/* for application temperature range.                                         */
//#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1430)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
//#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */                                                               /* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1405)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
//	#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1470)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */                                                               /* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

volatile int cb_count;
#define QUE_BITS 7
#define QUE_SIZE (1<<QUE_BITS)

struct results
{
	float voltage;
	float mA1;
	float mA2;
	float temperature;
} Queue[QUE_SIZE];

uint8_t iQueueHead=0;
uint8_t iQueueTail=0;

int qSize()
{
	return (iQueueHead-iQueueTail)&(QUE_SIZE-1);
}
void calcSumm(struct sADC_BUFFAcc* accs,struct sADC_BUFF* ADC_BUFFp)
{
	accs->V_REF = 0;
	accs->V_1 = 0;
	accs->V_2 = 0;
	accs->V_3 = 0;
	accs->V_4 = 0;
	accs->V_TEMP = 0;
	for(int k=0;k<HSAMPLES/2;k++)
	{
		accs->V_REF +=  ADC_BUFFp[k].V_REF;
		accs->V_1+=ADC_BUFFp[k].V_1;
		accs->V_2+=ADC_BUFFp[k].V_2;
		accs->V_3+=ADC_BUFFp[k].V_3;
		accs->V_4+=ADC_BUFFp[k].V_4;
		accs->V_TEMP+=ADC_BUFFp[k].V_TEMP;
	}
	accs->V_REF/=HSAMPLES/2;
	accs->V_1/=HSAMPLES/2;
	accs->V_2/=HSAMPLES/2;
	accs->V_3/=HSAMPLES/2;
	accs->V_4/=HSAMPLES/2;
	accs->V_TEMP/=HSAMPLES/2;

#define R4  472000.0f
#define R5  470000.0f

#define R6  477000.0f
#define R7  470000.0f

#define R8  475000.0f
#define R9  470000.0f

#define R1  9.7f
#define R2  470.0f

#define MA1_ZERO_CORR 	-0.15f
#define MA2_ZERO_CORR 	+0.0028f

#define VV12	1.2f
	//
	float v1			= ((R4+ R5)/R5)*accs->V_1*VV12/accs->V_REF;
	float v2			= ((R6+ R7)/R7)*accs->V_2*VV12/accs->V_REF;
	float v3			= ((R8+ R9)/R9)*accs->V_3*VV12/accs->V_REF;
	float temperature	= (INTERNAL_TEMPSENSOR_V25-accs->V_TEMP*VV12*1000.0f/accs->V_REF)*1000/INTERNAL_TEMPSENSOR_AVGSLOPE+25.0f;
	Queue[iQueueHead].voltage = v3;
	Queue[iQueueHead].mA1 = 1000*(v1-v2)/R1-MA1_ZERO_CORR;
	Queue[iQueueHead].mA2 = 1000*(v2-v3)/R2+MA2_ZERO_CORR;
	Queue[iQueueHead].temperature = temperature;
	iQueueHead = (iQueueHead +1)&(QUE_SIZE-1);
	//v1,1000*(v1-v2)/R1-0.9-3.6,1000*(v2-v3)/R2+0.027;

    //printf("%f %fmA %fmA temp=%d.%d %d.%d %d.%03d %x %x %x %x\n",v1,1000*(v1-v2)/10.0f-0.9-3.6,1000*(v2-v3)/470.0f+0.027,temp/10,temp%10,itemp/10,itemp%10,ivoltage/1000,ivoltage%1000,V_REF,ADC_BUFFp[0].V_REF,ADC_BUFFp[0].V_1,ADC_BUFFp[0].V_2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
	struct sADC_BUFFAcc* accs = &Acc;
	struct sADC_BUFF* ADC_BUFFp = &ADC_BUFF[HSAMPLES*NCHAN/2];

	calcSumm(accs,ADC_BUFFp);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	cb_count++;
  //ubSequenceCompleted = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	struct sADC_BUFFAcc* accs = &Acc;
	struct sADC_BUFF* ADC_BUFFp = &ADC_BUFF[0];
	calcSumm(accs,ADC_BUFFp);
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
 // printf("Program_ start!\n");
  int i,ret;
 // Queue = malloc(QUE_SIZE*sizeof(struct results));
	ST7789_Init();
//printf("init ok!\n");
	ST7789_Fill_Color(GREEN);
	HAL_Delay(250);
	ST7789_Fill_Color(BLACK);
	//ST7789_Fill_Color(RED);
	int ADCOK = 0;
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
		/* Calibration Error */
		printf("HAL_ADCEx_Calibration error\r\n");
		ADCOK = 0;
	}
	else
	{
		printf("HAL_ADCEx_Calibration OK\r\n");
		ADCOK = 1;
	}
#if 0
	  for(i=1; i<128; i++)
	      {
	          ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
	          if (ret != HAL_OK) /* No ACK Received At That Address */
	          {
	              printf(" - ");
	          }
	          else if(ret == HAL_OK)
	          {
	              printf("0x%X", i*2);
	          }
	      }
	  	  printf("\nDone! \n\n");
#endif
#if 0
	  	if(!LM75_Init(100000))
	  			{
	  			// printf("LM35 init error\n");
	  			}
	  	else
	  		{
	  		/*
	  		uint16_t value;

	  	    value = LM75_ReadReg(0x00);
	  	    printf("V=%x\n",value);
	  		value = LM75_ReadConf();
	  		printf("V=%x\n",value);
	  		value = LM75_ReadReg(0x02);
	  		printf("V=%x\n",value);
	  		value = LM75_ReadReg(0x03);
	  		printf("V=%x\n",value);
*/
	  	    LM75_Shutdown(DISABLE);

	  		}
#endif
  //printf("Program_start on %d %d!\n" ,HAL_RCC_GetSysClockFreq(),HAL_RCC_GetHCLKFreq());

		 	//ST7789_Test();
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BUFF, HSAMPLES*NCHAN);
		uint32_t lastTime = HAL_GetTick();
		uint32_t firstTime = HAL_GetTick();
		float mASeclob = 0;
		float mJ = 0;
		int flagClearDisp1 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //read all
	  int iSize = qSize();
	  float voltage = 0;
	  float mA1 = 0;
	  float mA2 = 0;
	  float mASec = 0;
	  float temperature = 0;
	  float watt = 0;
	  for(int k=0;k<iSize;k++)
	  {
		  voltage+=Queue[iQueueTail].voltage;
		  mA1+=Queue[iQueueTail].mA1;
		  mA2+=Queue[iQueueTail].mA2;
		  //TBD make LS approximation with weights between 300-1000uA instead of this dummy threshold !!!

		  float mA = Queue[iQueueTail].mA2>0.3f?Queue[iQueueTail].mA1:Queue[iQueueTail].mA2;
//#if DEBUG
		  //float mA = Queue[iQueueTail].mA2;
		  //float mA = Queue[iQueueTail].mA2*1000;
//#endif
		  watt +=Queue[iQueueTail].voltage*mA;
 		  mASec +=mA;
		  temperature+=Queue[iQueueTail].temperature;
		  iQueueTail = (iQueueTail+1)&(QUE_SIZE-1);
	  }
	  float scale = 1.0f;
	  if(iSize)
	  {
		  scale=1.0f/iSize;
	  }
	  voltage*= scale;
	  mA1*= scale;
	  mA2*= scale;
	  temperature*= scale;
	  mASec*= scale;
	  watt *= scale;
	 // float 	temp = LM75_Temperature()*0.1f;
	  printf("%1.3fV %3.3fmA %0.4fmA %2.1f %d\n",voltage,mA1,mA2,temperature,iSize);
	  //ST7789_Fill_Color(BLACK);
	  float ma = mASec;
	  //float ma = mA1;
	  char srt[0x10];
	  int32_t nlastTime = HAL_GetTick();
	  int32_t delta_mS = nlastTime-lastTime;
	  lastTime = nlastTime;

	  mASec*=delta_mS/1000.0f;

	  mJ += watt*delta_mS/1000.0f;

	  mASeclob+=mASec;
#define FON_CURR  Font_7x10
#define WRITE_STRING	 ST7789_WriteString3
	  float integratedTime = (nlastTime-firstTime)/1000.0f;

	  sprintf(srt,"%3.3fmA  ",ma);
	  WRITE_STRING(10, 10, srt, FON_CURR, YELLOW, BLACK);

	  sprintf(srt,"%3.3fmA  ",mASeclob/integratedTime);
	  WRITE_STRING(10, 10+12*3, srt, FON_CURR, BLUE, BLACK);

	  sprintf(srt,"%1.3fV  ",voltage);
	  WRITE_STRING(10, 10+12*6, srt, FON_CURR, GREEN, BLACK);

	  sprintf(srt,"IT: %ds    ",(int)integratedTime);
	  WRITE_STRING(10, 10+12*9, srt, FON_CURR, RED, BLACK);

	  if(mASeclob<1800.0f)
	  {
		  sprintf(srt,"%4.2fmAs  ",mASeclob);
		  WRITE_STRING(10, 10+12*12, srt, FON_CURR, CYAN, BLACK);
		  flagClearDisp1 = 1;
	  }
	  else
	  {
		  if(flagClearDisp1)
		  {
			  ST7789_Fill_Color(BLACK);
			  flagClearDisp1 = 0;
		  }
		  sprintf(srt,"%2.2fmAh  ",mASeclob/3600.0f);
		  WRITE_STRING(10, 10+12*12, srt, FON_CURR, CYAN, BLACK);
	  }
	  if(mJ<1000.0f)
	  {
		  sprintf(srt,"%2.2fmJ  ",mJ);
		  WRITE_STRING(10, 10+12*15, srt, FON_CURR, RED, BLACK);
	  }
	  else
	  {
		  sprintf(srt,"%2.2fJ    ",mJ/1000.0f);
		  WRITE_STRING(10, 10+12*15, srt, FON_CURR, RED, BLACK);

	  }



	  HAL_Delay(200);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RES_Pin|LCD_CS_Pin|LCD_DC_Pin|LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RES_Pin LCD_CS_Pin LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_RES_Pin|LCD_CS_Pin|LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
