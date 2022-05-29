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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

#define CHANNEL1 1
#define CHANNEL2	2
#define CHANNEL3	3
#define CHANNEL4	4

#define PWM_DISABLE_VALUE 0
#define PWM_ENABLE_VALUE 106

#define MIN_VAL 20
#define MAX_VAL 28

#define DEF_LED2_TIMEOUT 25
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	#define DEF_BUZZER_ON_TIMEOUT 100
	#define MAX_SENSORS 12
	
	volatile unsigned char Sense,BuzzerOnTimeout;
	
	
unsigned char IRSelect;
unsigned char BlueLedTimeout,Led1Timeout,Led2Timeout,PulseDropCount,PulseDropCount1;
_Bool VendSensed,CheckIR,GreenLedOn;

volatile unsigned int MiniTicks,SmallTicks,SmallTickOver,OneSecOver;
_Bool Ms100_Over;

unsigned int SensorTimeout[MAX_SENSORS],PowerOnTimeout;
unsigned char ChannelSensed[MAX_SENSORS];
volatile unsigned char i,c;
volatile unsigned char MicroSeconds;
volatile unsigned int PulseLength[MAX_SENSORS],TempPulseLength,MinPulseLength,MaxPulseLength;
//void PWM_Config(unsigned char TIMER,unsigned char Channel,unsigned int Value);

void Generate_Burst(unsigned char bTIMER,unsigned char bChannel);
void Generate_Burst2(unsigned char bTIMER,unsigned char bChannel);

void Delay10MicroSeconds(unsigned int uSeconds);

unsigned char SenseThroughMaxMin;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(100);
	//HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_RESET);
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim17);
	HAL_TIM_Base_Start(&htim15);
	HAL_TIM_Base_Start(&htim16);
	
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	
	IRSelect=0;
	PowerOnTimeout = 500;
	MinPulseLength = 0xFFFF;
	MaxPulseLength = 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	//	IR_Interrupt=0;
		
		
		switch(IRSelect)
		{
			case 0:

				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
			break;
			
			case 1:

				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
			break;
			
			case 2:

				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
				
			break;
			
			case 3:

				HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
				
			break;
			
			case 4:

				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
				
			break;
			
			case 5:

				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
				
			break;
			
			case 6:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
				
			break;
			
			case 7:

				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
				
			break;
			
//			case 8:

//				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
//				
//			break;
//			
//			case 9:
//				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//				
//			break;
//			
//			case 10:
//				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//				
//			break;
//						
//			case 11:
//				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//				
//			break;
		}
		
		Delay10MicroSeconds(2);
		//HAL_Delay(2);
		
		switch(IRSelect)
		{
			case 0:

				HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
			break;
			
			case 1:

				HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
			break;
			
			case 2:

				HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_2);
				
			break;
			
			case 3:

				HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
				
			break;
			
			case 4:

				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
				
			break;
			
			case 5:

				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
				
			break;
			
			case 6:
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
				
			break;
			
			case 7:

				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
				
			break;
			
//			case 8:

//				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
//				
//			break;
//			
//			case 9:
//				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
//				
//			break;
//			
//			case 10:
//				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
//				
//			break;
//						
//			case 11:
//				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
//				
//			break;
		}
		
		//Delay10MicroSeconds(12);
		//HAL_Delay(2);
		
		
		IRSelect++;
		if(IRSelect >= MAX_SENSORS)
		{
			IRSelect=0;
		}
		
	
		if(SmallTickOver)
		{
			SmallTickOver=0;
			
			if(Led2Timeout > 0)
			{
				Led2Timeout--;
				if(Led2Timeout == 0)
				{
					//HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(OUT_GPIO_Port,OUT_Pin,GPIO_PIN_RESET);
				}
			}
			
			if(BuzzerOnTimeout > 0)
			{
				BuzzerOnTimeout--;
				if(BuzzerOnTimeout == 0)
				{
					HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_SET);
				}
				
			}
			
			if(PowerOnTimeout>0)
			{
				PowerOnTimeout--;
			}
			else 
			{
				//
				/*
				MinPulseLength = TempPulseLength;
					}
					
					if(TempPulseLength > MaxPulseLength)
					{
						MaxPulseLength 
				*/
				
				if(OneSecOver)
				{
					OneSecOver=0;
					
					if(MinPulseLength < MIN_VAL )
					{
						HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port,YELLOW_LED_Pin,GPIO_PIN_RESET);
						
//						if(SenseThroughMaxMin ==0)
//						{
//							HAL_GPIO_WritePin(OUT_GPIO_Port,OUT_Pin,GPIO_PIN_SET);
//							Led2Timeout=DEF_LED2_TIMEOUT;
//							SenseThroughMaxMin=1;
//						}
					}
					else if(MaxPulseLength > MAX_VAL)
					{
						HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port,YELLOW_LED_Pin,GPIO_PIN_RESET);
//						if(SenseThroughMaxMin ==0)
//						{
//							HAL_GPIO_WritePin(OUT_GPIO_Port,OUT_Pin,GPIO_PIN_SET);
//							Led2Timeout=DEF_LED2_TIMEOUT;
//							SenseThroughMaxMin=1;
//						}
					}
					else 
					{
						HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port,YELLOW_LED_Pin,GPIO_PIN_SET);
						SenseThroughMaxMin=0;
					}
					
					MinPulseLength = 0xFFFF;
					MaxPulseLength = 0;
				}
				
			}
		}
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 571;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 285;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 571;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 285;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 571;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 285;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 571;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 285;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 571;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 285;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_GPIO_Port, OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, RED_LED_Pin|YELLOW_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12
                           PB13 PB3 PB4 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_Pin */
  GPIO_InitStruct.Pin = OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|YELLOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void ActChannelInterrupt(unsigned char channelNum)
{
	if( (SensorTimeout[channelNum] > MAX_VAL) || (SensorTimeout[channelNum] < MIN_VAL) )
	{
		//HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(OUT_GPIO_Port,OUT_Pin,GPIO_PIN_SET);
		Led2Timeout=DEF_LED2_TIMEOUT;
		
		Sense=1;
	}
	TempPulseLength = PulseLength[channelNum];
	PulseLength[channelNum] = SensorTimeout[channelNum];
	SensorTimeout[channelNum]=0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Sense=0;
	// IR_Interrupt
	if(GPIO_Pin == GPIO_PIN_0)
	{
		ActChannelInterrupt(0);
	}
	
	else if(GPIO_Pin == GPIO_PIN_1)
	{
		ActChannelInterrupt(1);
	}
	
	else if(GPIO_Pin == GPIO_PIN_2)
	{
		ActChannelInterrupt(2);
	}
	
	else if(GPIO_Pin == GPIO_PIN_3)
	{
		ActChannelInterrupt(3);
	}
	
	else if(GPIO_Pin == GPIO_PIN_4)
	{
		ActChannelInterrupt(4);
	}
	
	else if(GPIO_Pin == GPIO_PIN_5)
	{
		ActChannelInterrupt(5);
	}
	
	else if(GPIO_Pin == GPIO_PIN_6)
	{
		ActChannelInterrupt(6);
	}
	
	else if(GPIO_Pin == GPIO_PIN_7)
	{	
		ActChannelInterrupt(7);
	}
	
//	if(GPIO_Pin == GPIO_PIN_10)
//	{
//		ActChannelInterrupt(8);
//	}
//	
//	if(GPIO_Pin == GPIO_PIN_11)
//	{
//		ActChannelInterrupt(9);
//	}
//	else if(GPIO_Pin == GPIO_PIN_12)
//	{
//		ActChannelInterrupt(10);
//	}
//	else if(GPIO_Pin == GPIO_PIN_13)
//	{
//		ActChannelInterrupt(11);
//	}
//	
	
	if(Sense)
	{

		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
		BuzzerOnTimeout = DEF_BUZZER_ON_TIMEOUT;
		

		
	}
	
	if(PowerOnTimeout == 0)
	{
		if(TempPulseLength < MinPulseLength)
		{
			MinPulseLength = TempPulseLength;
		}
		
		if(TempPulseLength > MaxPulseLength)
		{
			MaxPulseLength = TempPulseLength;
		}
	}
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		// 10uS interrupt
		MiniTicks++;
		if(MicroSeconds > 0)
			MicroSeconds--;

		for(i=0;i<MAX_SENSORS;i++)
		{
			if(SensorTimeout[i] < 0xFFFF)
			{
				SensorTimeout[i]++;
			}
		}
		
		if(MiniTicks >= 20)
		{
			MiniTicks=0; // 1Ms Over
			SmallTicks++;
			
			SmallTickOver=1;
			
			if( (SmallTicks % 100) == 0)
			{
				Ms100_Over=1;
			}
			
			
			if(SmallTicks >= 1000)
			{
				SmallTicks=0;
				OneSecOver=1;
				//HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port,YELLOW_LED_Pin);
			}
		}
	}
}

void Delay10MicroSeconds(unsigned int uSeconds)
{
	MicroSeconds=uSeconds;
	while(MicroSeconds > 0);
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
