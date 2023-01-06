/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "stm32f1xx_hal_tim.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GPIO_IRQ_		  GPIOC
#define GPIO_IRQ_PIN  GPIO_PIN_15
#define GPIO_DA			  GPIOC
#define GPIO_DA_PIN   GPIO_PIN_14
#define GPIO_CK			  GPIOC
#define GPIO_CK_PIN	  GPIO_PIN_13
#define GPIO_LD			  GPIOA
#define GPIO_LD_PIN	  GPIO_PIN_12

#define EXTI_Keyboard_PinSource EXTI_PinSource15
#define EXTI_Keyboard_Line EXTI_Line15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx;
int w=0;
int speed=0;

uint32_t TIM1_CH3_VAL[5];//0:last;1:current;2:delta;4:COUNTOUT;5:CNT
uint32_t TIM1_CH4_VAL[5];//0:last;1:current;2:delta;4:COUNTOUT;5:CNT
volatile float SPEED3[10];
volatile float SPEED4[10];
volatile float SPEED_3=0;
volatile float SPEED_4=0;
volatile int setspeed3=200;
volatile int setspeed4=200;
const float D=0.065f;
const float PI=3.14159265f;
struct{
	float x_theta;
} car;

typedef struct 
{
	volatile float target;
	volatile float kp;
	volatile float ki;
	volatile float kd;
	volatile float P;
	volatile float I;
	volatile float D;
	volatile float last_e;
	volatile float current_e;
}P_I_D;

P_I_D PID_Theta;
float theta=0;
volatile float speed_delta=0;

uint8_t rx_buf1[24];  //  for saving RX Data
uint8_t rx_buf2[24];
uint8_t rx_buf3[24];
char usart1rx[24];
char usart2rx[24];
char usart3rx[24];

char TIM_4_Cnt=0;;

float targeta=450;

char stop_cnt=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int fputc(int ch,FILE *f);
void Wheel(int w,int speed);
void PID_Init(P_I_D *pid);
float PID_DO(P_I_D *pid,float actualvalue);
float PID_Do(P_I_D *pid,float actualvalue);
float PID_DA(P_I_D *pid,float actualvalue);
void PID_Set_Para(P_I_D *pid,int para,float value);
int Getpix(char* str);
uint16_t Keyboard_ReadData(void);
void KeyAction(uint8_t key);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
	HAL_TIM_Base_Start_IT(&htim1);//Timer turn on(input capture)
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
	HAL_UART_Receive_IT(&huart1,(uint8_t*)rx_buf1, 1);
	HAL_UART_Receive_IT(&huart3,(uint8_t*)rx_buf3, 1);
	HAL_TIM_Base_Start_IT(&htim4);//Timer on
	P_I_D PIDV3,PIDV4;
	PID_Init(&PIDV3);
	PID_Init(&PIDV4);
	PIDV3.target=0.5;
	PIDV4.target=0.5;
	Wheel(3,setspeed3);
	Wheel(4,setspeed4);
	PID_Set_Para(&PIDV3,0,-40);
	PID_Set_Para(&PIDV4,0,-40);
	PID_Set_Para(&PIDV3,1,-0.02);
	PID_Set_Para(&PIDV4,1,-0.02);
	PID_Set_Para(&PIDV3,2,-400);
	PID_Set_Para(&PIDV4,2,-2000);	
	PID_Init(&PID_Theta);
	PID_Theta.target=0;
	PID_Set_Para(&PID_Theta,0,0.011);//0.01
	PID_Set_Para(&PID_Theta,1,0.00005);//0.00005
	PID_Set_Para(&PID_Theta,2,0.015);//0.013

	HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//TIM_4_Cnt
		if(TIM_4_Cnt){
			static int t=0;
			if(stop_cnt==2){
				setspeed3=-40;
				setspeed4=5;
				Wheel(3,setspeed3);
				Wheel(4,setspeed4);
				HAL_Delay(5000);
				setspeed3 = 200;
				setspeed4 = 200;
				stop_cnt=0;
			}
			if(t>=5){
//				theta=Getpix(usart3rx);
				speed_delta+=PID_DA(&PID_Theta,theta);
				if(speed_delta>0.25){
					speed_delta=0.25;
				}
				if(speed_delta<-0.25){
					speed_delta=-0.25;
				}
				PIDV3.target=(targeta/1000)+speed_delta;
				PIDV4.target=(targeta/1000)-speed_delta;
				t=0;
			}
			setspeed3+=PID_DO(&PIDV3,SPEED_3);
			setspeed4+=PID_Do(&PIDV4,SPEED_4);
			if(setspeed4<0){
				setspeed4=50;
			}
			if(1){
				
			}
			Wheel(3,setspeed3);
			Wheel(4,setspeed4);
			TIM_4_Cnt=0;
			t++;
			//printf("%f,%f\r\n",SPEED_3*1000,SPEED_4*1000);
			if(usart3rx[0]=='s'){
				usart3rx[0]=0;
				stop_cnt=1;
			}
			if(usart3rx[0]=='w'){
				usart3rx[0]=0;
				stop_cnt=2;
			}
			if(stop_cnt==1){
				Wheel(3,-40);
				Wheel(4,5);
				while(1){
					
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function is for USART
  * @retval int
  */

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit (&huart1 ,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
}
/**
  * @brief  This function is used to set the speed of the wheel
	* @param	w: whick wheel to run
	* @param	speed: speed of the wheel
  * @retval int
  */
void Wheel(int w,int speed)
{
	if(speed>750)
	{
		speed=750;
	}
	if(speed<-1000)
	{
		speed=-1000;
	}
	if(w==3){
		if(speed>0)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,1000-speed);
		}else if(speed<0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,-speed);
		}else if(speed==0){
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,1000);
		}
	}else if(w==4){
		if(speed>0){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,speed);
		}else if(speed<0){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,1000+speed);
		}else if(speed==0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
		}
	}
}

/**
  * @brief  TIMER 
  * @retval None

  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			TIM1_CH3_VAL[3]++;
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
			TIM1_CH4_VAL[3]++;
		}
	}else if(htim->Instance == TIM4){
		static int i=0;
		i++;
		if(i>=10){
			TIM_4_Cnt=1;
			i=0;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if(stop_cnt==2){
			return;
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			if(TIM1_CH3_VAL[4]==0){
				TIM1_CH3_VAL[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
				TIM1_CH3_VAL[4]=1;
			}else if(TIM1_CH3_VAL[4]==1){
				TIM1_CH3_VAL[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
				TIM1_CH3_VAL[2] = TIM1_CH3_VAL[3]*0xffff+TIM1_CH3_VAL[1] - TIM1_CH3_VAL[0];
				TIM1_CH3_VAL[4]=0;
				TIM1_CH3_VAL[3]=0;
				float tt=0;
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET){
					int temp = TIM1_CH3_VAL[2];
					tt=PI*D*1000000/390/temp;
				}else{
					int temp = TIM1_CH3_VAL[2];
					tt=-PI*D*1000000/390/temp;
				}
				SPEED_3=0;
				for(int i=0;i<9;i++){
					SPEED_3+=SPEED3[i];
					SPEED3[i]=SPEED3[i+1];
				}
				SPEED_3+=tt;
				SPEED3[9]=tt;
				SPEED_3/=10;
			}
			
		}
		
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
			if(TIM1_CH4_VAL[4]==0){
				TIM1_CH4_VAL[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
				TIM1_CH4_VAL[4]=1;
			}else if(TIM1_CH4_VAL[4]==1){
				TIM1_CH4_VAL[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
				TIM1_CH4_VAL[2] = TIM1_CH4_VAL[3]*0xffff+TIM1_CH4_VAL[1] - TIM1_CH4_VAL[0];
				TIM1_CH4_VAL[4]=0;
				TIM1_CH4_VAL[3]=0;
				float tt=0;
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET){
					int temp = TIM1_CH4_VAL[2];
					tt=-PI*D*1000000/390/temp;
				}else{
					int temp = TIM1_CH4_VAL[2];
					tt=PI*D*1000000/390/temp;
					
				}
				SPEED_4=0;
				for(int i=0;i<9;i++){
					SPEED_4+=SPEED4[i];
					SPEED4[i]=SPEED4[i+1];
				}
				SPEED_4+=tt;
				SPEED4[9]=tt;
				SPEED_4/=10;
			}
		}
		
	}
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
}
/**
  * @brief  This function is used for USART receive
  * @retval None
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart1)
//	{
//		DataGet1(rx_buf1[0]);
//		HAL_UART_AbortReceive_IT(&huart1);
//		HAL_UART_Receive_IT(&huart1, rx_buf1, 1);
//	}
//	if(huart == &huart2)
//	{
//		DataGet2(rx_buf2[0]);
//		HAL_UART_AbortReceive_IT(&huart2);
//		HAL_UART_Receive_IT(&huart2, rx_buf2, 1);
//	}
//	if(huart == &huart3)
//	{
//		DataGet3(rx_buf3[0]);
//		HAL_UART_AbortReceive_IT(&huart3);
//		HAL_UART_Receive_IT(&huart3, rx_buf3, 1);
//	}
//}


void DataGet1(uint8_t ch){
	static int i=0;
	if((ch==0x0D)||(ch==0x0A)){
		i=0;
		targeta=Getpix(usart1rx);
		return;
	}
	usart1rx[i]=ch;
	i++;
}
void DataGet3(uint8_t ch){
	static int i=0;
	if(ch==0x0D||ch==0x0A){
		i=0;
		theta=Getpix(usart3rx);
		return;
	}	
	usart3rx[i]=ch;
	i++;
}
void DataGet2(uint8_t ch){
	static int i=0;
	if(ch==0x0D||ch==0x0A){
		i=0;
		return;
	}	
	usart2rx[i]=ch;
	i++;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		DataGet1(rx_buf1[0]);
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_Receive_IT(&huart1,rx_buf1, 1);
	}else if(huart == &huart2){
		DataGet2(rx_buf2[0]);
		HAL_UART_AbortReceive_IT(&huart2);
		HAL_UART_Receive_IT(&huart3,rx_buf2, 1);
	}else if(huart == &huart3)
	{
		DataGet3(rx_buf3[0]);
		HAL_UART_AbortReceive_IT(&huart3);
		HAL_UART_Receive_IT(&huart3,rx_buf3, 1);
	}
}
/**
  * @brief  Initialize PID structure
	* @param	*pid :the pid which need to be initialize
  * @retval None
  */
void PID_Init(P_I_D *pid){
	(*pid).target=0;
	(*pid).kp=0;
	(*pid).ki=0;
	(*pid).kd=0;
	(*pid).P=0;
	(*pid).I=0;
	(*pid).D=0;
	(*pid).last_e=0;
	(*pid).current_e=0;
}
/**
  * @brief  do pid
	* @param	*pid : pid which need to be done
	* @param	actual_e : actual err input
  * @retval output for controller
  */
float PID_DO(P_I_D *pid,float actualvalue){
	static float last;
	if(actualvalue>1){
		actualvalue = last;
	}
	last=actualvalue;
	float res=0;
	(*pid).current_e=actualvalue-(*pid).target;
	(*pid).P=(*pid).kp*(*pid).current_e;
	(*pid).I+=(*pid).current_e;
	(*pid).D=(*pid).kd*((*pid).current_e-(*pid).last_e);
	(*pid).last_e=(*pid).current_e;
	
	if((*pid).I>10){
		(*pid).I=10;
	}
	if((*pid).I<-10){
		(*pid).I=-10;
	}
	
	res=(*pid).P+(*pid).ki*(*pid).I+(*pid).D;
	if(res>1000){
		res=1000;
	}else if(res<-1000){
		res=-1000;
	}
	return res;
}

float PID_Do(P_I_D *pid,float actualvalue){
	static float last;
	if(actualvalue>1){
		actualvalue = last;
	}
	if((actualvalue-last>0.1)||(last-actualvalue>0.1)){
		actualvalue = last;
	}
	last=actualvalue;
	float res=0;
	(*pid).current_e=actualvalue-(*pid).target;
	(*pid).P=(*pid).kp*(*pid).current_e;
	(*pid).I+=(*pid).current_e;
	(*pid).D=(*pid).kd*((*pid).current_e-(*pid).last_e);
	(*pid).last_e=(*pid).current_e;
	if((*pid).I>10){
		(*pid).I=10;
	}
	if((*pid).I<-10){
		(*pid).I=-10;
	}
	res=(*pid).P+(*pid).ki*(*pid).I+(*pid).D;
	if(res>1000){
		res=1000;
	}else if(res<-1000){
		res=-1000;
	}
	return res;
}

float PID_DA(P_I_D *pid,float actualvalue){
	float res=0;
	if(actualvalue>360){
		actualvalue = (*pid).current_e;
	}else{
		actualvalue=actualvalue-(*pid).target;
	}
	(*pid).P=(*pid).kp*(actualvalue-(*pid).current_e);
	(*pid).I=actualvalue;
	(*pid).D=(*pid).kd*(actualvalue-2*(*pid).current_e+(*pid).last_e);
	(*pid).last_e=(*pid).current_e;
	(*pid).current_e=actualvalue;
	if((*pid).I>50){
		(*pid).I=50;
	}
	if((*pid).I<-50){
		(*pid).I=-50;
	}
	
	res=(*pid).P+(*pid).ki*(*pid).I+(*pid).D;
	if(res>1000){
		res=1000;
	}else if(res<-1000){
		res=-1000;
	}
	return res;
}

/**
  * @brief  settings for pid
	* @param	*pid : pid which need to be set
	* @param	para : 1-kp,2-ki,3-kd,4-target
	* @param	value : new value
  * @retval none
  */
void PID_Set_Para(P_I_D *pid,int para,float value){
	switch (para){
		case 0:
			(*pid).kp=value;
			break;
		case 1:
			(*pid).ki=value;
			break;
		case 2:
			(*pid).kd=value;
			break;
		case 3:
			(*pid).target=value;
			break;
	}
}

int Getpix(char* str){
	int i =0;
	int res =0;
	int flag =1;
	if(str[i]=='#'){
		i++;
		if(str[i]=='-'){
			i++;
			flag=-1;
		}
		while(str[i]!='#'){
			res*=10;
			res+=str[i]-48;
			i++;
		}
	}
	return res*flag;
}

void delay_us(uint32_t tim)//lowpercision
{
	uint32_t t,num=0;
	for(num=0;num<tim;num++)
	{
		t=11;
		while(t!=0)
		{
			t--;
		}
	}
}


uint16_t Keyboard_ReadData(void)
{
	uint8_t i,data;
	HAL_GPIO_WritePin(GPIO_LD, GPIO_LD_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(GPIO_LD, GPIO_LD_PIN,GPIO_PIN_RESET);
	for(i=0;i<16;i++)
	{
		delay_us(1);
		if(HAL_GPIO_ReadPin(GPIO_DA, GPIO_DA_PIN))
			data = 16 - i;
		HAL_GPIO_WritePin(GPIO_CK, GPIO_CK_PIN,GPIO_PIN_SET);
		delay_us(1);
		HAL_GPIO_WritePin(GPIO_CK, GPIO_CK_PIN,GPIO_PIN_RESET);
	}
	return data;
}

void KeyAction(uint8_t key)
{
	switch(key)
	{
		case 1:
		{
			
		}
	  break;
		case 2:
		{
			
		}	
	  break;
		case 3:
		{
			
		}
	break;
		
		case 4:
		{
			
		}
	break;
		
		case 5:
		{
			
		}
	  break;
				
		case 6:
			{
		}
	break;
				
		case 7:
			{
		}
	  break;
				
		case 8:
			{
		}
	  break;
		
		case 9:
			{
			
		}
	   break;
				
		case 10:
			{
			
		}
	  break;
				
		case 11:
			{
			
		}
	  break;
				
		case 12:
		{
			
		}
	  break;
				
		case 13:
			{
			
		}
	  break;
				
		case 14:
			{
			
		}
	  break;
				
		case 15:
			{
		}
	  break;
				
		case 16:
			{
		}
	  break;
	}	
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	KeyAction(Keyboard_ReadData());
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
