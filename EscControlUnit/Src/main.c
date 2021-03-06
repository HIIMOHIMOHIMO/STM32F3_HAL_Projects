/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "defined_constants.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim); 
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
/*--- buffer ---*/
int _index;
uint8_t RxData;
uint8_t Rxbuff[5];
uint16_t databuf[3];
float esc_value[10]={val_default};
void sendData(uint16_t *Data);
void REDEOn();
void REDEOff();
void EV1On();
void EV1Off();
void EV2On();
void EV2Off();
void EV3On();
void EV3Off();
void EV4On();
void EV4Off();
void EV5On();
void EV5Off();
void EV6On();
void EV6Off();
void EV7On();
void EV7Off();
void LED1On();
void LED1Off();
void LED1Toggle();
void LED2On();
void LED2Off();
void LED2Toggle();
void EscStart();
void EscStop();
uint8_t isOn(uint8_t a,uint8_t b,uint8_t c);
uint8_t ESC_CONTROL(uint8_t *Data);
void user_A0_pwm_setvalue(float value);
void user_A1_pwm_setvalue(float value);
void user_A3_pwm_setvalue(float value);
void user_A4_pwm_setvalue(float value);
void user_A6_pwm_setvalue(float value);
void user_B0_pwm_setvalue(float value);
void user_B1_pwm_setvalue(float value);
void user_A8_pwm_setvalue(float value);
void user_A9_pwm_setvalue(float value);
void user_A10_pwm_setvalue(float value);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,&RxData,1);
	LED1Off();
	LED2Off();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  LED1Toggle();
  HAL_Delay(10);
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 39999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, EV7_Pin|EV5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|EV6_Pin|EV3_Pin 
                          |EV4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, REDE_Pin|EV1_Pin|EV2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EV7_Pin EV5_Pin */
  GPIO_InitStruct.Pin = EV7_Pin|EV5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin EV6_Pin EV3_Pin 
                           EV4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|EV6_Pin|EV3_Pin 
                          |EV4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : REDE_Pin EV1_Pin EV2_Pin */
  GPIO_InitStruct.Pin = REDE_Pin|EV1_Pin|EV2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*GPIO_On*/
void EV1On(){
	HAL_GPIO_WritePin(EV1_GPIO_Port,EV1_Pin,GPIO_PIN_SET);
}
void EV2On(){
	HAL_GPIO_WritePin(EV2_GPIO_Port,EV2_Pin,GPIO_PIN_SET);
}
void EV3On(){
	HAL_GPIO_WritePin(EV3_GPIO_Port,EV3_Pin,GPIO_PIN_SET);
}
void EV4On(){
	HAL_GPIO_WritePin(EV4_GPIO_Port,EV4_Pin,GPIO_PIN_SET);
}
void EV5On(){
	HAL_GPIO_WritePin(EV5_GPIO_Port,EV5_Pin,GPIO_PIN_SET);
}
void EV6On(){
	HAL_GPIO_WritePin(EV6_GPIO_Port,EV6_Pin,GPIO_PIN_SET);
}
void EV7On(){
	HAL_GPIO_WritePin(EV7_GPIO_Port,EV7_Pin,GPIO_PIN_SET);
}
void LED1On(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
}
void LED2On(){
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
}
void REDEOn(){
	HAL_GPIO_WritePin(REDE_GPIO_Port,REDE_Pin,GPIO_PIN_SET);
}
/*GPIO_Off*/
void EV1Off(){
	HAL_GPIO_WritePin(EV1_GPIO_Port,EV1_Pin,GPIO_PIN_RESET);
}
void EV2Off(){
	HAL_GPIO_WritePin(EV2_GPIO_Port,EV2_Pin,GPIO_PIN_RESET);
}
void EV3Off(){
	HAL_GPIO_WritePin(EV3_GPIO_Port,EV3_Pin,GPIO_PIN_RESET);
}
void EV4Off(){
	HAL_GPIO_WritePin(EV4_GPIO_Port,EV4_Pin,GPIO_PIN_RESET);
}
void EV5Off(){
	HAL_GPIO_WritePin(EV5_GPIO_Port,EV5_Pin,GPIO_PIN_RESET);
}
void EV6Off(){
	HAL_GPIO_WritePin(EV6_GPIO_Port,EV6_Pin,GPIO_PIN_RESET);
}
void EV7Off(){
	HAL_GPIO_WritePin(EV7_GPIO_Port,EV7_Pin,GPIO_PIN_RESET);
}
void LED1Off(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
}
void LED2Off(){
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
}
void REDEOff(){
	HAL_GPIO_WritePin(REDE_GPIO_Port,REDE_Pin,GPIO_PIN_RESET);
}
/*GPIOToggle*/
void LED1Toggle(){
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
}
void LED2Toggle(){
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}
/*Interupt*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	volatile int j=0;
	HAL_UART_Receive_IT(&huart1, &RxData, 1);
	uint8_t Data = RxData;
	if(_index != 0){
		Rxbuff[_index-1] = Data;
		_index++;
		if(_index == 5){
			_index=0;
			if(Rxbuff[0] == 0xBB){
				ESC_CONTROL(Rxbuff);
			}
		}
	}else
		if(Data == 0x23){
		_index=1;
	}
}
/*UART*/
void sendData(uint16_t *Data){
	int i;
	volatile int j;
	uint8_t TxData[10];
	uint8_t CheckSum = 0;
	TxData[0] = '#';
	TxData[1] = 0xBB;
	TxData[2] = 0x06;
	TxData[3] = ((Data[0]>>0)&0xFF);
	TxData[4] = ((Data[0]>>8)&0xFF);
	TxData[5] = ((Data[1]>>0)&0xFF);
	TxData[6] = ((Data[1]>>8)&0xFF);
	TxData[7] = ((Data[2]>>0)&0xFF);
	TxData[8] = ((Data[2]>>8)&0xFF);
	for(i=0; i<9; i++){
		CheckSum = CheckSum ^ TxData[i];
	}
	TxData[9] = CheckSum;
	REDEOn();
	HAL_UART_Transmit(&huart1,TxData,10,1);
	for(j=0;j<SENDDELAYCOUNT;j++);
	REDEOff();
}
/*ESC*/
uint8_t ESC_CONTROL(uint8_t *Dat){
	int i=0;
	uint8_t Data[4];
	for(i=0;i<4;i++){
		Data[i] = Dat[i];
	}
	int _checksum = 0;
	_checksum = _checksum^0x23;
	for(i=0;i<3;i++){
		_checksum = _checksum ^ Data[i];
	}
	if(Data[3] == _checksum){
		if((Data[1] & 0x01) != 0 )EV1On();
		else EV1Off();
		if((Data[1] & 0x02) != 0 )EV2On();
		else EV2Off();
		if((Data[1] & 0x04) != 0 )EV3On();
		else EV3Off();
		if((Data[1] & 0x08) != 0 )EV4On();
		else EV4Off();
		if((Data[1] & 0x10) != 0 )EV5On();
		else EV5Off();
		if((Data[1] & 0x20) != 0 )EV6On();
		else EV6Off();
		if((Data[1] & 0x40) != 0 )EV7On();
		else EV7Off();
		if((Data[1] & 0x80) != 0 )EscStart();
		else EscStop();
		LED2Toggle();
		return true;
	}else{
		return false;
	}
}
void EscStart(){
	user_A0_pwm_setvalue(val_A0_1);
	user_A1_pwm_setvalue(val_A1_1);
	user_A3_pwm_setvalue(val_A3_1);
	user_A4_pwm_setvalue(val_A4_1);
	user_A6_pwm_setvalue(val_A6_1);
	user_B0_pwm_setvalue(val_B0_1);
	user_B1_pwm_setvalue(val_B1_1);
}
void EscStop(){
	user_A0_pwm_setvalue(val_default);
	user_A1_pwm_setvalue(val_default);
	user_A3_pwm_setvalue(val_default);
	user_A4_pwm_setvalue(val_default);
	user_A6_pwm_setvalue(val_default);
	user_B0_pwm_setvalue(val_default);
	user_B1_pwm_setvalue(val_default);
}
uint8_t isOn(uint8_t a,uint8_t b,uint8_t c){
	return ((a&c)!=0 + (b&c)!=0);
}
/*timer*/
void user_A0_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
}
void user_A1_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_2);
}
void user_A3_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
   HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_4);
}
void user_A4_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_2);
}
void user_A6_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
}
void user_B0_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_3);
}
void user_B1_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
   HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_4);
}
void user_A8_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
}
void user_A9_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}
void user_A10_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((39999)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
