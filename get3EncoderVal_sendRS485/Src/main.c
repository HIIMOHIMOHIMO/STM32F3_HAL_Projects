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
#define SENDDELAYCOUNT 500
#define RCVDELAYCOUNT 510
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

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
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*--- buffer ---*/
int _index;
uint8_t RxData;
uint8_t RxBuff[5];
uint16_t databuf[3];
uint8_t UartReady;
/*--- define ---*/
#define true (1)
#define false (0)
/*--- Encoder ---*/
void Encoder_Read_Strat();
void Encoder_Read_Stop();
int getTIM1Enc();
int getTIM2Enc();
int getTIM3Enc();
int isChange(int old,int new);
/*--- UART ---*/
void sendData(uint16_t *Data);
void REDEOn();
void REDEOff();
void waitRxInterrupt();
/*--- LED ---*/
void stateLED(int State);
void Led0On();
void Led1On();
void Led2On();
void Led0Off();
void Led1Off();
void Led2Off();
void Led0Toggle();
void Led1Toggle();
void Led2Toggle();
/*--- JP ---*/
int Bit1_state();
int Bit2_state();
/*--- out cnt ---*/
void output_cntled();
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t Enc1_value=0,Enc2_value=0,Enc3_value=0;
	uint16_t Enc1_old = 0,Enc2_old = 0,Enc3_old = 0;
	UartReady = false;
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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &RxData, 1);
	Encoder_Read_Strat();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		Enc1_value = getTIM1Enc();
		Enc2_value = getTIM2Enc();
		Enc3_value = getTIM3Enc();

		if( isChange(Enc1_old,Enc1_value) != 0) Led2Toggle();
		if( isChange(Enc2_old,Enc2_value) != 0) Led0Toggle();	
		if( isChange(Enc3_old,Enc3_value) != 0) Led1Toggle();

		Enc1_old = Enc1_value;
		Enc2_old = Enc2_value;
		Enc3_old = Enc3_value;

		/*--- Send Data ---*/
		databuf[0] = Enc1_value;
		databuf[1] = Enc2_value;
		databuf[2] = Enc3_value;

//		sendData(databuf);
		output_cntled();

		HAL_Delay(10);

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
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

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
  HAL_GPIO_WritePin(StateLED_GPIO_Port, StateLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, REDE_Pin|LED0_Pin|B7_Pin|B6_Pin 
                          |B5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|B4_Pin|B3_Pin 
                          |B2_Pin|B1_Pin|B0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : StateLED_Pin */
  GPIO_InitStruct.Pin = StateLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(StateLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Bit1_state_Pin */
  GPIO_InitStruct.Pin = Bit1_state_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Bit1_state_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : REDE_Pin LED0_Pin B7_Pin B6_Pin 
                           B5_Pin */
  GPIO_InitStruct.Pin = REDE_Pin|LED0_Pin|B7_Pin|B6_Pin 
                          |B5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin B4_Pin B3_Pin 
                           B2_Pin B1_Pin B0_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|B4_Pin|B3_Pin 
                          |B2_Pin|B1_Pin|B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Bit2_state_Pin */
  GPIO_InitStruct.Pin = Bit2_state_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Bit2_state_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*--- Encoder ---*/
void Encoder_Read_Strat(){
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
}
void Encoder_Read_Stop(){
	HAL_TIM_Encoder_Stop(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_ALL);
}
int getTIM1Enc(){
	return TIM1->CNT;
}
int getTIM2Enc(){
	return TIM2->CNT;
}
int getTIM3Enc(){
	return TIM3->CNT;
}
int isChange(int old,int new){
	if(old != new){
		return true;
	}else{
		return false;
	}
}
/*--- UART ---*/
void sendData(uint16_t *Data){
	int i;
	volatile int j;
	uint8_t TxData[10];
	uint8_t CheckSum = 0;
	TxData[0] = '#';
	TxData[1] = 0xEE;
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
	HAL_UART_Transmit(&huart2,TxData,10,1);
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
	REDEOff();
	stateLED(2);
}
void REDEOn(){
	HAL_GPIO_WritePin(REDE_GPIO_Port,REDE_Pin,GPIO_PIN_SET);
}
void REDEOff(){
	HAL_GPIO_WritePin(REDE_GPIO_Port,REDE_Pin,GPIO_PIN_RESET);
}
/*--- Interrupt ---*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	volatile int j=0;
  HAL_UART_Receive_IT(&huart2, &RxData, 1);
	uint8_t Data = RxData;
	if(_index != 0){
		RxBuff[_index-1] = Data;
		_index++;
		if(_index == 5){
			_index = 0;
			if(RxBuff[0]==0xEE && RxBuff[1]==0x01){
				//HAL_Delay(1);
				//for(j=0;j<RCVDELAYCOUNT;j++);
				sendData(databuf);
			}
	
		}
	}else
		if(Data == 0x23){
			_index = 1;
		}
}
void waitRxInterrupt(){
	while(1){
		if(RxBuff[0] == 0xEE && RxBuff[1]==0x01)break;
	}
}
/*--- LED ---*/
void stateLED(int State){
	if(State == 1){
		HAL_GPIO_WritePin(StateLED_GPIO_Port,StateLED_Pin,GPIO_PIN_SET);
	}else
	if(State == 0){
		HAL_GPIO_WritePin(StateLED_GPIO_Port,StateLED_Pin,GPIO_PIN_RESET);
	}else
	{
		HAL_GPIO_TogglePin(StateLED_GPIO_Port,StateLED_Pin);
	}
}
void Led0On(){
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
}
void Led1On(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
}
void Led2On(){
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
}
void Led0Off(){
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
}
void Led1Off(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
}
void Led2Off(){
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
}
void Led0Toggle(){
	HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
}
void Led1Toggle(){
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
}
void Led2Toggle(){
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}
/*--- JP ---*/
int Bit1_state(){
	return HAL_GPIO_ReadPin(Bit1_state_GPIO_Port,Bit1_state_Pin);
}
int Bit2_state(){
	return HAL_GPIO_ReadPin(Bit2_state_GPIO_Port,Bit2_state_Pin);
}
/*--- out cnt ---*/
void output_cntled(){
	int bit1_st = Bit1_state();
	int bit2_st = Bit2_state();
	if( bit1_st == 0 && bit2_st == 1 ){
		if((databuf[0]&0x01) != 0)HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x02) != 0)HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x04) != 0)HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x08) != 0)HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x10) != 0)HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x20) != 0)HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x40) != 0)HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x80) != 0)HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
	}else 
	if( bit1_st == 1 && bit2_st == 0 ){
		if((databuf[1]&0x01) != 0)HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x02) != 0)HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x04) != 0)HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x08) != 0)HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x10) != 0)HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x20) != 0)HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x40) != 0)HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_RESET);

		if((databuf[1]&0x80) != 0)HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
	}else
	if( bit1_st == 0 && bit2_st ==0  ){

		if((databuf[2]&0x01) != 0)HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x02) != 0)HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);

		if((databuf[2]&0x04) != 0)HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x08) != 0)HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_RESET);

		if((databuf[2]&0x10) != 0)HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x20) != 0)HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x40) != 0)HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_SET);

		else HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x80) != 0)HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
	}

}
/*
void output_cntled(){
	int bit1_st = Bit1_state();
	int bit2_st = Bit2_state();
	if( bit1_st == 0 && bit2_st == 1 ){
		if((databuf[0]&0x0100) != 0)HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x0200) != 0)HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x0400) != 0)HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x0800) != 0)HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x1000) != 0)HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x2000) != 0)HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x4000) != 0)HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_RESET);
		if((databuf[0]&0x8000) != 0)HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
	}else 
	if( bit1_st == 1 && bit2_st == 0 ){
		if((databuf[1]&0x0100) != 0)HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x0200) != 0)HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x0400) != 0)HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x0800) != 0)HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x1000) != 0)HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x2000) != 0)HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x4000) != 0)HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_RESET);
		if((databuf[1]&0x8000) != 0)HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
	}else
	if( bit1_st == 0 && bit2_st ==0  ){
		if((databuf[2]&0x0100) != 0)HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B0_GPIO_Port,B0_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x0200) != 0)HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x0400) != 0)HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x0800) != 0)HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B3_GPIO_Port,B3_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x1000) != 0)HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B4_GPIO_Port,B4_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x2000) != 0)HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B5_GPIO_Port,B5_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x4000) != 0)HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B6_GPIO_Port,B6_Pin,GPIO_PIN_RESET);
		if((databuf[2]&0x8000) != 0)HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
	}
}*/
/*---  ---*/
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
