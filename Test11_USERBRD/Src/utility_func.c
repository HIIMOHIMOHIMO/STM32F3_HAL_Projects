/* GPIO LED */
//LD2
void LD2_ON(){
	GPIO_WritePin(GPIOF,D2_Pin,GPIO_PIN_SET);
}
void LD2_OFF(){
	GPIO_WritePin(GPIOF,D2_Pin,GPIO_PIN_RESET);
}
void LD2_Toggle(){
	GPIO_TogglePin(GPIOF,D2_Pin);
}
//LD3
void LD3_ON(){
	GPIO_WritePin(GPIOF,D3_Pin,GPIO_PIN_SET);
}
void LD3_OFF(){
	GPIO_WritePin(GPIOF,D3_Pin,GPIO_PIN_RESET);
}
void LD3_Toggle(){
	GPIO_TogglePin(GPIOF,D3_Pin);
}
//LD4
void LD4_ON(){
	GPIO_WritePin(GPIOA,D4_Pin,GPIO_PIN_SET);
}
void LD4_OFF(){
	GPIO_WritePin(GPIOA,D4_Pin,GPIO_PIN_RESET);
}
void LD4_Toggle(){
	GPIO_TogglePin(GPIOA,D4_Pin);
}
//LD5
void LD5_ON(){
	GPIO_WritePin(GPIOA,D5_Pin,GPIO_PIN_SET);
}
void LD5_OFF(){
	GPIO_WritePin(GPIOA,D5_Pin,GPIO_PIN_RESET);
}
void LD5_Toggle(){
	GPIO_TogglePin(GPIOA,D5_Pin);
}
/* Timer PWM */
void user_tim3_1_setvalue(float value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)((PWM_VALUE-1)*value);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  
}
void user_tim3_2_setvalue(float value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)((PWM_VALUE-1)*value);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  
}
void user_tim16_1_setvalue(float value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)((PWM_VALUE-1)*value);
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);  
}
void user_tim17_1_setvalue(float value)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)((PWM_VALUE)*value);
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);  
}
