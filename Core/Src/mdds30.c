/*
 * mdds30.c
 *
 *  Created on: Sep 9, 2022
 *      Author: Shashank
 */


#include "mdds30.h"
#include "DC_MOTOR_cfg.h"
#include "main.h"

void DC_MOTOR_Init(DC_MOTOR_CfgType au8_MOTOR_Instance)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_HandleTypeDef htim;
	uint32_t PSC_Value = 0;
	uint32_t ARR_Value = 0;
	uint8_t i = 0;

	/*------------Configure Direction GPIO Pin---------------*/
//	if(DC_MOTOR_CfgParam[au8_MOTOR_Instance].IN_GPIO == GPIOA){
	if(au8_MOTOR_Instance.IN_GPIO == GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	GPIO_InitStruct.Pin = au8_MOTOR_Instance.IN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(au8_MOTOR_Instance.IN_GPIO, &GPIO_InitStruct);
	HAL_GPIO_WritePin(au8_MOTOR_Instance.IN_GPIO, au8_MOTOR_Instance.IN_PIN,0);

	//Calculate PSC & ARR
	ARR_Value = 1;
	for(i=0;i<au8_MOTOR_Instance.PWM_RES_BITS;i++){
		ARR_Value *=2;
	}
	PSC_Value =(uint32_t)((au8_MOTOR_Instance.TIM_CLK_MHz*1000000) / (ARR_Value*au8_MOTOR_Instance.PWM_FREQ_Hz));
	PSC_Value--;
	ARR_Value -= 2;

	//Configure the DC Motor PWM Timer Channel

	htim.Instance = au8_MOTOR_Instance.TIM_Instance;
	htim.Init.Prescaler = PSC_Value;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = ARR_Value;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, au8_MOTOR_Instance.PWM_TIM_CH);
	HAL_TIM_MspPostInit(&htim);

	//Start PWM
	HAL_TIM_PWM_Start(&htim, au8_MOTOR_Instance.PWM_TIM_CH);
}
void DC_MOTOR_Start(DC_MOTOR_CfgType au8_MOTOR_Instance,uint8_t au8_DIR,uint16_t au16_SPEED){
	//Write to Direction PIN

	if(au8_DIR == DIR_CW){
		HAL_GPIO_WritePin(au8_MOTOR_Instance.IN_GPIO, au8_MOTOR_Instance.IN_PIN, 1);

	}
	else if(au8_DIR == DIR_ACW){
		HAL_GPIO_WritePin(au8_MOTOR_Instance.IN_GPIO, au8_MOTOR_Instance.IN_PIN, 0);
	}

	//Write Speed -> Duty Cycle Register
	if(au8_MOTOR_Instance.PWM_TIM_CH == TIM_CHANNEL_1){
		au8_MOTOR_Instance.TIM_Instance->CCR1 = au16_SPEED;
	}
	else if(au8_MOTOR_Instance.PWM_TIM_CH == TIM_CHANNEL_2){
		au8_MOTOR_Instance.TIM_Instance->CCR2 = au16_SPEED;
	}
}

void DC_MOTOR_Set_Speed(DC_MOTOR_CfgType au8_MOTOR_Instance, uint16_t au16_SPEED){
	//Write SPeed -> DutyCYcle Register
	//Write Speed -> Duty Cycle Register
		if(au8_MOTOR_Instance.PWM_TIM_CH == TIM_CHANNEL_1){
			au8_MOTOR_Instance.TIM_Instance->CCR1 = au16_SPEED;
		}
		else if(au8_MOTOR_Instance.PWM_TIM_CH == TIM_CHANNEL_2){
			au8_MOTOR_Instance.TIM_Instance->CCR2 = au16_SPEED;
		}

}

void DC_MOTOR_Set_Dir(DC_MOTOR_CfgType au8_MOTOR_Instance,uint8_t au8_DIR){
	//Write to Direction Control Pin
	if(au8_DIR == DIR_CW){
			HAL_GPIO_WritePin(au8_MOTOR_Instance.IN_GPIO, au8_MOTOR_Instance.IN_PIN, 1);
		}
		else if(au8_DIR == DIR_ACW){
			HAL_GPIO_WritePin(au8_MOTOR_Instance.IN_GPIO, au8_MOTOR_Instance.IN_PIN, 0);
		}

}

void DC_MOTOR_Stop(DC_MOTOR_CfgType au8_MOTOR_Instance){
	//Write to Direction Pin
	HAL_GPIO_WritePin(au8_MOTOR_Instance.IN_GPIO, au8_MOTOR_Instance.IN_PIN, 0);

	//Write Zero to PWM DutyCycle Register
	if(au8_MOTOR_Instance.PWM_TIM_CH == TIM_CHANNEL_1){
		au8_MOTOR_Instance.TIM_Instance->CCR1 = 0;
	}
	else if(au8_MOTOR_Instance.PWM_TIM_CH == TIM_CHANNEL_2){
		au8_MOTOR_Instance.TIM_Instance->CCR2 = 0;
	}
}

uint32_t DC_MOTOR_Get_MaxFreq(DC_MOTOR_CfgType au8_MOTOR_Instance){
	uint32_t ARR_Value = 1;
	uint8_t i = 0;
	for(i=0 ; i<au8_MOTOR_Instance.PWM_RES_BITS;i++){
		ARR_Value *= 2;
	}
	return((au8_MOTOR_Instance.TIM_CLK_MHz*1000000)/ARR_Value);
}



