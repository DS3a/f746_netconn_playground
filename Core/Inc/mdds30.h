/*
 * mdds30.h
 *
 *  Created on: Sep 9, 2022
 *      Author: shashank
 */



#ifndef INC_MDDS30_H_
#define INC_MDDS30_H_

#define HAL_TIM_MODULE_ENABLED
#include "stm32f7xx_hal.h"

//DC Motor Rotation Direction
#define DIR_CW   1
#define DIR_ACW  0



//DC Motor Rotation
#define DC_MOTOR_PWM_RES 10
#define DC_MOTOR_F_PWM 500

//The Number of DC motors used
#define DC_MOTOR_UNITS 1

typedef struct{
	GPIO_TypeDef *IN_GPIO;
	uint16_t 	  IN_PIN;
	TIM_TypeDef  *TIM_Instance;
	uint32_t	  PWM_TIM_CH;
	uint16_t	  TIM_CLK_MHz;
	uint32_t      PWM_FREQ_Hz;
	uint8_t		  PWM_RES_BITS;


}DC_MOTOR_CfgType;

void DC_MOTOR_Init(DC_MOTOR_CfgType au8_MOTOR_Instance);
void DC_MOTOR_Start(DC_MOTOR_CfgType au8_MOTOR_Instance,uint8_t au8_DIR,uint16_t au16_SPEED);
void DC_MOTOR_Set_Speed(DC_MOTOR_CfgType au8_MOTOR_Instance,uint16_t au16_SPEED);
void DC_MOTOR_Set_Dir(DC_MOTOR_CfgType au8_MOTOR_Instance,uint8_t au8_DIR);
void DC_MOTOR_Stop(DC_MOTOR_CfgType au8_MOTOR_Instance);
uint32_t DC_MOTOR_Get_MaxFreq(DC_MOTOR_CfgType au8_MOTOR_Instance);

#endif /* INC_MDDS30_H_ */

