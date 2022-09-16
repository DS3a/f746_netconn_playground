/*
 * DC_MOTOR_cfg.c
 *
 *  Created on: 10-Sep-2022
 *      Author: shashank
 */

#include "mdds30.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS]=
{
		//Steering Configurations PA5 TIM 3 CHANNEL 1

		{
				GPIOA,
				GPIO_PIN_5,
				TIM3,
				TIM_CHANNEL_1,
				108,
				DC_MOTOR_F_PWM,
				DC_MOTOR_PWM_RES

		},

		//Steering Configurations PC8 TIM 3 CHANNEL 2

		{
				GPIOC,
				GPIO_PIN_8,
				TIM3,
				TIM_CHANNEL_2,
				108,
				DC_MOTOR_F_PWM,
				DC_MOTOR_PWM_RES

		},


};

