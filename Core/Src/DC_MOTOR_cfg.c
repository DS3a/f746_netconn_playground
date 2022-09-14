/*
 * DC_MOTOR_cfg.c
 *
 *  Created on: 10-Sep-2022
 *      Author: shashank
 */

#include "mdds30.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS]=
{
		//Steering Configurations PA6 TIM 3 CHANNEL 1

		{
				GPIOA,
				GPIO_PIN_7,
				TIM3,
				TIM_CHANNEL_1,
				72,
				DC_MOTOR_F_PWM,
				DC_MOTOR_PWM_RES

		},

};

