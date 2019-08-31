/*
 * motor.c
 *
 *  Created on: 29 août 2019
 *      Author: Guillaume
 */

#include <stdint.h>

#include "stm32f1xx_hal.h"

#define MOTOR_PWM_MIN 		0
#define MOTOR_PWM_MAX 		1023
#define MOTOR_PWM_DEFAULT 	0

uint32_t MOTOR_Init(TIM_HandleTypeDef *pTimer, uint32_t Channel)
{
	if (NULL == pTimer)
			return 1;

		// Set default PWM threshold
		switch(Channel)
		{
		case TIM_CHANNEL_1:
			pTimer->Instance->CCR1 = MOTOR_PWM_DEFAULT;
			break;
		case TIM_CHANNEL_2:
			pTimer->Instance->CCR2 = MOTOR_PWM_DEFAULT;
			break;
		case TIM_CHANNEL_3:
			pTimer->Instance->CCR3 = MOTOR_PWM_DEFAULT;
			break;
		case TIM_CHANNEL_4:
			pTimer->Instance->CCR4 = MOTOR_PWM_DEFAULT;
			break;
		default:
			return 1;
		}

		// Start PWM
		HAL_TIM_PWM_Start(pTimer, Channel);

	return 0;
}

uint32_t MOTOR_SetSpeed(TIM_HandleTypeDef *pTimer, uint32_t ChannelA, uint32_t ChannelB, int16_t Speed)
{
	if (NULL == pTimer)
			return 1;

		// Set default PWM threshold
		/*switch(Channel)
		{
		case TIM_CHANNEL_1:
			pTimer->Instance->CCR1 = MOTOR_PWM_DEFAULT;
			break;
		case TIM_CHANNEL_2:
			pTimer->Instance->CCR2 = MOTOR_PWM_DEFAULT;
			break;
		case TIM_CHANNEL_3:
			pTimer->Instance->CCR3 = MOTOR_PWM_DEFAULT;
			break;
		case TIM_CHANNEL_4:
			pTimer->Instance->CCR4 = MOTOR_PWM_DEFAULT;
			break;
		default:
			return 1;
		}

		// Start PWM
		HAL_TIM_PWM_Start(pTimer, Channel);*/

	return 0;
}
