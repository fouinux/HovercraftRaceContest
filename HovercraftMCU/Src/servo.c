/*
 * servo.c
 *
 *  Created on: 26 août 2019
 *      Author: Guillaume
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <servo.h>

#include "stm32f1xx_hal.h"

#define SERVO_THRESHOLD_MIN		500UL
#define SERVO_THRESHOLD_MAX		2500UL
#define SERVO_THRESHOLD_DEFAULT	1500UL

float gServoScaleA = 1;
float gServoScaleB = 0;

uint32_t SERVO_Init(TIM_HandleTypeDef *pTimer, uint32_t Channel)
{
	if (NULL == pTimer)
		return 1;

	// Set default PWM threshold
	switch(Channel)
	{
	case TIM_CHANNEL_1:
		pTimer->Instance->CCR1 = SERVO_THRESHOLD_DEFAULT;
		break;
	case TIM_CHANNEL_2:
		pTimer->Instance->CCR2 = SERVO_THRESHOLD_DEFAULT;
		break;
	case TIM_CHANNEL_3:
		pTimer->Instance->CCR3 = SERVO_THRESHOLD_DEFAULT;
		break;
	case TIM_CHANNEL_4:
		pTimer->Instance->CCR4 = SERVO_THRESHOLD_DEFAULT;
		break;
	default:
		return 1;
	}

	// Compute scale A and B
	gServoScaleA = ((float) (SERVO_THRESHOLD_MAX - SERVO_THRESHOLD_MIN) /  (float) (SERVO_POSITION_MAX - SERVO_POSITION_MIN));
	gServoScaleB = ((float) SERVO_THRESHOLD_MAX - (gServoScaleA * (float) SERVO_POSITION_MAX));

	// Start PWM
	HAL_TIM_PWM_Start(pTimer, Channel);

	return 0;
}


uint32_t SERVO_SetPosition(TIM_HandleTypeDef *pTimer, uint32_t Channel, int16_t Position)
{
	uint16_t Threshold = 0;

	if (NULL == pTimer)
		return 1;

	// Limit position
	if (Position < SERVO_POSITION_MIN)
		Position = SERVO_POSITION_MIN;
	if (Position > SERVO_POSITION_MAX)
		Position = SERVO_POSITION_MAX;

	// Compute PWM threshold (y = ax + b)
	//Threshold = (uint16_t) roundf(gServoScaleA * Position + gServoScaleB);
	// TODO Remove super optimisation
	Threshold = Position + 1500;

	// Set PWM threshold
	switch(Channel)
	{
	case TIM_CHANNEL_1:
		pTimer->Instance->CCR1 = Threshold;
		break;
	case TIM_CHANNEL_2:
		pTimer->Instance->CCR2 = Threshold;
		break;
	case TIM_CHANNEL_3:
		pTimer->Instance->CCR3 = Threshold;
		break;
	case TIM_CHANNEL_4:
		pTimer->Instance->CCR4 = Threshold;
		break;
	default:
		return 1;
	}

	return 0;
}

