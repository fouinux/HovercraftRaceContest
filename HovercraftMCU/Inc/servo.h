/*
 * servo.h
 *
 *  Created on: 26 août 2019
 *      Author: Guillaume
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f1xx_hal.h"

#define SERVO_POSITION_MIN	(-1000)
#define SERVO_POSITION_MAX	(+1000)

uint32_t SERVO_Init(TIM_HandleTypeDef *pTimer, uint32_t Channel);
uint32_t SERVO_SetPosition(TIM_HandleTypeDef *pTimer, uint32_t Channel, int16_t Position);

#endif /* SERVO_H_ */
