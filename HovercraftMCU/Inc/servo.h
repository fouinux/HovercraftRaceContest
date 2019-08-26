/*
 * servo.h
 *
 *  Created on: 26 août 2019
 *      Author: Guillaume
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f1xx_hal.h"

#define SERVO_POSITION_MIN	(-100)
#define SERVO_POSITION_MAX	(+100)

uint32_t servo_init(TIM_HandleTypeDef *pTimer, uint32_t Channel);
uint32_t servo_set_position(TIM_HandleTypeDef *pTimer, uint32_t Channel, int8_t Position);

#endif /* SERVO_H_ */
