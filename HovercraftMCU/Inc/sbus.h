/*
 * sbus.h
 *
 *  Created on: 27 août 2019
 *      Author: gfouille
 */

#ifndef SBUS_H_
#define SBUS_H_

#include <stdint.h>

struct sbusframe_raw
{
	uint8_t Header: 8;
	uint16_t Channel_1: 11;
	uint16_t Channel_2: 11;
	uint16_t Channel_3: 11;
	uint16_t Channel_4: 11;
	uint16_t Channel_5: 11;
	uint16_t Channel_6: 11;
	uint16_t Channel_7: 11;
	uint16_t Channel_8: 11;
	uint16_t Channel_9: 11;
	uint16_t Channel_10: 11;
	uint16_t Channel_11: 11;
	uint16_t Channel_12: 11;
	uint16_t Channel_13: 11;
	uint16_t Channel_14: 11;
	uint16_t Channel_15: 11;
	uint16_t Channel_16: 11;
	uint8_t Channel_17: 1;
	uint8_t Channel_18: 1;
	uint8_t FrameLost: 1;
	uint8_t FailSafe: 1;
	uint8_t Zero: 4;
	uint8_t Footer: 8;
}__attribute__((packed));

struct sbusframe_user
{
	uint16_t Channels[16];
};

uint32_t SBUS_Init(void);
uint32_t SBUS_AddByte(uint8_t Byte);
uint32_t SBUS_GetFrame(struct sbusframe_user *pFrame);


#endif /* SBUS_H_ */
