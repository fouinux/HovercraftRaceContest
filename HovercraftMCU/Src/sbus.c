/*
 * sbus.c
 *
 *  Created on: 27 août 2019
 *      Author: gfouille
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <sbus.h>

#include "usbd_cdc_if.h"

#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

#define SBUS_FRAME_SIZE 25

uint8_t gFIFO[SBUS_FRAME_SIZE];
uint8_t gFIFOIndex;
uint8_t gFIFOCounter;

uint8_t gBufferedFrame[SBUS_FRAME_SIZE];

uint32_t SBUS_Init(void)
{
	// Reset FIFO
	memset(&gFIFO[0], 0, sizeof(gFIFO));
	gFIFOIndex = 0;
	gFIFOCounter = 0;

	// Reset buffered frame
	memset(&gBufferedFrame[0], 0, sizeof(gBufferedFrame));

	return 0;
}


uint32_t SBUS_AddByte(uint8_t Byte)
{
	gFIFO[gFIFOIndex++] = Byte;
	if (gFIFOIndex >= SBUS_FRAME_SIZE)
		gFIFOIndex = 0;

	if (gFIFOCounter < SBUS_FRAME_SIZE)
	{
		gFIFOCounter++;
	}

	return 0;
}


uint32_t SBUS_GetFrame(struct sbusframe_user *pFrame)
{
	struct sbusframe_raw *pSBUSFrame = (struct sbusframe_raw *) &gBufferedFrame[0];
	if (NULL == pFrame)
		return 1;

	// Enough byte received
	if (gFIFOCounter == SBUS_FRAME_SIZE)
	{
		// Frame with Header and Footer?
		uint8_t Header = gFIFO[gFIFOIndex];
		uint8_t Footer;
		if (gFIFOIndex != 0)
			Footer = gFIFO[gFIFOIndex - 1];
		else
			Footer = gFIFO[SBUS_FRAME_SIZE - 1];

		if (Header == SBUS_HEADER && Footer == SBUS_FOOTER)
		{
			// Extract frame
			memcpy(&gBufferedFrame[0], &gFIFO[gFIFOIndex], SBUS_FRAME_SIZE - gFIFOIndex);
			memcpy(&gBufferedFrame[SBUS_FRAME_SIZE - gFIFOIndex], &gFIFO[0], gFIFOIndex);

			// Debug SBUS
//			char Str[128];
//			sprintf(Str, "%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%01X:%01X:%01X:%01X\r\n",
//					pSBUSFrame->Channel_1,
//					pSBUSFrame->Channel_2,
//					pSBUSFrame->Channel_3,
//					pSBUSFrame->Channel_4,
//					pSBUSFrame->Channel_5,
//					pSBUSFrame->Channel_6,
//					pSBUSFrame->Channel_7,
//					pSBUSFrame->Channel_8,
//					pSBUSFrame->Channel_9,
//					pSBUSFrame->Channel_10,
//					pSBUSFrame->Channel_11,
//					pSBUSFrame->Channel_12,
//					pSBUSFrame->Channel_13,
//					pSBUSFrame->Channel_14,
//					pSBUSFrame->Channel_15,
//					pSBUSFrame->Channel_16,
//					pSBUSFrame->Channel_17,
//					pSBUSFrame->Channel_18,
//					pSBUSFrame->FailSafe,
//					pSBUSFrame->FrameLost);
//			CDC_Transmit_FS((uint8_t *)Str, strlen(Str));

			// Check for error
			if (pSBUSFrame->FailSafe || pSBUSFrame->FrameLost || pSBUSFrame->Zero)
				return 1;

			// Parse frame
			pFrame->Channels[0] = pSBUSFrame->Channel_1;
			pFrame->Channels[1] = pSBUSFrame->Channel_2;
			pFrame->Channels[2] = pSBUSFrame->Channel_3;
			pFrame->Channels[3] = pSBUSFrame->Channel_4;
			pFrame->Channels[4] = pSBUSFrame->Channel_5;
			pFrame->Channels[5] = pSBUSFrame->Channel_6;
			pFrame->Channels[6] = pSBUSFrame->Channel_7;
			pFrame->Channels[7] = pSBUSFrame->Channel_8;
			pFrame->Channels[8] = pSBUSFrame->Channel_9;
			pFrame->Channels[9] = pSBUSFrame->Channel_10;
			pFrame->Channels[10] = pSBUSFrame->Channel_11;
			pFrame->Channels[11] = pSBUSFrame->Channel_12;
			pFrame->Channels[12] = pSBUSFrame->Channel_13;
			pFrame->Channels[13] = pSBUSFrame->Channel_14;
			pFrame->Channels[14] = pSBUSFrame->Channel_15;
			pFrame->Channels[15] = pSBUSFrame->Channel_16;

			// Clean FIFO
			gFIFOIndex = 0;
			gFIFOCounter = 0;
		}
	}

	return 0;
}

#define SBUS_CHANNEL_VALUE_MIN	172
#define SBUS_CHANNEL_VALUE_MAX	1811

int32_t SBUS_NormalizeChannel(uint16_t ChannelValue, int32_t Min, int32_t Max)
{
	int32_t ScaleA = Max - Min;
	int32_t ScaleB = (Min * SBUS_CHANNEL_VALUE_MAX) - (Max * SBUS_CHANNEL_VALUE_MIN);
	int32_t ScaleD = SBUS_CHANNEL_VALUE_MAX - SBUS_CHANNEL_VALUE_MIN;

	return ((int32_t) ChannelValue * ScaleA + ScaleB) / ScaleD;
}
