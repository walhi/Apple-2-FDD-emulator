#include "dsk2nic.h"

#define VOLUME 0xfe

__attribute__((always_inline)) void writeAAVal(uint8_t val, uint8_t *target)
{
	*(target++) = 0xaa | (val >> 1);
	*target = 0xaa | val;
}

void dsk2nic(uint8_t *src, uint8_t *dst, uint8_t track, uint8_t sector)
{
	static const uint8_t encTable[] = {
		0x96,0x97,0x9A,0x9B,0x9D,0x9E,0x9F,0xA6,
		0xA7,0xAB,0xAC,0xAD,0xAE,0xAF,0xB2,0xB3,
		0xB4,0xB5,0xB6,0xB7,0xB9,0xBA,0xBB,0xBC,
		0xBD,0xBE,0xBF,0xCB,0xCD,0xCE,0xCF,0xD3,
		0xD6,0xD7,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,
		0xDF,0xE5,0xE6,0xE7,0xE9,0xEA,0xEB,0xEC,
		0xED,0xEE,0xEF,0xF2,0xF3,0xF4,0xF5,0xF6,
		0xF7,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
	};
	static const uint8_t FlipBit1[4] = { 0, 2,  1,  3  };
	static const uint8_t FlipBit2[4] = { 0, 8,  4,  12 };
	static const uint8_t FlipBit3[4] = { 0, 32, 16, 48 };

	int i;
	uint8_t x, ox = 0;

	for (i = 0; i < 22; i++) *(dst++) = 0xff;

	*(dst++) = 0x03;
	*(dst++) = 0xfc;
	*(dst++) = 0xff;
	*(dst++) = 0x3f;
	*(dst++) = 0xcf;
	*(dst++) = 0xf3;
	*(dst++) = 0xfc;
	*(dst++) = 0xff;
	*(dst++) = 0x3f;
	*(dst++) = 0xcf;
	*(dst++) = 0xf3;
	*(dst++) = 0xfc;

	// Address marker start
	*(dst++) = 0xd5;
	*(dst++) = 0xaa;
	*(dst++) = 0x96;
	// Address data
	writeAAVal(VOLUME, dst);
	dst += 2;
	writeAAVal(track, dst);
	dst += 2;
	writeAAVal(sector, dst);
	dst += 2;
	writeAAVal(VOLUME ^ track ^ sector, dst);
	dst += 2;
	// Address marker end
	*(dst++) = 0xde;
	*(dst++) = 0xaa;
	*(dst++) = 0xeb;

	// Sync
	for (i = 0; i < 5; i++) *(dst++) = 0xff;

	// Data marker start
	*(dst++) = 0xd5;
	*(dst++) = 0xaa;
	*(dst++) = 0xad;
	// Data

	// ?????????
	src[256] = src[257] = 0;

	for (i = 0; i < 86; i++) {
		x = (FlipBit1[src[i] & 3] | FlipBit2[src[i + 86] & 3] | FlipBit3[src[i + 172] & 3]);
		*(dst++) = encTable[(x ^ ox) & 0x3f];
		ox = x;
	}
	for (i = 0; i < 256; i++) {
		x = (src[i] >> 2);
		*(dst++) = encTable[(x ^ ox) & 0x3f];
		ox = x;
	}
	*(dst++) = encTable[ox & 0x3f];
	// Data marker end
	*(dst++) = 0xde;
	*(dst++) = 0xaa;
	*(dst++) = 0xeb;

	// memset ?
	for (i = 0; i < 14; i++) *(dst++) = 0xff;
	for (i = 0; i < (512 - 416); i++) *(dst++) = 0x00;
}
