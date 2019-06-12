#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include <stdint.h>


static const unsigned char dsk_scramble[] = {
	0,7,14,6,13,5,12,4,11,3,10,2,9,1,8,15
};

void dsk2nic(uint8_t *src, uint8_t *dst, uint8_t track, uint8_t sector);

#endif
