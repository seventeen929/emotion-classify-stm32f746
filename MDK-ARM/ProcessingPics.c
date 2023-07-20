#include "string.h"
#include "stdio.h"
#include <stdint.h>
#include "ProcessingPics.h"


void resize(uint16_t*image)
{
	for(int j = 0 ; j < 56 ; j++)
	{
		for(int i = 0 ; i < 56; i++)//横
		{
			//downImage[j*56 + i] = image[2560 + i*5 + j* 1200 ];
			//downImage[j*56 + i] = (image[2560 + i*5 + j* 1200] + image[2560 + i*5 + (j-1)*1200 ] + image[2560 + i*5 + (j+1)*1200 ] + image[2560 + (i+1)*5 + j*1200] + image[2560 + (i-1)*5 + j*1200])/5;
		}
	}
}