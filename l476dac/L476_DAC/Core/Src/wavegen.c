/*
 * wavegen.c
 *
 *  Created on: Jul 20, 2023
 *      Author: KMarton
 */

#include "wavegen.h"

void fillWithBell(uint32_t* array, size_t size, double avg, double dev, double amp, double offset)
{

    for (int i = 0; i < size; i++)
    {
        array[i] = amp*exp(-((i - avg)*(i - avg)/(2*dev*dev))) + offset;
        if (array[i] > 4095)
        {
        	array[i] = 4095;
        }
        if (array[i] < 0)
        {
        	array[i] = 0;
        }
    }
}


