/*
 * wavegen.h
 *
 *  Created on: Jul 20, 2023
 *      Author: KMarton
 */

#ifndef INC_WAVEGEN_H_
#define INC_WAVEGEN_H_
#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

void fillWithBell(uint32_t* array, size_t size, double avg, double dev, double amp, double offset);
void parseUserInput();
bool startBurst();


#endif /* INC_WAVEGEN_H_ */
