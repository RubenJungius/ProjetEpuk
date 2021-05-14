/*
 * process_mic.c
 *
 *  Created on: 09.05.2021
 *      Author: Ruben
 */

#include "process_mic.h"
#include "ch.h"
#include "communications.h"

int launch_status = 0;

void processAudioData(int16_t *data, uint16_t num_samples){
	SendInt16ToComputer(data, num_samples);
}
