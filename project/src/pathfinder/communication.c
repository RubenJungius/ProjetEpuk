/*
 * communication.c
 *
 *  Created on: 09.05.2021
 *      Author: Ruben
 */

#include "communications.h"
#include "ch.h"
#include "hal.h"

void SendUint8ToComputer(uint8_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

void SendInt16ToComputer(int16_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(int16_t) * size);
}

/*
chSequentialStreamWrite(out, (uint8_t*)"START", 5);
chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
*/
