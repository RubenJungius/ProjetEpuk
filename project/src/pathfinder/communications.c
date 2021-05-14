#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include <chprintf.h>
#include "hal.h"
#include "main.h"
#include "communications.h"

void serial_start() {
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void sendInt16ToComputer(int16_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(int16_t) * size);
}
