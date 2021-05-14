#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <chprintf.h>
#include <leds.h>
#include "calibration.h"
#include "regulator.h"


#include "sensors/proximity.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    //serial_start();
    //start the USB communication
    usb_start();
    //inits the motors
	motors_init();


	proximity_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
//	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	messagebus_find_topic_blocking(&bus, "/proximity");

	calibrate_ir();

	/* Calibration of the proximity captors */
	calibration();

	/* Return to the initial position */
	dist_positioning(MEASUREMENT_NUMBER );

	angle_positioning((float)M_PI / (float)2);

	chThdSleepSeconds(2);

	//launch threads
	measurements_start();
	regulation_start();



   	/* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
