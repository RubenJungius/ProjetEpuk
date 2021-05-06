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
    serial_start();
    //start the USB communication
    usb_start();
    //inits the motors
	motors_init();


	uint16_t tmp[8];

	// Tab of correspondence threw raw proximity values and correspondent distance values
	uint16_t conversionTab[MEASUREMENT_NUMBER][2];



	// constants        !!! faire un define je pense !!!
	uint8_t period = 50; //milliseconds


	proximity_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");

	calibrate_ir();

	/*Calibration of the proximity captors*/
	calibration(conversionTab);
	/* Return to the initial position */
	positioning(conversionTab, 40);
	chThdSleepMilliseconds(1000);
	positioning(conversionTab, 10);
	chThdSleepMilliseconds(1000);
	positioning(conversionTab, conversionTab[MEASUREMENT_NUMBER - 1][0]);
	// past measurements
	uint16_t lastLatSpeed = 0; // pas sur
	//uint16_t* p_lastLatSpeed = &lastLatSpeed;
	uint16_t integral = 0;
	//uint16_t* p_integral = &integral;
	uint16_t pOld = get_distance(conversionTab, get_prox(2));
	//uint16_t* p_pOld = &pOld; //


   	/* Infinite loop. */
    while (1) {
    	/*
    	for(int i = 0; i < 8; i++){
        	tmp[i] = get_distance(conversionTab, get_prox(i));
    	}

    	chprintf((BaseSequentialStream *)&SD3, " FRONT : %d ; ", tmp[0] == 0 ? 0 : tmp[0]);
    	chprintf((BaseSequentialStream *)&SD3, " RIGHT : %d ; ", tmp[2] == 0 ? 0 : tmp[2]);
    	chprintf((BaseSequentialStream *)&SD3, " LEFT : %d ; ", tmp[5] == 0 ? 0 : tmp[5]);

    	//SendUint8ToComputer(tmp, 8);
        chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
        */


        // implementation of the maze algorithm
        regulation(period, 2, &pOld, &lastLatSpeed, &integral, conversionTab);

        chThdSleepMilliseconds(period);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
