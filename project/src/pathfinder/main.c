#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "floatmath.h"
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include "measurements.h"
#include <chprintf.h>
#include <leds.h>
#include "calibration.h"
#include "constants.h"
#include "regulator.h"
#include "communications.h"
#include "collision_detect.h"


#include <audio/microphone.h>
#include <process_mic.h>
#include "sensors/proximity.h"

#define START_ANGLE	  M_PI/2.25 // Starts with an angle of 90° - 80° = 10° (Robot-Wall angle)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

#ifdef PROXIMITY
	proximity_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
//	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	messagebus_find_topic_blocking(&bus, "/proximity");

	calibrate_ir();
#endif

#ifdef AUDIO
	//launch Audio Thread
	init_counter();
	mic_start(&processAudioData);
#endif


#ifdef DRIVE

	/* Calibration of the proximity captors */
	calibration();

	/* Return to the initial position */
	dist_positioning(MEASUREMENT_NUMBER + 2);

	/* Rotates to the START_ANGLE */
	angle_positioning(START_ANGLE);

	//launch Measurements thread
	measurements_start();

	//launch Measurements thread
	regulation_start();

#endif

#ifdef COLLISION
	collision_detect_start();
#endif

	//chThdSleepSeconds(2);

	/* Infinite loop */
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
