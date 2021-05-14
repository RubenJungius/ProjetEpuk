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
#include "communications.h"
#include "collision_detect.h"

#include <audio/microphone.h>
#include <process_mic.h>
#include "sensors/proximity.h"

messagebus_t bus;
microphone_msg_t microphone_value;
proximity_msg_t prox_values;
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


	proximity_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//	messagebus_find_topic_blocking(&bus, "/proximity");

	calibrate_ir();

#ifdef AUDIO
	//launchAudioThread
	init_counter();
	init_messagebus();
	mic_start(&processAudioData);
	messagebus_topic_t *microphone_topic = messagebus_find_topic_blocking(&bus, "/microphone");
#endif


#ifdef DRIVE
	/* Calibration of the proximity captors */
	calibration();

	/* Return to the initial position */
	dist_positioning(MEASUREMENT_NUMBER );
	angle_positioning((float)M_PI / (float)2);

	//launch thread
	measurements_start();
	regulation_start();
#ifdef COLLISION
	collision_detect_start();
#endif

#endif


	chThdSleepSeconds(2);

	/* Infinite loop. */
    while (1) {
    	messagebus_topic_read(proximity_topic, &prox_values, sizeof(prox_values));
    	chprintf((BaseSequentialStream*)&SD3, "%d, %d\r\n",prox_values.ambient[0], prox_values.ambient[7]);
#ifdef AUDIO_MESSAGE
    	messagebus_topic_read(microphone_topic, &microphone_value, sizeof(microphone_value));
    	if(microphone_value.microphone_value){
    		set_body_led(1);
    		chThdSleepSeconds(1);
    		set_body_led(0);
    	}
#endif
    	chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
