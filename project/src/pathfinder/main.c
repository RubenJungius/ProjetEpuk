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

	uint8_t tmp[8];
	int light = 0;

	proximity_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");

	calibrate_ir();

   	/* Infinite loop. */
    while (1) {
    	//waits 1 second
    	for(int i = 0; i < 8; i++){
        	tmp[i]=get_prox(i);
        	//chprintf((BaseSequentialStream *)&SD3, "%d: %d; ",i, tmp[i]);
    	}
       	SendUint8ToComputer(tmp, 8);
        //chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
       	//left_motor_set_speed(500);
       	//right_motor_set_speed(500);

       	for(int i = 0; i < 8; i++){
        	if(tmp[i]>100){
        		//set_body_led(2);
        		//light = 1;
        		left_motor_set_speed(0);
        		right_motor_set_speed(0);
        		break;
        	}
        }
        /*if(light){
        	set_body_led(1);
        	light = 0;
        }*/
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
