#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"

#include "collision_detect.h"
#include "floatmath.h"
#include "calibration.h"
#include <leds.h>
#include <motors.h>

#include "sensors/proximity.h"

static THD_WORKING_AREA(collision_thd_wa, 256);
static THD_FUNCTION(collision_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while(1){
		if(get_distance(get_prox(7)) < MIN_COL_DISTANCE){
			set_body_led(1);
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}else
			set_body_led(0);
		chThdSleepMilliseconds(10);
	}
}

void collision_detect_start() {
	chThdCreateStatic(collision_thd_wa, sizeof(collision_thd_wa), NORMALPRIO+1, collision_thd, NULL);
}
