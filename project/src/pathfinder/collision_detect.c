#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"

#include "calibration.h"
#include <leds.h>

#include "sensors/proximity.h"

static THD_WORKING_AREA(collision_thd_wa, 256);
static THD_FUNCTION(collision_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while(1){
		if(get_distance(get_prox(7))<MIN_COL_DISTANCE)
			set_body_led(1);
		else
			set_body_led(0);
		chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);
	}
}

void collision_detect_start() {
	chThdCreateStatic(collision_thd_wa, sizeof(collision_thd_wa), NORMALPRIO+1, collision_thd, NULL);
}
