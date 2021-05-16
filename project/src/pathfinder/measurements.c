/*
 * measurements.c
 *
 *  Created on: 13 mai 2021
 *      Author: Luca
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include <chprintf.h>
#include "floatmath.h"

#include "calibration.h"
#include <leds.h>
#include "main.h"
#include <motors.h>
#include "measurements.h"
//#include "regulator.h"
#include "constants.h"
#include "process_mic.h"

#include "sensors/proximity.h"

static fixed_point pastMeasurements[RECORDED_MEASUREMENTS_NUMBER];
static fixed_point alpha = 0;

static mutex_t mutex;
static condition_variable_t dataProduced;


static THD_WORKING_AREA(measurements_thd_wa, 256); ///256????
static THD_FUNCTION(measurements_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);


	chMtxObjectInit(&mutex);
	chCondObjectInit(&dataProduced);
#ifdef AUDIO
	int status = 0;
#else
	int status = 1;
#endif

	while(1){
		if(!status){
			chMtxLock(mic_get_mutex());
			chCondWait(mic_get_condition());
			status = return_status();
			chprintf((BaseSequentialStream*)&SD3, "Staus Measurement: %d", status);
			chMtxUnlock(mic_get_mutex());
		}else{
			chMtxLock(&mutex);
			measurements(2, &alpha);
			chCondSignal(&dataProduced);
			chMtxUnlock(&mutex);
		}
		chThdSleepMilliseconds(PERIOD_MEASUREMENTS * 1000);
	}
}

void measurements_start(){
	//serial_start();
	chThdCreateStatic(measurements_thd_wa, sizeof(measurements_thd_wa), NORMALPRIO+2, measurements_thd, NULL);
}

void measurements(uint8_t captorNumber, fixed_point* p_alpha) {

	left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	for(int8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i >= 0 ; i--) {
		pastMeasurements[i] = - get_distance(get_prox(captorNumber));
		chThdSleepMilliseconds(PERIOD_MEASUREMENT * 1000);
	}
	find_alpha(p_alpha);
}

void find_alpha(fixed_point* p_alpha) {
	fixed_point angle[RECORDED_MEASUREMENTS_NUMBER - 2];
	fixed_point angleSum = 0;
	for(uint8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i > 1 ; i--) {
		angle[i] = asin((pastMeasurements[i] - pastMeasurements[i - 2])/(float)(MAX_DIST_ONE_CYCLE));
		angleSum += angle[i];
	}
	*p_alpha = fix_div(angleSum, int32_to_fixed(2));
}

fixed_point get_alpha() {
	return alpha;
}

fixed_point* get_dist_data() {
	return pastMeasurements;
}

mutex_t* get_mutex() {
	return &mutex;
}

condition_variable_t* get_condition() {
	return &dataProduced;
}
