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
//#include "pthread.h"
//#include "unistd.h"
//#include "chmtx.h"
//#include "chconf.h"

#include "calibration.h"
#include <leds.h>
#include <motors.h>
#include "measurements.h"
#include "regulator.h"

#include "sensors/proximity.h"


#define PERIOD_MEASUREMENTS 0.5 //sec
#define PERIOD_MEASUREMENT 0.1 //sec
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_MEASUREMENT // mm

int16_t pastMeasurements[RECORDED_MEASUREMENTS_NUMBER];
float alpha = 0;

static mutex_t mutex;
static condition_variable_t dataProduced;


static THD_WORKING_AREA(measurements_thd_wa, 256); ///256????
static THD_FUNCTION(measurements_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// initial conditions
	//int16_t integral = 0;
	//int16_t pOld = - DIST_DETECTION + OFFSET;

	//uint8_t firstDetection = 1;

	chMtxObjectInit(&mutex);
	chCondObjectInit(&dataProduced);

	while(1){
		chMtxLock(&mutex);
		measurements(2, &alpha);
		chCondSignal(&dataProduced);
		chMtxUnlock(&mutex);
		chThdSleepMilliseconds(PERIOD_MEASUREMENTS * 1000);
	}
}

void measurements_start(){
	chThdCreateStatic(measurements_thd_wa, sizeof(measurements_thd_wa), NORMALPRIO+2, measurements_thd, NULL);
}

void measurements(uint8_t captorNumber, float* p_alpha) {

	left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	for(int8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i >= 0 ; i--) {
		pastMeasurements[i] = - get_distance(get_prox(captorNumber));
		chThdSleepMilliseconds(PERIOD_MEASUREMENT * 1000);
	}
	find_alpha(p_alpha);
}

void find_alpha(float* p_alpha) {
	float angle[RECORDED_MEASUREMENTS_NUMBER - 1];
	for(uint8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i > 0 ; i--) {
		angle[i] = (float)(asin((pastMeasurements[i] - pastMeasurements[i - 1]))/(float)(MAX_DIST_ONE_CYCLE));
		*p_alpha += angle[i];
	}
	*p_alpha = *p_alpha / (float)3;
}

float get_alpha() {
	return alpha;
}

int16_t* get_dist_data() {
	return pastMeasurements;
}

mutex_t* get_mutex() {
	return &mutex;
}

condition_variable_t* get_condition() {
	return &dataProduced;
}