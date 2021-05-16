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
#include "regulator.h"
#include "process_mic.h"

#include "sensors/proximity.h"


#define PERIOD_MEASUREMENTS 0.2 //sec
#define PERIOD_MEASUREMENT 0.025 //sec
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_MEASUREMENT // mm

float pastMeasurements[RECORDED_MEASUREMENTS_NUMBER];
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
#ifdef AUDIO
	int status = 0;
#else
	int status = 1;
#endif
	chThdSleepMilliseconds(400);
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

			if(/*tmp*/!regulator_return_status()){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				chThdSleepSeconds(5);
			}
		}
		chThdSleepMilliseconds(PERIOD_MEASUREMENTS * 1000);
	}
}

void measurements_start(){
	//serial_start();
	chThdCreateStatic(measurements_thd_wa, sizeof(measurements_thd_wa), NORMALPRIO+2, measurements_thd, NULL);
}

void measurements(uint8_t captorNumber, float* p_alpha) {

	left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	for(int8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i >= 0 ; i--) {
		pastMeasurements[i] = - get_distance(get_prox(captorNumber));
		chThdSleepMilliseconds(PERIOD_MEASUREMENT * 1000);
		/*chprintf((BaseSequentialStream *)&SD3, "pos %d : %f", i, pastMeasurements[i]);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/
	}
	find_alpha(p_alpha);
}

void find_alpha(float* p_alpha) {
	float angle[RECORDED_MEASUREMENTS_NUMBER - 2];
	float angleSum = 0;
	for(uint8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i > 1 ; i--) {
		angle[i] = atan((pastMeasurements[i] - pastMeasurements[i - 2])/(float)(2*MAX_DIST_ONE_CYCLE));
		angleSum += angle[i];
		/*chprintf((BaseSequentialStream *)&SD3, "angle %d : %f", i, angle[i]);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/
	}
	*p_alpha = angleSum / (float)2;
	/*chprintf((BaseSequentialStream *)&SD3, "angle %f", *p_alpha);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/
}

float get_alpha() {
	return alpha;
}

float* get_dist_data() {
	return pastMeasurements;
}

mutex_t* get_mutex() {
	return &mutex;
}

condition_variable_t* get_condition() {
	return &dataProduced;
}
