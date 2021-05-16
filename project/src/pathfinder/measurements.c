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
#include "constants.h"
#include <leds.h>
#include <motors.h>
#include "measurements.h"
#include "regulator.h"
#include "process_mic.h"

#include "sensors/proximity.h"


			/***** DEFINE *****/


//#define PERIOD_MEASUREMENTS 0.1 //sec
#define PERIOD_MEASUREMENT 0.0125 //sec
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_MEASUREMENT // mm
#define RADIUS_ROBOT 35 //mm

float pastMeasurements[RECORDED_MEASUREMENTS_NUMBER];
float alpha = 0;


			/***** LOCAL FUNCTIONS *****/


// Find the (average) angle between the robot current direction and a wall.
void find_alpha(float* p_alpha) {
	float angle[RECORDED_MEASUREMENTS_NUMBER - 2];
	float angleSum = 0;
	for(uint8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i > 1 ; i--) {
		angle[i] = atan((pastMeasurements[i] - pastMeasurements[i - 2])/(float)(2*MAX_DIST_ONE_CYCLE));
		angleSum += angle[i];
	}
	*p_alpha = angleSum / (float)2;
}

/* 	Store in a tab a sampling of distance from a captor taken with a frequency = 1/PERIOD_MEASUREMENT.
 	Based on these values, finds alpha the angle between the robot x-axis and the wall and store it in a pointer.
	During this process, the robot goes straight forward. */
void measurements(uint8_t captorNumber, float* p_alpha) {

	left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	for(int8_t i = RECORDED_MEASUREMENTS_NUMBER - 1 ; i >= 0 ; i--) {
		pastMeasurements[i] = - ((get_distance(get_prox(captorNumber)) - RADIUS_ROBOT) * cos(alpha) + RADIUS_ROBOT);
		chThdSleepMilliseconds(PERIOD_MEASUREMENT * 1000);
	}
	find_alpha(p_alpha);
}

			/***** MUTEX *****/

static mutex_t mutex;
static condition_variable_t dataProduced;



			/***** THREAD OF THE MEASUREMENTS *****/

static THD_WORKING_AREA(measurements_thd_wa, 256);
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
	chThdSleepMilliseconds(400);
	while(1){
		if(!status){
			chMtxLock(mic_get_mutex());
			chCondWait(mic_get_condition());
			status = return_status();
			chMtxUnlock(mic_get_mutex());
		}else{

			chMtxLock(&mutex);
			measurements(2, &alpha);
			chCondSignal(&dataProduced);
			chMtxUnlock(&mutex);

			if(!regulator_return_status()){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				chThdSleepSeconds(1);
				alpha = 0;
				status = 0;
			}
		}
		chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);
	}
}


			/***** GLOBAL FUNCTIONS *****/


void measurements_start(){
	chThdCreateStatic(measurements_thd_wa, sizeof(measurements_thd_wa), NORMALPRIO+2, measurements_thd, NULL);
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
