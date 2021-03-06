/*
 * regulator.c
 *
 *  Created on: 1 mai 2021
 *      Author: Luca
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "floatmath.h"
#include <chprintf.h>


#include "regulator.h"
#include "calibration.h"
#include "constants.h"
#include <leds.h>
#include <motors.h>
#include "measurements.h"
#include "process_mic.h"

#include "sensors/proximity.h"


				/***** DEFINE *****/

//#define PERIOD_REGULATOR	 0.1 // sec
#define SAFETY_DELAY 400 //msec
#define R_ROT_ROB_MIN	 2 * (DIAM_ROBOT/2) // mm
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR // mm
#define MAX_ANGLE_ROT	 (float)(MAX_DIST_ONE_CYCLE) / (float)(R_ROT_ROB_MIN + (DIAM_ROBOT/2)) // rad
#define DIST_DETECTION	   MEASUREMENT_NUMBER + 2 // mm
#define OFFSET 		25 // mm, distance to the wall we want the robot to stabilize

#define KP 		(float)(MAX_ANGLE_ROT) / (float)(DIST_DETECTION-OFFSET)
#define	KI		0  // useless in our case
#define	KD		0  // useless in our case


				/***** LOCAL VARIABLES *****/

static int run_status;
static int count;
static mutex_t regulator_mutex;
static condition_variable_t regulator_cond;



				/***** LOCAL FUNCTIONS *****/


/*** Positioning Functions ***/

void dist_positioning(uint16_t frontDist) {

	uint16_t speed = speed_conversion(MOTOR_SPEED_LIMIT_MARGIN_MM_S);
	// move forward
	if (get_distance(get_prox(0)) > frontDist) {
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		set_led(LED1, 1);
		while (get_distance(get_prox(0)) >= frontDist) {}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_led(LED1, 0);
	}
	// move backward
	else {
		left_motor_set_speed(-speed);
		right_motor_set_speed(-speed);
		set_led(LED5, 1);
		while (get_distance(get_prox(0)) <= frontDist) {}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_led(LED5, 0);
	}
}

void angle_positioning(float angle) {
	float T = (float)((DIAM_ROBOT/2) * angle) / ((float)MOTOR_SPEED_LIMIT_MARGIN_MM_S);
	uint16_t speed = speed_conversion(MOTOR_SPEED_LIMIT_MARGIN_MM_S);
	left_motor_set_speed(-speed);
	right_motor_set_speed(speed);
	chThdSleepMilliseconds(T * 1000);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


/*** Regulator Functions ***/

/* Output the ratio between the speed of the wheels that is needed in order to accomplish an angular rotation of gama in a PERIOD_REGULATOR. */
float speedWheelRatio(float gama) {

	int16_t rRotRob = 0;

	if(gama > 0) {
		rRotRob = ((float)(MAX_DIST_ONE_CYCLE) / gama) - (DIAM_ROBOT/2);
	}

	else if(gama < 0) {
		rRotRob = ((float)(MAX_DIST_ONE_CYCLE) / gama) + (DIAM_ROBOT/2);
	}

	// gama == 0, ratio wL/wR = 1 (the robot goes straight forward)
	else {
		return 1;
	}

	return (float)(rRotRob - (DIAM_ROBOT/2)) / (float)(rRotRob + (DIAM_ROBOT/2)); // wL/wR
}

/* It calculates the "objective angle" (beta) based on a Proportional term, an Integral term and a Derivative term. */
float pid(float yOld, float y, float* p_integral) {

	// Calculates the derivative
	float deriv = (y - yOld)/(PERIOD_REGULATOR);

	// Calculates the integral and save the value in a pointer for next iteration
	*p_integral += y;

	// Calculates beta, the objective angle
	return ((KP * y) + (KI * (*p_integral)) + (KD * deriv));
}

/* It calculates the corrected angle (output) and adjust it with a wheel speed ratio algorithm. */
fixed_point regulation(float* p_yOld, float* p_integral) {

	float alpha = get_alpha();
	float y = (get_dist_data())[0] + OFFSET;

	// wall approaching algorithm
	if(y >= - DIST_DETECTION + OFFSET){

		// is there a big curve ?
		float dist1 = get_distance(get_prox(1)); // distance seen by the captor on the diagonal
		uint8_t big_curve = 0;
		if(dist1 < OFFSET) {
			big_curve = 1;
		}

		// Calculates beta, the objective angle
		float beta = pid(*p_yOld, y, p_integral);

		// Calculates the angle we want the robot to rotate
		float gama = (beta - alpha);

		// Security to avoid a too big angle of rotation in one period
		if(gama > MAX_ANGLE_ROT) {
			gama = MAX_ANGLE_ROT;
		}
		if(gama < -MAX_ANGLE_ROT) {
			gama = -MAX_ANGLE_ROT;
		}

		// Find the ratio between the speeds of the wheels
		float ratio = speedWheelRatio(gama);

		// Give the commands to the motor
		int16_t rightSpeed = MOTOR_SPEED_LIMIT_MARGIN;
		int16_t leftSpeed = MOTOR_SPEED_LIMIT_MARGIN;

		if(gama >= 0 && !big_curve) {
			leftSpeed = ratio * rightSpeed;
		}

		else if(gama < 0 && !big_curve) {
			rightSpeed = leftSpeed / ratio;
		}

		// there is a big curve ! the robot need to stop and rotate on its axis => R_ROT_ROB_MIN = 0
		else {
			// reduce a bit the speed during the maneuver
			rightSpeed = MOTOR_SPEED_LIMIT_MARGIN / 2;
			leftSpeed = - MOTOR_SPEED_LIMIT_MARGIN / 2;

			gama = (float)((MOTOR_SPEED_LIMIT_MARGIN_RAD_S / 2) * RADIUS_WHEEL * PERIOD_REGULATOR) / (float)(DIAM_ROBOT / 2);
		}

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);

		// Save current position in a pointer for next iteration
		*p_yOld = y;


		return float_to_fixed(gama);
	}
	// game = 0 (the robots is far from the wall and goes straight forward)
	return 0;
}


			/***** MUTEX *****/

mutex_t* regulator_get_mutex() {
	return &regulator_mutex;
}

condition_variable_t* regulator_get_condition() {
	return &regulator_cond;
}



			/***** THREAD OF THE REGULATOR *****/

static THD_WORKING_AREA(regulation_thd_wa, 256);
static THD_FUNCTION(regulation_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// initial conditions
	fixed_point angleSum = 0;
	fixed_point quarter_circle = float_to_fixed(2*M_PI/4);
	float yOld = - DIST_DETECTION + OFFSET;
	float integral = 0;
	run_status = 1;
	count = 0;

#ifdef AUDIO
	int status = 0;
#else
	int status = 1;
#endif
	chThdSleepMilliseconds(SAFETY_DELAY);
	while(1) {
		if(!status){
			chMtxLock(mic_get_mutex());
			chCondWait(mic_get_condition());
			status = return_status();
			chMtxUnlock(mic_get_mutex());
		}else{
			chMtxLock(get_mutex());
			chCondWait(get_condition());
			angleSum += regulation(&yOld, &integral);
			chMtxUnlock(get_mutex());
			if(angleSum >= quarter_circle) {
				count++;
				set_body_led(count%2);
				angleSum = 0;
				run_status = 0;
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				chThdSleepSeconds(1);
				init_counter();//reset audio so that it can listen to new sequence
				status = 0;
				run_status = 1;
				yOld = - DIST_DETECTION + OFFSET;
				integral = 0;
			}
		}
		chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);
	}
}


				/***** GLOBAL FUNCTIONS *****/


void regulation_start() {
	chThdCreateStatic(regulation_thd_wa, sizeof(regulation_thd_wa), NORMALPRIO+1, regulation_thd, NULL);
}

int regulator_return_status(void){
	return run_status;
}

int regulator_return_count(void){
	return count;
}
