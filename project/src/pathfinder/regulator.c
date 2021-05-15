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
#include <chprintf.h>
#include "main.h"

#include "regulator.h"
#include "calibration.h"
#include <leds.h>
#include <motors.h>
#include "measurements.h"
#include "process_mic.h"

#include "sensors/proximity.h"


#define PERIOD_REGULATOR	 0.2 // sec
#define R_ROT_ROB_MIN	 2 * (DIAM_ROBOT/2)
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR // mm
#define MAX_ANGLE_ROT	 (float)(MAX_DIST_ONE_CYCLE) / (float)(R_ROT_ROB_MIN)
#define DIST_DETECTION	   MEASUREMENT_NUMBER + 2 // mm
#define OFFSET 		20 // mm, distance to the wall we want the robot to stabilize

#define KP 		(float)(MAX_ANGLE_ROT) / (float)(DIST_DETECTION)
#define	KI		0.0000
#define	KD		0 //0.01



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



static THD_WORKING_AREA(regulation_thd_wa, 256);
static THD_FUNCTION(regulation_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// initial conditions
	static int angleSum = 0;
	float pOld = - DIST_DETECTION + OFFSET;
	float integral = 0;


#ifdef AUDIO
	int status = 0;
#else
	int status = 1;
#endif

	while(1) {
		if(!status){
			chMtxLock(mic_get_mutex());
			chCondWait(mic_get_condition());
			status = return_status();
			chprintf((BaseSequentialStream *)&SD3, "Staus Regulateur: %d", status);
			chMtxUnlock(mic_get_mutex());
		}else{
			chMtxLock(get_mutex());
			chCondWait(get_condition());
			regulation(&pOld, &integral);
			//angleSum = tmp;
			chMtxUnlock(get_mutex());
			set_led(LED1, 1);
		}
		chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);
		set_led(LED1, 0);
	}
}

void regulation_start() {
	chThdCreateStatic(regulation_thd_wa, sizeof(regulation_thd_wa), NORMALPRIO+1, regulation_thd, NULL);
}


/****** PID *******/

regulation(float* p_pOld, float* p_integral) {

	float pNew = ((float*)get_dist_data())[0] + OFFSET;
	float alpha = get_alpha();

	// wall approaching algorithm
	if(pNew >= - DIST_DETECTION + OFFSET){

		// correction if something is seen by the captor 1
		float dist2 = get_distance(get_prox(2));
		float dist1 = get_distance(get_prox(1));
		float correction = 0;
		if(dist1 <= DIST_DETECTION) {
			correction = dist1 + 35 - sqrt(2)*(35 + dist2);
			set_led(LED1, 1);
		}
		/*chprintf((BaseSequentialStream *)&SD3, "correction : %f", correction);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");

*/
		chprintf((BaseSequentialStream *)&SD3, "pnew : %f", pNew);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
		// Calculates beta, the objective angle
		float beta = pid(*p_pOld, pNew, p_integral);

		// Calculates the angle we want the robot to rotate
		float gama = (beta - alpha) /*- 0.1 * (correction)*/;

		// Security to avoid a too big angle of rotation in one period
		if(gama > MAX_ANGLE_ROT /*&& correction <= -30*/) {
			gama = MAX_ANGLE_ROT;
		}
		if(gama < -MAX_ANGLE_ROT /*&& correction > 30*/) {
			gama = -MAX_ANGLE_ROT;
		}
		int tmp = (int)(gama*256);

		chprintf((BaseSequentialStream *)&SD3, "beta : %f, alpha: %f, gama: %f", beta, alpha, gama);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");



		// Find the ratio between the speed of the wheels
		float ratio = speedWheelRatio(gama);

		if(ratio > 1) {
			set_led(LED7, 1);
			set_led(LED3, 0);
		}
		if(ratio <= 1) {
			set_led(LED7, 0);
			set_led(LED3, 1);
		}

		// Give the commands to the motor
		int16_t rightSpeed = MOTOR_SPEED_LIMIT_MARGIN;
		int16_t leftSpeed = MOTOR_SPEED_LIMIT_MARGIN;
		if(gama >= 0) {
			leftSpeed = ratio * rightSpeed;
		}

		else {
			rightSpeed = leftSpeed / ratio;
		}

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);
		// Save current position in a pointer for next iteration
		*p_pOld = pNew;
	}
}

float pid(float pOld, float pNew, float* p_integral) {

	// Calculates the derivative
	float deriv = (pNew - pOld)/(PERIOD_REGULATOR);
	/*
	chprintf((BaseSequentialStream *)&SD3, "deriv : %f", deriv);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/

	// Calculates the integral and save the value in a pointer for next iteration
	*p_integral += pNew;
	/*chprintf((BaseSequentialStream *)&SD3, "integral : %f", *p_integral);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/


	// Calculates beta, the objective angle
	return ((KP * pNew) + (KI * (*p_integral)) + (KD * deriv));
}


float speedWheelRatio(float gama) {

	int16_t rRotRob = 1000; // *** a changer c'est moche !!!

	if(gama > 0) {
		rRotRob = ((float)(MAX_DIST_ONE_CYCLE) / gama) - (DIAM_ROBOT/2);
	}

	else if(gama < 0) {
		rRotRob = ((float)(MAX_DIST_ONE_CYCLE) / gama) + (DIAM_ROBOT/2);
	}

	// gama == 0
	else {
		return 1; // ratio wL/wR = 1 (the robot goes straight forward)
	}

	return (float)(rRotRob - (DIAM_ROBOT/2)) / (float)(rRotRob + (DIAM_ROBOT/2)); // wL/wR
}

