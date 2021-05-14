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

#include "regulator.h"
#include "calibration.h"
#include <leds.h>
#include <motors.h>
#include "measurements.h"

#include "sensors/proximity.h"


#define PERIOD_REGULATOR	 0.24 // sec
#define R_ROT_ROB_MIN	 2 * (DIAM_ROBOT/2)
#define MAX_ANGLE_ROT	 (float)(MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR) / (float)(R_ROT_ROB_MIN + (DIAM_ROBOT/2))
#define DIST_DETECTION	   MEASUREMENT_NUMBER + 2 // mm // à changer
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR // mm
#define OFFSET 		10 // mm, distance to the wall we want the robot to stabilize

#define KP 		20.0 * (float)(MAX_ANGLE_ROT) / (float)(DIST_DETECTION)
#define	KI		0
#define	KD		0 //0.01



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


static THD_WORKING_AREA(regulation_thd_wa, 256); ///256????
static THD_FUNCTION(regulation_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// initial conditions
	int16_t integral = 0;
	float pOld = - DIST_DETECTION + OFFSET;
	//float alphaNew = 0;
	//uint8_t firstDetection = 1;
	//left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	//right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	//chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);

	//chMtxObjectInit(get_mutex());


	while(1) {
		chMtxLock(get_mutex());
		chCondWait(get_condition());
		regulation(/*2, */&pOld,/* &alphaNew,*/ &integral/*, &firstDetection*/);
		chMtxUnlock(get_mutex());
		//chThdYield();
		chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);
	}
}

void regulation_start() {
	serial_start();
	chThdCreateStatic(regulation_thd_wa, sizeof(regulation_thd_wa), NORMALPRIO+1, regulation_thd, NULL);
}














/****** PID *******/


void regulation(/*uint8_t captorNumber, */float* p_pOld,/* float* p_alphaNew,*/ int16_t* p_integral/*, uint8_t* p_firstDetection*/) {
	// Get the current lateral position

	float pNew = ((float*)get_dist_data())[0] + OFFSET;
	// float pOld = ((float*)get_dist_data())[3] + OFFSET;
	float alpha = get_alpha();

	chprintf((BaseSequentialStream *)&SD3, "p0 : %f", ((float*)get_dist_data())[0]);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
	chprintf((BaseSequentialStream *)&SD3, "p1 : %f", ((float*)get_dist_data())[1]);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
	chprintf((BaseSequentialStream *)&SD3, "p2 : %f", ((float*)get_dist_data())[2]);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
	chprintf((BaseSequentialStream *)&SD3, "p3 : %f", ((float*)get_dist_data())[3]);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");



	/*
	chprintf((BaseSequentialStream *)&SD3, "firstDetection : %d", *p_firstDetection);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/

	// If first detection of a wall at its right, calculates the initial alpha, the angle between the robot current direction and the wall
	/*if(pNew >= - DIST_DETECTION && *p_firstDetection){
	 *p_alphaNew = atan((*p_pOld - pNew)/(MAX_DIST_ONE_CYCLE));
	 *p_firstDetection = 0;
	 *p_pOld = pNew;
		set_led(LED5, 0);
	}

	// If there is no longer a wall close enough at the right of the robot
	else if(pNew < - DIST_DETECTION && !*p_firstDetection){
	 *p_pOld = - DIST_DETECTION + OFFSET;
	 *p_alphaNew = 0;
	 *p_firstDetection = 1;
		left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
		right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	}
	 */
	if(pNew >= - DIST_DETECTION + OFFSET /*&& !*p_firstDetection*/){
		// wall approaching algorithm
		// Calculates beta, the objective angle
		float beta = pid(*p_pOld, pNew, p_integral);

		chprintf((BaseSequentialStream *)&SD3, "beta : %f", beta);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
		chprintf((BaseSequentialStream *)&SD3, "alpha : %f", alpha);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
		chprintf((BaseSequentialStream *)&SD3, "pNew : %f", pNew);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");

		// Calculates the angle we want the robot to rotate
		float gama = beta - alpha;

		// Security to avoid a too big angle of rotation in one period
		if(gama > MAX_ANGLE_ROT) {
			gama = MAX_ANGLE_ROT;
		}
		if(gama < -MAX_ANGLE_ROT) {
			gama = -MAX_ANGLE_ROT;
		}
		chprintf((BaseSequentialStream *)&SD3, "gama : %f", gama);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");

		// Find the ratio between the speed of the wheels
		float ratio = speedWheelRatio(gama);
		/*
		chprintf((BaseSequentialStream *)&SD3, "ratio : %f", ratio);
		chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/
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
		*p_pOld = pNew; // Peut-être qu'il faut changer ça

		// Save the future angle alpha (robot - wall)
		//*p_alphaNew = *p_alphaNew + gama;
	}
	/*else {
	 *p_firstDetection = 1;
		set_led(LED5, 1);
		left_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
		right_motor_set_speed(MOTOR_SPEED_LIMIT_MARGIN);
	}*/
}

float pid(float pOld, float pNew, int16_t* p_integral) {

	// Calculates the derivative
	float deriv = (pNew - pOld)/(PERIOD_REGULATOR);
	/*chprintf((BaseSequentialStream *)&SD3, "deriv : %d", deriv);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/

	// Calculates the integral and save the value in a pointer for next iteration
	*p_integral = *p_integral + pNew;
	/*chprintf((BaseSequentialStream *)&SD3, "integral : %d", *p_integral);
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
	/*chprintf((BaseSequentialStream *)&SD3, "rRotRob : %d", rRotRob);
	chprintf((BaseSequentialStream *)&SD3, "\r\n\n");*/

	return (float)(rRotRob - (DIAM_ROBOT/2)) / (float)(rRotRob + (DIAM_ROBOT/2)); // wL/wR
}

