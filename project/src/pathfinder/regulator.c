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
#include "floatmath.h"
#include "constants.h"

#include "regulator.h"
#include "calibration.h"
#include <leds.h>
#include <motors.h>
#include "measurements.h"
#include "process_mic.h"

#include "sensors/proximity.h"


#define PERIOD_REGULATOR	 0.05 // sec
#define R_ROT_ROB_MIN	 (DIAM_ROBOT/2)
#define MAX_ANGLE_ROT	 (float)(MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR) / (float)(R_ROT_ROB_MIN + (DIAM_ROBOT/2))
#define DIST_DETECTION	   MEASUREMENT_NUMBER + 2 // mm
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR // mm
#define OFFSET 		20 // mm, distance to the wall we want the robot to stabilize

#define KP 		(float)(MAX_ANGLE_ROT) / (float)(DIST_DETECTION)
#define	KI		0.0
#define	KD		0 //0.01



void dist_positioning(uint16_t frontDist) {
	uint16_t speed = speed_conversion(MOTOR_SPEED_LIMIT_MARGIN_MM_S);
	// move forward
	if (fixed_to_int32(get_distance(get_prox(0))) > frontDist) {
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		set_led(LED1, 1);
		while (fixed_to_int32(get_distance(get_prox(0))) >= frontDist) {}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_led(LED1, 0);
	}
	// move backward
	else {
		left_motor_set_speed(-speed);
		right_motor_set_speed(-speed);
		set_led(LED5, 1);
		while (fixed_to_int32(get_distance(get_prox(0))) <= frontDist) {}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_led(LED5, 0);
	}
}

void angle_positioning(fixed_point angle) {
	fixed_point temp = fix_mult(int32_to_fixed(DIAM_ROBOT/2), angle);
	fixed_point T = fix_div(temp, float_to_fixed(MOTOR_SPEED_LIMIT_MARGIN_MM_S));
			//(float)((DIAM_ROBOT/2) * angle) / ((float)MOTOR_SPEED_LIMIT_MARGIN_MM_S);
	uint16_t speed = speed_conversion(MOTOR_SPEED_LIMIT_MARGIN_MM_S);
	left_motor_set_speed(-speed);
	right_motor_set_speed(speed);
	chThdSleepMilliseconds(/*fixed_to_int32(T) */ 1000);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}



static THD_WORKING_AREA(regulation_thd_wa, 256);
static THD_FUNCTION(regulation_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// initial conditions
	fixed_point angleSum = 0;
	fixed_point full_circle = float_to_fixed(2*M_PI);
	fixed_point pOld = float_to_fixed(- DIST_DETECTION + OFFSET);
	fixed_point integral = 0;


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
			double tmp = angleSum;
			angleSum += regulation(&pOld, &integral);
			chprintf((BaseSequentialStream *)&SD3, "angleSum : %d", /*regulation(&pOld, &integral));/*/angleSum);
			chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
			//angleSum = tmp;
			chMtxUnlock(get_mutex());
		}
		chThdSleepMilliseconds(PERIOD_REGULATOR * 1000);
	}
}

void regulation_start() {
	chThdCreateStatic(regulation_thd_wa, sizeof(regulation_thd_wa), NORMALPRIO+1, regulation_thd, NULL);
}


/****** PID *******/

int regulation(float* p_pOld, float* p_integral) {

	float pNew = ((float*)get_dist_data())[0] + OFFSET;

	//float alpha = get_alpha();
	fixed_point alpha = float_to_fixed(get_alpha());

	fixed_point alpha = float_to_fixed(get_alpha());
	fixed_point pNew = ((get_dist_data())[0] + int32_to_fixed(OFFSET))*float_to_fixed(cos(fixed_to_float(alpha)));

		// correction if something is seen by the captor 1
		float dist2 = get_distance(get_prox(2));
		float dist1 = get_distance(get_prox(1));
		float correction = 0;
		if(dist1 >= - DIST_DETECTION) {
			correction = dist1 + 35 - sqrt(2)*(35 + dist2);

		}
		//chprintf((BaseSequentialStream *)&SD3, "dist1 : %f, dist2 : %f, correction : %f", dist1, dist2, correction);
		//chprintf((BaseSequentialStream *)&SD3, "\r\n\n");

		// Calculates beta, the objective angle
		//float beta = pid(*p_pOld, pNew, p_integral);
		fixed_point beta = float_to_fixed(pid(*p_pOld, pNew, p_integral));

		fixed_point gama = (beta - alpha) /*- 0.1 * (correction)*/;

		// Security to avoid a too big angle of rotation in one period
		if(gama > MAX_ANGLE_ROT && abs(correction) < 10) {
			gama = MAX_ANGLE_ROT;
		}
		if(gama < -MAX_ANGLE_ROT && abs(correction) < 10) {
			gama = -MAX_ANGLE_ROT;
		}
		int tmp = (int)(gama*256);
		chprintf((BaseSequentialStream *)&SD3, "gama : %f tmp: %d", gama, tmp);
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
		if(fixed_to_float(gama) >= 0) {
			leftSpeed = ratio * rightSpeed;
		}

		else {
			rightSpeed = leftSpeed / ratio;
		}

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);

		// Save current position in a pointer for next iteration
		*p_pOld = pNew;
		return 1;
	}
	return 0;
}

fixed_point pid(fixed_point pOld, fixed_point pNew, fixed_point* p_integral) {

	// Calculates the derivative
	fixed_point deriv = (pNew - pOld)/(PERIOD_REGULATOR);
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


fixed_point speedWheelRatio(fixed_point gama) {

	fixed_point rRotRob = 0;

	if(gama > 0) {
		//rRotRob = ((float)(MAX_DIST_ONE_CYCLE) / gama) - (DIAM_ROBOT/2);
		rRotRob = fix_div(float_to_fixed(MAX_DIST_ONE_CYCLE),gama)-int32_to_fixed(DIAM_ROBOT/2);
	}

	else if(gama < 0) {
		//rRotRob = ((float)(MAX_DIST_ONE_CYCLE) / gama) + (DIAM_ROBOT/2);
		rRotRob = fix_div(float_to_fixed(MAX_DIST_ONE_CYCLE),gama)+int32_to_fixed(DIAM_ROBOT/2);
	}

	// gama == 0
	else {
		return int32_to_fixed(1); // ratio wL/wR = 1 (the robot goes straight forward)
	}
	//return (float)(rRotRob - (DIAM_ROBOT/2)) / (float)(rRotRob + (DIAM_ROBOT/2)); // wL/wR
	return fix_div(rRotRob - int32_to_fixed(DIAM_ROBOT/2), rRotRob + int32_to_fixed(DIAM_ROBOT/2)); // wL/wR
}
