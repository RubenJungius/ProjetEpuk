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

void angle_positioning(fixed_point angle) {
	fixed_point temp = fix_mult(int32_to_fixed(DIAM_ROBOT/2), angle);
	fixed_point T = fix_div(temp, float_to_fixed(MOTOR_SPEED_LIMIT_MARGIN_MM_S));
			//(float)((DIAM_ROBOT/2) * angle) / ((float)MOTOR_SPEED_LIMIT_MARGIN_MM_S);
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
			angleSum += regulation(&pOld, &integral);
			chprintf((BaseSequentialStream *)&SD3, "angleSum : %f", fixed_to_float(full_circle - angleSum));
			chprintf((BaseSequentialStream *)&SD3, "\r\n\n");
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

fixed_point regulation(fixed_point* p_pOld, fixed_point* p_integral) {

	fixed_point alpha = float_to_fixed(get_alpha());
	fixed_point pNew = ((get_dist_data())[0] + int32_to_fixed(OFFSET))*float_to_fixed(cos(fixed_to_float(alpha)));


	// wall approaching algorithm
	if(pNew >= int32_to_fixed(- DIST_DETECTION + OFFSET)){

		fixed_point dist2 = get_distance(get_prox(2));
		fixed_point dist1 = get_distance(get_prox(1));
		fixed_point correction = 0;
		if(dist1 >= int32_to_fixed(- DIST_DETECTION))
			correction = dist1 + int32_to_fixed(35) - float_to_fixed(sqrt(2)*(35 + dist2));

		fixed_point beta = float_to_fixed(pid(*p_pOld, pNew, p_integral));

		fixed_point gama = (beta - alpha) /*- 0.1 * (correction)*/;

		// Security to avoid a too big angle of rotation in one period
		if(fixed_to_float(gama) > MAX_ANGLE_ROT && abs(correction) < 10) {
			gama = float_to_fixed(MAX_ANGLE_ROT);
		}
		if(fixed_to_float(gama) < -MAX_ANGLE_ROT && abs(correction) < 10) {
			gama = float_to_fixed(-MAX_ANGLE_ROT);
		}

		fixed_point ratio = float_to_fixed(speedWheelRatio(gama));

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
		return gama;
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
