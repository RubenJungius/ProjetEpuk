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

#include "regulator.h"
#include "calibration.h"
#include <leds.h>
#include <motors.h>

#include "sensors/proximity.h"



void positioning(uint16_t frontDist) {
	uint16_t speed = speed_conversion(MAX_SPEED_MM_S / 3); // attention magic number !!!
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

int16_t pid(uint8_t captorNumber, uint16_t period, uint16_t* p_pOld, int16_t* p_lastLatSpeed, int16_t* p_integral) {

	// Get the current position
	uint16_t pNew = get_distance(get_prox(captorNumber));

	// Get the current lateral speed
	int16_t currentLatSpeed = derivative(*p_pOld, pNew, period); // the error (we want it to be 0) in mm/s
	/*if(currentLatSpeed > 0) {
			set_led(LED3, 0);
			set_led(LED7, 1);
		}
		else if(currentLatSpeed < 0) {
			set_led(LED7, 0);
			set_led(LED3, 1);
		}*/

	// Save current position in a pointer for next iteration
	*p_pOld = pNew;

	// Calculate the integral and save the value in a pointer for next iteration
	*p_integral = *p_integral + currentLatSpeed;

	// Calculate the derivative
	int16_t deriv = currentLatSpeed - *p_lastLatSpeed;

	// Save current lateral speed in a pointer for next iteration
	*p_lastLatSpeed = currentLatSpeed;

	// Calculate the Control Variable
	int16_t c = (KP * currentLatSpeed) + (KI * (*p_integral)) + (KD * deriv);

	return c;
}


float speedWheelRatio(int16_t controlVar) {
	float rRotRob = 1000; // a changer absolument !!!!

	if(controlVar == 0) {
		set_led(LED7, 0);
		set_led(LED3, 1);
		return 1;
	}
	if(controlVar != 0) {
		rRotRob = ((float)ALPHA / (float)controlVar);
		set_led(LED7, 1);
		set_led(LED3, 0);
	}// rRotRob is inverse proportional to the control variable

	return -(double)(rRotRob + (DIAM_ROBOT/2)) / (double)((DIAM_ROBOT/2) - rRotRob);
}


float derivative(uint16_t pOld, uint16_t pNew, uint16_t T) {
	float d = (float)(pNew - pOld) / (T/(double)1000) ; //   ATTENTION TO CHANGE
	return d;
}

void regulation(uint8_t period, uint8_t captorNumber, uint16_t* p_pOld, int16_t* p_lastLatSpeed, int16_t* p_integral) {

	int16_t controlVar = pid(captorNumber, period, p_pOld, p_lastLatSpeed, p_integral);

	int16_t rightSpeed = MAX_SPEED_MM_S;
	int16_t leftSpeed = MAX_SPEED_MM_S;

	float speedWRatio = speedWheelRatio(controlVar);


	if(speedWRatio > 1 || speedWRatio < -1) {
		set_led(LED1, 0);
		set_led(LED5, 1);
	}
	if(speedWRatio < 1 && speedWRatio > -1) {
		set_led(LED5, 0);
		set_led(LED1, 1);
	}

	if(speedWRatio >= 1 || speedWRatio <= -1) {
		leftSpeed = MAX_SPEED_MM_S; // attention probleme avec les unité
		rightSpeed = leftSpeed / speedWRatio;
		/*set_led(LED3, 0);
		set_led(LED7, 1);*/
	}
	else if (speedWRatio < 1 && speedWRatio > -1){
		rightSpeed = MAX_SPEED_MM_S;
		leftSpeed = rightSpeed * speedWRatio;
		/*set_led(LED3, 1);
		set_led(LED7, 0);*/
	}

	left_motor_set_speed(leftSpeed);
	right_motor_set_speed(rightSpeed);
}


static THD_WORKING_AREA(regulation_thd_wa, 256); ///256????
static THD_FUNCTION(regulation_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// past measurements
	int16_t lastLatSpeed = 0; // pas sur
	uint16_t integral = 0;
	uint16_t pOld = get_distance(get_prox(2));
	//chprintf((BaseSequentialStream *)&SD3, " HI ");
	left_motor_set_speed(MAX_SPEED_MM_S);
	right_motor_set_speed(MAX_SPEED_MM_S);
	chThdSleepMilliseconds(PERIOD_REGULATOR);
	while(1){
		regulation(PERIOD_REGULATOR, 2, &pOld, &lastLatSpeed, &integral);

		chThdSleepMilliseconds(PERIOD_REGULATOR);
	}
}

void regulation_start(){
	chThdCreateStatic(regulation_thd_wa, sizeof(regulation_thd_wa), NORMALPRIO+1, regulation_thd, NULL);
}














/////// ****** Nouveau PI *******


void regulation_new(uint8_t period, uint8_t captorNumber, uint16_t* p_pOld, int16_t* p_lastLatSpeed, int16_t* p_integral) {

	int16_t controlVar = pid(captorNumber, period, p_pOld, p_integral);

	int16_t rightSpeed = MOTOR_SPEED_LIMIT_MARGIN;
	int16_t leftSpeed = MOTOR_SPEED_LIMIT_MARGIN;

	float speedWRatio = speedWheelRatio(controlVar);


	if(speedWRatio > 1 || speedWRatio < -1) {
		set_led(LED1, 0);
		set_led(LED5, 1);
	}
	if(speedWRatio < 1 && speedWRatio > -1) {
		set_led(LED5, 0);
		set_led(LED1, 1);
	}

	if(speedWRatio >= 1 || speedWRatio <= -1) {
		leftSpeed = MOTOR_SPEED_LIMIT_MARGIN;
		rightSpeed = leftSpeed / speedWRatio;
		/*set_led(LED3, 0);
		set_led(LED7, 1);*/
	}
	else if (speedWRatio < 1 && speedWRatio > -1){
		rightSpeed = MOTOR_SPEED_LIMIT_MARGIN;
		leftSpeed = rightSpeed * speedWRatio;
		/*set_led(LED3, 1);
		set_led(LED7, 0);*/
	}

	left_motor_set_speed(leftSpeed);
	right_motor_set_speed(rightSpeed);
}

int16_t pid(uint8_t captorNumber, uint16_t period, uint16_t* p_pOld, int16_t* p_integral) {

	// Get the current position
	uint16_t pNew = get_distance(get_prox(captorNumber));

	// Save current position in a pointer for next iteration
	*p_pOld = pNew;

	// Calculate the integral and save the value in a pointer for next iteration
	*p_integral = *p_integral + pNew;

	// Calculate the derivative
	int16_t deriv = pNew - *p_pOld;

	// Calculate the Control Variable
	int16_t c = (KP * pNew) + (KI * (*p_integral)) + (KD * deriv);

	return c;
}


