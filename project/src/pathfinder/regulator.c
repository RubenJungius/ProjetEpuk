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

// Tab of correspondence threw raw proximity values and correspondent distance values
uint16_t conversionTab[MEASUREMENT_NUMBER][2];

uint16_t get_distance(uint16_t conversionTab[][2], uint16_t rawValue) {
	for(int i = 0; i < MEASUREMENT_NUMBER - 1; i++) {
		// Find the closest values in the tab and return the proportional converted result.
		if((rawValue <= conversionTab[i][1]) && (rawValue > conversionTab[i + 1][1])) {
			/*uint16_t interval = conversionTab[i][1] - conversionTab[i + 1][1];
			uint16_t a = conversionTab[i][1] - rawValue;
			return 10 * conversionTab[i][0] + (a % interval);*/
			return conversionTab[i][0];
		}
	}
	set_led(LED3, 0);
	// very close to an obstacle
	if (rawValue > conversionTab[0][1]) {
		return 0;
	}
	// far from an obstacle
	else return 100; // attention magic number
}

void positioning(uint16_t conversionTab[][2], uint16_t frontDist) {
	uint16_t speed = speed_conversion(MAX_SPEED_MM_S / 3); // attention magic number !!!
	// move forward
	if (get_distance(conversionTab, get_prox(0)) > frontDist) {
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		set_led(LED1, 1);
		while (get_distance(conversionTab, get_prox(0)) >= frontDist) {}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_led(LED1, 0);
	}
	// move backward
	else {
		left_motor_set_speed(-speed);
		right_motor_set_speed(-speed);
		set_led(LED5, 1);
		while (get_distance(conversionTab, get_prox(0)) <= frontDist) {}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		set_led(LED5, 0);
	}
}

uint16_t pid(uint8_t captorNumber, uint16_t period, uint16_t* p_pOld, uint16_t* p_lastLatSpeed, uint16_t* p_integral, uint16_t conversionTab[][2]) {

	// Get the current position
	uint16_t pNew = get_distance(conversionTab, get_prox(captorNumber));

	// Get the current lateral speed
	uint16_t currentLatSpeed = derivative(p_pOld, pNew, period); // the error (we want it to be 0)

	// Save current position in a pointer for next iteration
	*p_pOld = pNew;

	// Calculate the integral and save the value in a pointer for next iteration
	*p_integral = *p_integral + currentLatSpeed;

	// Calculate the derivative
	uint16_t deriv = currentLatSpeed - *p_lastLatSpeed;

	// Save current lateral speed in a pointer for next iteration
	*p_lastLatSpeed = currentLatSpeed;

	// Calculate the Control Variable
	uint16_t c = (KP * currentLatSpeed) + (KI * (*p_integral)) + (KD * deriv);

	return c;
}


uint16_t speedWheelRatio(uint16_t controlVar) {
	uint16_t rRotRob = ALPHA / controlVar; // rRotRob is inverse proportional to the control variable
	return - (rRotRob + (DIAM_ROBOT/2)) / (rRotRob - (DIAM_ROBOT/2));
}


uint16_t derivative(uint16_t* p_pOld, uint16_t pNew, uint16_t T) {
		return (pNew - *p_pOld) / T;
}

void regulation(uint8_t period, uint8_t captorNumber, uint16_t* p_pOld, uint16_t* p_lastLatSpeed, uint16_t* p_integral, uint16_t conversionTab[][2]) {

	uint16_t controlVar = pid(captorNumber, period, p_pOld, p_lastLatSpeed, p_integral, conversionTab);

	uint16_t rightSpeed = MAX_SPEED_MM_S;
	uint16_t leftSpeed = MAX_SPEED_MM_S;

	if(speedWheelRatio(controlVar) >= 1) {
		//set_led(LED7, 1);
		if(speedWheelRatio(controlVar) > 1) {
			set_led(LED7, 1);
		}
		else if(speedWheelRatio(controlVar) == 1) {
			set_led(LED3, 1);
		}
		leftSpeed = MAX_SPEED_MM_S;
		rightSpeed = leftSpeed / speedWheelRatio(controlVar);
	}
	else {
		set_led(LED3, 1);
		rightSpeed = MAX_SPEED_MM_S;
		leftSpeed = rightSpeed * speedWheelRatio(controlVar);
	}
	left_motor_set_speed(leftSpeed);
	right_motor_set_speed(rightSpeed);
}


static THD_WORKING_AREA(regulation_thd_wa, 256); ///256????
static THD_FUNCTION(regulation_thd, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	// past measurements
	uint16_t lastLatSpeed = 0; // pas sur
	uint16_t integral = 0;
	uint16_t pOld = get_distance(conversionTab, get_prox(2));

	while(1){
		regulation(PERIOD_REGULATOR, 2, &pOld, &lastLatSpeed, &integral, conversionTab);

		chThdSleepMilliseconds(PERIOD_REGULATOR);
	}
}

void regulation_start(){
	chThdCreateStatic(regulation_thd_wa, sizeof(regulation_thd_wa), NORMALPRIO+1, regulation_thd, NULL);
}
