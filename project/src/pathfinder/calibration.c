/*
 * calibration.c
 *
 *  Created on: 30 avr. 2021
 *      Author: Luca
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "calibration.h"
#include "constants.h"
#include <motors.h>
#include <leds.h>

#include "sensors/proximity.h"

#define CALIBRATION_SPEED 10 //  mm/s
#define MEASUREMENT_DISTANCE 1 // mm


// Tab of correspondence threw raw proximity values and correspondent distance values
uint16_t conversionTab[MEASUREMENT_NUMBER][2];


void calibration() {

	// fill the first column with MEASUREMENT_NUMBER values with an interval of 1 between each values
	for(int i = 0; i < MEASUREMENT_NUMBER ; i++){
		conversionTab[i][0] = 3 + 1 * i;
	}

	set_led(LED1, 1);
	for(int i = MEASUREMENT_NUMBER - 1; i >= 0 ; i--){
		// move the robot of 1 mm then put the proximity measurement in the tab
		left_motor_set_speed(speed_conversion(CALIBRATION_SPEED));
		right_motor_set_speed(speed_conversion(CALIBRATION_SPEED));
		chThdSleepMilliseconds(MEASUREMENT_DISTANCE * 1000 / CALIBRATION_SPEED);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		conversionTab[i][1] = get_prox(0);
	}
	set_led(LED1, 0);
}

float get_distance(uint16_t rawValue) {
	for(int i = 0; i < MEASUREMENT_NUMBER - 1; i++) {
		// Find the closest values in the tab and return the proportional converted result in mm.
		if((rawValue <= conversionTab[i][1]) && (rawValue > conversionTab[i + 1][1])) {
			uint16_t interval = conversionTab[i][1] - conversionTab[i + 1][1];
			uint16_t a = conversionTab[i][1] - rawValue;
			return conversionTab[i][0] + ((float)a/(float)interval);
		}
	}
	// very close to an obstacle
	if (rawValue > conversionTab[0][1]) {
		return 0;
	}
	// far from an obstacle
	else return (float)(MEASUREMENT_NUMBER + 3);
}

float speed_conversion(int16_t speed_mm_s) {
	return speed_mm_s * 7.692;
}
