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
#include <motors.h>
#include <leds.h>

#include "sensors/proximity.h"

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
		left_motor_set_speed(speed_conversion(10)); // 10mm/s (attention magic number)
		right_motor_set_speed(speed_conversion(10)); // 10mm/s
		chThdSleepMilliseconds(100);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		conversionTab[i][1] = get_prox(0);
		// security in order to only have values that increase when the position to the obstacle decrease
		/*if (i == MEASUREMENT_NUMBER - 1 || conversionTab[i][1] > conversionTab[i + 1][1]) {
			conversionTab[i][1] = get_prox(i);
		}
		else {
			conversionTab[i][1] = conversionTab[i + 1][1];
			set_led(LED3, 1);
		}*/
	}
	set_led(LED1, 0);
}

uint16_t get_distance(uint16_t rawValue) {
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

uint16_t speed_conversion(uint8_t speed_mm_s) {
	return speed_mm_s * 7.692;
}
