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


void calibration(uint16_t conversionTab[][2]) {

	// fill the first column with MEASUREMENT_NUMBER values with an interval of 1 between each values
	for(int i = 0; i < MEASUREMENT_NUMBER ; i++){
		conversionTab[i][0] = 3 + 1 * i;
	}

	set_led(LED1, 1);
	for(int i = MEASUREMENT_NUMBER - 1; i >= 0 ; i--){
		// move the robot of 1 mm then put the proximity measurement in the tab
		left_motor_set_speed(speed_conversion(10)); // 8mm/s
		right_motor_set_speed(speed_conversion(10)); // 8mm/s
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

uint16_t speed_conversion(uint8_t speed_mm_s) {
	return speed_mm_s * 7.692;
}
