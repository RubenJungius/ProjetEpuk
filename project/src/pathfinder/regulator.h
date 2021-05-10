/*
 * regulator.h
 *
 *  Created on: 1 mai 2021
 *      Author: Luca
 */

#ifndef REGULATOR_H_
#define REGULATOR_H_

#define MAX_SPEED_MM_S 130
#define DIAM_ROBOT 70 // value to be change !!!
#define KP 1
#define	KI 0
#define	KD 0
#define	ALPHA 1000

#define PERIOD_REGULATOR 100


/* Find the tab values against the raw value and return the proportional converted value from these 2 values in tenth of mm.
   If the raw value is not in the tab values interval, it returns 0. */
uint16_t get_distance(uint16_t rawValue);

/* Position the robot according to a given distance between the front captor and an obstacle in mm. */
void positioning(uint16_t frontDist);

int16_t pid(uint8_t captorNumber, uint16_t period, uint16_t* pOld, int16_t* lastLatSpeed, int16_t* integral);

float speedWheelRatio(int16_t controlVar);

float derivative(uint16_t pOld, uint16_t pNew, uint16_t T);

void regulation(uint8_t period, uint8_t captorNumber, uint16_t* pOld, int16_t* lastLatSpeed, int16_t* integral);

//wrapper to launch thread
void regulation_start();

#endif /* REGULATOR_H_ */
