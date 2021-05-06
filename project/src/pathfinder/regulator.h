/*
 * regulator.h
 *
 *  Created on: 1 mai 2021
 *      Author: Luca
 */

#ifndef REGULATOR_H_
#define REGULATOR_H_

#define MAX_SPEED_MM_S 130
#define DIAM_ROBOT 8 // value to be change !!!
#define KP 2
#define	KI 3
#define	KD 4
#define	ALPHA 5

#define PERIOD_REGULATOR 50

/* Find the tab values against the raw value and return the proportional converted value from these 2 values in tenth of mm.
   If the raw value is not in the tab values interval, it returns 0. */
uint16_t get_distance(uint16_t conversionTab[][2], uint16_t rawValue);

/* Position the robot according to a given distance between the front captor and an obstacle in mm. */
void positioning(uint16_t conversionTab[][2], uint16_t frontDist);

uint16_t pid(uint8_t captorNumber, uint16_t period, uint16_t* pOld, uint16_t* lastLatSpeed, uint16_t* integral, uint16_t conversionTab[][2]);

uint16_t speedWheelRatio(uint16_t controlVar);

uint16_t derivative(uint16_t* pOld, uint16_t pNew, uint16_t T);

void regulation(uint8_t period, uint8_t captorNumber, uint16_t* pOld, uint16_t* lastLatSpeed, uint16_t* integral, uint16_t conversionTab[][2]);

//wrapper to launch thread
void regulation_start(void);

#endif /* REGULATOR_H_ */
