/*
 * regulator.h
 *
 *  Created on: 1 mai 2021
 *      Author: Luca
 */

#ifndef REGULATOR_H_
#define REGULATOR_H_

/* Find the tab values against the raw value and return the proportional converted value from these 2 values in tenth of mm.
   If the raw value is not in the tab values interval, it returns 0. */
fixed_point get_distance(uint16_t rawValue);

/* Position the robot according to a given distance between the front captor and an obstacle in mm.*/
void dist_positioning(uint16_t frontDist);

/* angular positioning of the robot */
void angle_positioning(fixed_point angle);

//wrapper to launch thread
void regulation_start(void);

//PID

fixed_point regulation(fixed_point* p_pOld, fixed_point* p_integral);

fixed_point pid(fixed_point p_pOld, fixed_point pNew, fixed_point* p_integral);

fixed_point speedWheelRatio(fixed_point gama);


#endif /* REGULATOR_H_ */
