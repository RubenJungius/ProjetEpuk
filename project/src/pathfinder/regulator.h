/*
 * regulator.h
 *
 *  Created on: 1 mai 2021
 *      Author: Luca
 */

#ifndef REGULATOR_H_
#define REGULATOR_H_


/* Position the robot according to a given distance between the front captor and an obstacle in mm.*/
void dist_positioning(uint16_t frontDist);

/* angular positioning of the robot */
void angle_positioning(float angle);


// wrapper to launch thread
void regulation_start(void);

int regulator_return_status(void);

int regulator_return_count(void);


mutex_t* regulator_get_mutex(void);

condition_variable_t* regulator_get_condition(void);

#endif /* REGULATOR_H_ */
