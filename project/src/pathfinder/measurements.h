/*
 * measurements.h
 *
 *  Created on: 13 mai 2021
 *      Author: Luca
 */

#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

void measurements_start(void);

// Use a FIFO method to store in a tab the last distance data from a captor
void measurements(uint8_t captorNumber, fixed_point* p_alpha);

// Find the (average) angle between the robot current direction and a wall
void find_alpha(fixed_point* p_alpha);

// Give the value of alpha
fixed_point get_alpha(void);

// Give the tab of the last distances recorded
fixed_point* get_dist_data(void);

mutex_t* get_mutex(void);

condition_variable_t* get_condition(void);


#endif /* MEASUREMENTS_H_ */
