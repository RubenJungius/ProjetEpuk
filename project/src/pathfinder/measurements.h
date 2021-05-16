/*
 * measurements.h
 *
 *  Created on: 13 mai 2021
 *      Author: Luca
 */

#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#define RECORDED_MEASUREMENTS_NUMBER 4

// Start the measurement thread
void measurements_start(void);

// Give the value of alpha
float get_alpha(void);

// Give the tab of the last distances recorded
float* get_dist_data(void);

mutex_t* get_mutex(void);

condition_variable_t* get_condition(void);


#endif /* MEASUREMENTS_H_ */
