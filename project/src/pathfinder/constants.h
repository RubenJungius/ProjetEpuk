/*
 * constants.h
 *
 *  Created on: 16.05.2021
 *      Author: Ruben
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_


#define AUDIO
#define DRIVE
#define COLLISION
#define PROXIMITY


// Constants for the different parts of the project


// Speed constants
#define MOTOR_SPEED_LIMIT_MARGIN 1000   // [step/s]
#define MOTOR_SPEED_LIMIT_MARGIN_RAD_S 2 * M_PI   // [rad/s]
#define MOTOR_SPEED_LIMIT_MARGIN_MM_S 130   // [mm/s]

// Robot constants
#define DIAM_ROBOT 54 //mm (the diameter that matters is the one between the wheels)
#define RADIUS_WHEEL 20.7 //mm

// Period constants
#define PERIOD_REGULATOR	 0.1 // sec

#define MEASUREMENT_NUMBER 48 // number of sampling in the calibration process

#endif /* CONSTANTS_H_ */
