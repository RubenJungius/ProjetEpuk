/*
 * constants.h
 *
 *  Created on: 16.05.2021
 *      Author: Ruben
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//#define AUDIO
#define DRIVE
#define COLLISION
#define PROXIMITY

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640

#define MOTOR_SPEED_LIMIT_MARGIN 1000.0 / 2.0 // [step/s]
#define MOTOR_SPEED_LIMIT_MARGIN_RAD_S 2 * M_PI / 2.0 // [rad/s]
#define MOTOR_SPEED_LIMIT_MARGIN_MM_S 130.0 / 2.0 // [mm/s]

#define DIAM_ROBOT 54 //mm (the diameter that matters is the one between the wheels)
#define RADIUS_WHEEL 20.7 //mm

#define PERIOD_REGULATOR	 0.2 // sec
#define R_ROT_ROB_MIN	 DIAM_ROBOT
#define MAX_DIST_ONE_CYCLE  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR // mm
#define MAX_ANGLE_ROT	 (float)(MAX_DIST_ONE_CYCLE) / (float)(R_ROT_ROB_MIN)
#define DIST_DETECTION	   MEASUREMENT_NUMBER + 2 // mm
#define OFFSET 		20 // mm, distance to the wall we want the robot to stabilize

#define KP 		(float)(MAX_ANGLE_ROT) / (float)(DIST_DETECTION)
#define	KI		0.0000
#define	KD		0 //0.01

#define PERIOD_MEASUREMENTS 0.2 //sec
#define PERIOD_MEASUREMENT 0.025 //sec
#define MAX_DIST_ONE_CYCLE_MEASUREMENT  	MOTOR_SPEED_LIMIT_MARGIN_RAD_S * RADIUS_WHEEL * PERIOD_MEASUREMENT // mm

#define RECORDED_MEASUREMENTS_NUMBER 4

#define MEASUREMENT_NUMBER 48

#endif /* CONSTANTS_H_ */
