/*
 * regulator.h
 *
 *  Created on: 1 mai 2021
 *      Author: Luca
 */

#ifndef REGULATOR_H_
#define REGULATOR_H_

#define DIAM_ROBOT 54 //mm (the diameter that matters is the one between the wheels)
#define RADIUS_WHEEL 20.7 //mm

#define MARGIN 5


//#define MAX_SPEED_MM_S 130 // [mm/s]
#define MOTOR_SPEED_LIMIT_MARGIN 1000 / 2 // [step/s]
#define MOTOR_SPEED_LIMIT_MARGIN_RAD_S 2 * M_PI / 2 // [rad/s]
#define MOTOR_SPEED_LIMIT_MARGIN_MM_S 130 / 2 // [mm/s]


/* Find the tab values against the raw value and return the proportional converted value from these 2 values in tenth of mm.
   If the raw value is not in the tab values interval, it returns 0. */
float get_distance(uint16_t rawValue);

/* Position the robot according to a given distance between the front captor and an obstacle in mm.*/
void dist_positioning(uint16_t frontDist);

/* angular positioning of the robot */
void angle_positioning(float angle);

//wrapper to launch thread
void regulation_start(void);

//PID

void regulation(/*uint8_t captorNumber,*/ float* p_pOld,/* float* p_alphaNew,*/ int16_t* p_integral/* uint8_t* p_firstDetection*/);

float pid(float p_pOld, float pNew, int16_t* p_integral);

float speedWheelRatio(float gama);


#endif /* REGULATOR_H_ */
