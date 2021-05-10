/*
 * calibration.h
 *
 *  Created on: 30 avr. 2021
 *      Author: Luca
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#define MEASUREMENT_NUMBER 48

/* Fill a tab of correspondence between raw values of the proximity captors and real distances.
   This is done by moving the robot with steps of 1mm (starting at 5 cm of a wall) with measurement recorded each step. */
void calibration();

uint16_t get_distance(uint16_t rawValue);

/* Convert the speed from mm/s in step/s */
uint16_t speed_conversion(uint8_t speed_mm_s);


#endif /* CALIBRATION_H_ */
