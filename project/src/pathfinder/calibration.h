/*
 * calibration.h
 *
 *  Created on: 30 avr. 2021
 *      Author: Luca
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_


/* Fill a tab of correspondence between raw values of the proximity captors and real distances.
   This is done by moving the robot with steps of 1mm (starting at MEASUREMENT_NUMBER + 2 mm of a wall) with measurement recorded each step. */
void calibration(void);

/* Give the converted value in mm of a captor measurement using the conversionTab. */
float get_distance(uint16_t rawValue);

/* Convert the speed from mm/s in step/s */
float speed_conversion(int16_t speed_mm_s);

#endif /* CALIBRATION_H_ */
