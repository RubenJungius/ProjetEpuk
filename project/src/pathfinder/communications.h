/*
 * communications.h
 *
 *  Created on: 09.05.2021
 *      Author: Ruben
 */

#ifndef COMMUNICATIONS_H_
#define COMMUNICATIONS_H_
void serial_start(void);
void sendInt16ToComputer(int16_t* data, uint16_t size);

#endif /* COMMUNICATIONS_H_ */
