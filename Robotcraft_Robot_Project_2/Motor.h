/*
 * Motor.h
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

class Motor {
	uint8_t dirPin;
	uint8_t stepPin;

public:
	Motor();
	void applyForce();
};

#endif /* MOTOR_H_ */
