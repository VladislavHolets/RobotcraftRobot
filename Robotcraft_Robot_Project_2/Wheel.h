/*
 * Wheel.h
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#ifndef WHEEL_H_
#define WHEEL_H_

#include "Encoder.h"
#include "PID.h"

class Wheel {

public:
	Encoder encoder;
	PID PIDController;
	Wheel(uint8_t encoderPin1, uint8_t encoderPin2, float p, float i, float d);
};

#endif /* WHEEL_H_ */
