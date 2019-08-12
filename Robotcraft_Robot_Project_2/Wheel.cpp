/*
 * Wheel.cpp
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#include "Wheel.h"

Wheel::Wheel(uint8_t encoderPin1, uint8_t encoderPin2, float p, float i, float d):encoder(encoderPin1,encoderPin2),PIDController(p,i,d) {
	// TODO Auto-generated constructor stub

}

