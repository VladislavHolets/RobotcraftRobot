/*
 * Motor.cpp
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#include "Motor.h"

#include <Arduino.h>

uint8_t Motor::getDirPin() const {
	return dirPin;
}

uint8_t Motor::getStepPin() const {
	return stepPin;
}

Motor::Motor(uint8_t dirPin, uint8_t stepPin) {
	this->dirPin = dirPin;
	this->stepPin = stepPin;
	pinMode(this->dirPin, OUTPUT);
}

Motor::Motor() {
	this->dirPin = 0;
	this->stepPin = 0;
}

