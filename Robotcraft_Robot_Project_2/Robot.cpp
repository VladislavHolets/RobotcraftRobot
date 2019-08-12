/*
 * Robot.cpp
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#include "Robot.h"

Robot::Robot(uint8_t leftSensorPin, uint8_t rightSensorPin,
		uint8_t frontSensorPin, uint8_t leftEncoderPin1,
		uint8_t leftEncoderPin2, float leftP, float leftI, float leftD,
		uint8_t rightEncoderPin1, uint8_t rightEncoderPin2, float rightP,
		float rightI, float rightD) :
				leftSensor(leftSensorPin),
				rightSensor(rightSensorPin),
				frontSensor(frontSensorPin),
				leftWheel(leftEncoderPin1, leftEncoderPin2, leftP, leftI, leftD),
				rightWheel(rightEncoderPin1, rightEncoderPin2, rightP, rightI, rightD) {

}

void Robot::setVelocities(float v, float w) {

}

void Robot::setVelocities(struct Velocity desired) {
}

struct Velocity Robot::getVelocities() {
	this->realVelocity.v = TWO_PI * ROBOT_R
			* (this->leftWheel.encoder.getChange()
					+ this->rightWheel.encoder.getChange())/ ROBOT_C / 2.0
			/ DELTA_T;
	this->realVelocity.w = TWO_PI * ROBOT_R
			* (this->leftWheel.encoder.getChange()
					+ this->rightWheel.encoder.getChange())/ ROBOT_C / 2.0
			/ DELTA_T;
	return realVelocity;
}
