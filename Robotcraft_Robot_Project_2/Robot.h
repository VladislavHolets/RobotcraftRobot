/*
 * Robot.h
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "RangeSensor.h"
#include "Wheel.h"
#include "DataStruct.h"

class Robot {
	Wheel rightWheel, leftWheel;
	RangeSensor rightSensor, frontSensor, leftSensor;
	struct Velocity desiredVelocity, realVelocity;
public:
	Robot(uint8_t leftSensorPin, uint8_t rightSensorPin,
			uint8_t frontSensorPin, uint8_t leftEncoderPin1,
			uint8_t leftEncoderPin2, float leftP, float leftI, float leftD,
			uint8_t rightEncoderPin1, uint8_t rightEncoderPin2, float rightP,
			float rightI, float rightD);
	void setVelocities(float v, float w);
	void setVelocities(Velocity desired);
	Velocity getVelocities();

};

#endif /* ROBOT_H_ */
