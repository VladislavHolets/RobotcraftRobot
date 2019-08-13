/*
 * Robot.h
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <Adafruit_NeoPixel.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <stdint.h>

#ifdef __AVR__
#include <avr/power.h>
#endif
#include "RangeSensor.h"
#include "Wheel.h"

class Robot {
	const uint8_t LEFT = 0;
	const uint8_t RIGHT = 1;
	const uint8_t FRONT = 2;
	const float ROBOT_R = (1.7 / 100.0);
	const float ROBOT_B = (9.5 / 100.0);
	const float ROBOT_C = (298.0 * 12.0 * 2.33);
	uint16_t pixels_size;

	//RangeSensor rightSensor, frontSensor, leftSensor;
	geometry_msgs::Twist desiredVelocity, realVelocity;
	geometry_msgs::Pose2D position;
public:
	RangeSensor rightSensor, frontSensor, leftSensor;
	Wheel rightWheel, leftWheel;
	Adafruit_NeoPixel pixels;
	Robot(uint8_t leftSensorPin, uint8_t rightSensorPin, uint8_t frontSensorPin,
			uint8_t leftEncoderPin1, uint8_t leftEncoderPin2, float leftP,
			float leftI, float leftD, uint8_t leftDirPin, uint8_t leftStepPin,
			uint8_t rightEncoderPin1, uint8_t rightEncoderPin2, float rightP,
			float rightI, float rightD, uint8_t rightDirPin,
			uint8_t rightStepPin, uint8_t neoPixelPin, uint16_t neoPixelNum);

	const geometry_msgs::Pose2D getPosition();
	void setPosition(geometry_msgs::Pose2D position);

	const geometry_msgs::Twist getDesireVelocity();
	void setDesiredVelocity(geometry_msgs::Twist desired);

	const geometry_msgs::Twist getRealVelocity();
	void setRealVelocity(geometry_msgs::Twist real);

	void updateWheelsVelocities();
	void updateRobotPose();
	void updateWheelsPID();

};

#endif /* ROBOT_H_ */
