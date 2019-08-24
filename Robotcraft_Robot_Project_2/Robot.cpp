/*
 * Robot.cpp
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#include "Robot.h"

#include <Arduino.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

#include "Constants.h"
#include "Encoder.h"

Robot::Robot(uint8_t leftSensorPin, uint8_t rightSensorPin,
		uint8_t frontSensorPin, uint8_t leftEncoderPin1,
		uint8_t leftEncoderPin2, float leftP, float leftI, float leftD,
		uint8_t leftDirPin, uint8_t leftStepPin, uint8_t rightEncoderPin1,
		uint8_t rightEncoderPin2, float rightP, float rightI, float rightD,
		uint8_t rightDirPin, uint8_t rightStepPin, uint8_t neoPixelPin,
		uint16_t neoPixelNum, uint8_t beepPin) :
		leftSensor(leftSensorPin), rightSensor(rightSensorPin), frontSensor(
				frontSensorPin), leftWheel(leftEncoderPin1, leftEncoderPin2,
				leftP, leftI, leftD, leftDirPin, leftStepPin), rightWheel(
				rightEncoderPin1, rightEncoderPin2, rightP, rightI, rightD,
				rightDirPin, rightStepPin), pixels(neoPixelNum, neoPixelPin,
		NEO_GRB + NEO_KHZ800) {
	this->pixels_size = neoPixelNum;
	this->beepPin = beepPin;
	pinMode(this->beepPin, OUTPUT);
	digitalWrite(this->beepPin, LOW);

}

const geometry_msgs::Pose2D Robot::getPosition() {
	const geometry_msgs::Pose2D result(this->position);
//	result.theta = this->position.theta;
//	result.x = this->position.x;
//	result.y = this->position.y;
	return result;
}

void Robot::setPosition(geometry_msgs::Pose2D position) {
	this->position.theta = position.theta;
	this->position.x = position.x;
	this->position.y = position.y;

}

const geometry_msgs::Twist Robot::getDesireVelocity() {
	const geometry_msgs::Twist result(this->desiredVelocity);
	return result;
}

void Robot::setDesiredVelocity(geometry_msgs::Twist desired) {
	this->desiredVelocity.linear.x = desired.linear.x;
	this->desiredVelocity.angular.z = desired.angular.z;
}

const geometry_msgs::Twist Robot::getRealVelocity() {
	const geometry_msgs::Twist result(this->realVelocity);
	return result;
}

void Robot::setRealVelocity(geometry_msgs::Twist real) {
	this->realVelocity.linear.x = real.linear.x;
	this->realVelocity.angular.z = real.angular.z;
}

void Robot::updateWheelsVelocities() {
	this->realVelocity.linear.x =
	TWO_PI * ROBOT_R
			* (rightWheel.encoder.getChange() + leftWheel.encoder.getChange())
			/ ROBOT_C / 2.0 / DELTA_T;
	this->realVelocity.angular.z =
	TWO_PI * ROBOT_R
			* (rightWheel.encoder.getChange() - leftWheel.encoder.getChange())
			/ ROBOT_C / ROBOT_B / DELTA_T;

	this->leftWheel.setRealVelocity(
			(this->realVelocity.linear.x
					- this->ROBOT_B / 2 * this->realVelocity.angular.z)
					/ this->ROBOT_R);
	this->rightWheel.setRealVelocity(
			(this->realVelocity.linear.x
					+ this->ROBOT_B / 2 * this->realVelocity.angular.z)
					/ this->ROBOT_R);

	this->leftWheel.setDesiredVelocity(
			(this->desiredVelocity.linear.x
					- this->ROBOT_B / 2 * this->desiredVelocity.angular.z)
					/ this->ROBOT_R);
	this->rightWheel.setDesiredVelocity(
			(this->desiredVelocity.linear.x
					+ this->ROBOT_B / 2 * this->desiredVelocity.angular.z)
					/ this->ROBOT_R);

}

void Robot::updateRobotPose() {
	this->position.theta = atan2(
			sin(this->position.theta + this->realVelocity.angular.z * DELTA_T),
			cos(this->position.theta + this->realVelocity.angular.z * DELTA_T));
	this->position.x = this->position.x
			+ realVelocity.linear.x * DELTA_T * cos(this->position.theta);
	this->position.y = this->position.y
			+ realVelocity.linear.x * DELTA_T * sin(this->position.theta);

}

void Robot::updateWheelsPID() {
	this->leftWheel.updatePID();
	this->rightWheel.updatePID();
}

