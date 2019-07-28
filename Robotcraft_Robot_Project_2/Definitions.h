/*
 * Definitions.h
 *
 *  Created on: 21 ???. 2019 ?.
 *      Author: curab
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

//RangeSensors pins
#define LEFT_SENSOR_PIN A2
#define RIGHT_SENSOR_PIN A4
#define FRONT_SENSOR_PIN A3

//Encoders pins
#define LEFT_MOTOR_ENCODER_1_PIN 18
#define LEFT_MOTOR_ENCODER_2_PIN 19
#define RIGHT_MOTOR_ENCODER_1_PIN 2
#define RIGHT_MOTOR_ENCODER_2_PIN 3

//Motors pins
#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 4
#define LEFT_MOTOR_DIR_PIN 6
#define RIGHT_MOTOR_DIR_PIN 5

//Left wheel PID parameters
#define LEFT_P (40)
#define LEFT_I (350)
#define LEFT_D (2)

//Right wheel PID parameters
#define RIGHT_P (40)
#define RIGHT_I (350)
#define RIGHT_D (2)

//Robot parameters
#define ROBOT_R (1.7/100.0)
#define ROBOT_B (9.5/100.0)
#define ROBOT_C (298.0*12.0*2.33)

//Common delta t
#define DELTA_T (0.1)

//Error
#define P_TYPE 0
#define I_TYPE 1
#define D_TYPE 2

//RangeSensor
#define ANALOG_PIN(pin) (true)
#define SAMPLES_SIZE 10
#define ERROR 10
#define MAX 900
#define MIN 20

#define LEFT 0
#define RIGHT 1
#define FRONT 2

#endif /* DEFINITIONS_H_ */
