/*
 * Definitions.h
 *
 *  Created on: 21 ???. 2019 ?.
 *      Author: curab
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#define LEFT_SENSOR_PIN A2
#define RIGHT_SENSOR_PIN A4
#define FRONT_SENSOR_PIN A3

#define LEFT_MOTOR_ENCODER_1_PIN 18
#define LEFT_MOTOR_ENCODER_2_PIN 19
#define RIGHT_MOTOR_ENCODER_1_PIN 2
#define RIGHT_MOTOR_ENCODER_2_PIN 3

#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 4
#define LEFT_MOTOR_DIR_PIN 6
#define RIGHT_MOTOR_DIR_PIN 5

#define SPEED 200

#define LEFT_P (40)
#define LEFT_I (350)
#define LEFT_D (2)

#define RIGHT_P (40)
#define RIGHT_I (350)
#define RIGHT_D (2)
//
//#define LEFT_P 3.8 * 0.6
//#define LEFT_I 17.1
//#define LEFT_D 0.211
//
//#define RIGHT_P 3.8 * 0.6
//#define RIGHT_I 17.1
//#define RIGHT_D 0.211

#define ROBOT_R (1.7/100.0)
#define ROBOT_B (9.5/100.0)
#define ROBOT_C (298.0*12.0*2.33)

#endif /* DEFINITIONS_H_ */
