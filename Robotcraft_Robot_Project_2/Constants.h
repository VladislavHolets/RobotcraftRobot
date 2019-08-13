/*
 * Definitions.h
 *
 *  Created on: 21 ???. 2019 ?.
 *      Author: curab
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <pins_arduino.h>
#include <stdint.h>

//RangeSensors pins
const uint8_t LEFT_SENSOR_PIN = A2;
const uint8_t RIGHT_SENSOR_PIN = A4;
const uint8_t FRONT_SENSOR_PIN = A3;

//Encoders pins
const uint8_t LEFT_MOTOR_ENCODER_1_PIN = 18;
const uint8_t LEFT_MOTOR_ENCODER_2_PIN = 19;
const uint8_t RIGHT_MOTOR_ENCODER_1_PIN = 2;
const uint8_t RIGHT_MOTOR_ENCODER_2_PIN = 3;

//Motors pins
const uint8_t LEFT_MOTOR_STEP_PIN = 9;
const uint8_t RIGHT_MOTOR_STEP_PIN = 4;
const uint8_t LEFT_MOTOR_DIR_PIN = 6;
const uint8_t RIGHT_MOTOR_DIR_PIN = 5;

//Left wheel PID parameters
const float LEFT_P = (40);
const float LEFT_I = (350);
const float LEFT_D = (2);

//Right wheel PID parameters
const float RIGHT_P = (40);
const float RIGHT_I = (350);
const float RIGHT_D = (2);

//Common delta t
const float DELTA_T = (0.1);

//NeoPixels
const uint8_t NEOPIXEL_PIN = 10;
const uint16_t NEOPIXEL_NUM = 2;

//Other
const uint32_t BAUD = 115200;
#endif /* CONSTANTS_H_ */
