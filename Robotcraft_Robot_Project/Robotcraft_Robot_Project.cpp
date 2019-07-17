#include "Arduino.h"
#include "RangeSensor.h"

#include "Encoder.h"

#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A1
#define FRONT_SENSOR_PIN A2

#define LEFT_MOTOR_ENCODER_1_PIN 2
#define LEFT_MOTOR_ENCODER_2_PIN 3
#define RIGHT_MOTOR_ENCODER_1_PIN 18
#define RIGHT_MOTOR_ENCODER_2_PIN 19

RangeSensor LeftS(LEFT_SENSOR_PIN);
RangeSensor RightS(RIGHT_SENSOR_PIN);
RangeSensor FrontS(FRONT_SENSOR_PIN);

Encoder LeftM(LEFT_MOTOR_ENCODER_1_PIN,LEFT_MOTOR_ENCODER_2_PIN);
Encoder RightM(RIGHT_MOTOR_ENCODER_1_PIN,RIGHT_MOTOR_ENCODER_2_PIN);

//TODO: create class Motor, compose Encoder object into Motor Object so that it will be possible to create setPosition() function.
void setup() {

}

void loop() {

}
