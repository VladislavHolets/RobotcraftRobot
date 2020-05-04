/*
 * Motor.h
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

class Motor
{
  uint8_t dirPin;
  //uint8_t stepPin;

public:
  Motor();
  Motor(uint8_t dirPin); //, uint8_t stepPin);
  uint8_t getDirPin() const;
  //uint8_t getStepPin() const;
};

#endif /* MOTOR_H_ */
