/*
 * Wheel.h
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#ifndef WHEEL_H_
#define WHEEL_H_

#include <stdint.h>

#include "Encoder.h"
#include "PID.h"

class Wheel
{
  float desiredVelocity, realVelocity;
  PID PIDController;
public:
  Encoder encoder;
  Wheel(uint8_t encoderPin1, uint8_t encoderPin2, float p, float i, float d, uint8_t dirPin, Adafruit_MCP4728 *DAC,
        uint8_t channel //,uint8_t stepPin
        );
  float getDesiredVelocity() const;
  void setDesiredVelocity(float desiredVelocity);
  float getRealVelocity() const;
  void setRealVelocity(float realVelocity);
  void updatePID();
};

#endif /* WHEEL_H_ */
