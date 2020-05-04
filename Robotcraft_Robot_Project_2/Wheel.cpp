/*
 * Wheel.cpp
 *
 *  Created on: 5 ????. 2019 ?.
 *      Author: curab
 */

#include "Wheel.h"

#include <stdint.h>
//#include <stdint.h>

class Adafruit_MCP4728;

Wheel::Wheel(uint8_t encoderPin1, uint8_t encoderPin2, float p, float i, float d, uint8_t dirPin, Adafruit_MCP4728 *DAC,
             uint8_t channel //, uint8_t stepPin
             ) :
    encoder(encoderPin1, encoderPin2), PIDController(p, i, d, dirPin, DAC, channel //, stepPin
                                                     )
{
  // TODO Auto-generated constructor stub
  this->desiredVelocity = 0;
  this->realVelocity = 0;

}

float Wheel::getDesiredVelocity() const
{
  return this->desiredVelocity;
}

void Wheel::setDesiredVelocity(float desiredVelocity)
{
  this->desiredVelocity = desiredVelocity;
}

float Wheel::getRealVelocity() const
{
  return this->realVelocity;
}

void Wheel::setRealVelocity(float realVelocity)
{
  this->realVelocity = realVelocity;
}

void Wheel::updatePID()
{
  this->PIDController.apply(
      this->PIDController.normalize(this->PIDController.calc(this->getDesiredVelocity(), this->getRealVelocity())));
}
