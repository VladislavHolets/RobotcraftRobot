/*
 * RangeSensor.h
 *
 *  Created on: 17 ???. 2019 ?.
 *      Author: curab
 */

#ifndef RANGESENSOR_H_
#define RANGESENSOR_H_

#define ANALOG_PIN(pin) (true)
#include "Constants.h"

#include <stdint.h>

class RangeSensor {
private:
	uint8_t pin;

	const uint8_t SAMPLES_SIZE = 10;
	const uint8_t ERROR = 10;
	const uint16_t MAX = 900;
	const uint8_t MIN = 20;

public:
	uint16_t getValue();
	float getDistance();
	RangeSensor(uint8_t pin);
	virtual ~RangeSensor();
};

#endif /* RANGESENSOR_H_ */
