/*
 * RangeSensor.h
 *
 *  Created on: 17 ???. 2019 ?.
 *      Author: curab
 */

#ifndef RANGESENSOR_H_
#define RANGESENSOR_H_

#include "Definitions.h"

#include <stdint.h>

class RangeSensor {
private:
	uint8_t pin;


public:
	uint16_t getValue();
	uint8_t getDistance();
	RangeSensor(uint8_t pin);
	virtual ~RangeSensor();
};

#endif /* RANGESENSOR_H_ */
