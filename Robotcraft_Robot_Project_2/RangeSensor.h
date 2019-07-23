/*
 * RangeSensor.h
 *
 *  Created on: 17 ???. 2019 ?.
 *      Author: curab
 */

#ifndef RANGESENSOR_H_
#define RANGESENSOR_H_

#include <stdint.h>

#define ANALOG_PIN(pin) (true)
#define SAMPLES_SIZE 10
#define ERROR 10
#define MAX 900
#define MIN 20
class RangeSensor {
private:
	uint8_t pin;
	RangeSensor();

public:
	uint16_t getValue();
	uint8_t getDistance();
	RangeSensor(uint8_t pin);
	virtual ~RangeSensor();
};

#endif /* RANGESENSOR_H_ */
