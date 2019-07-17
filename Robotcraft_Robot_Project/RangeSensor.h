/*
 * RangeSensor.h
 *
 *  Created on: 17 ???. 2019 ?.
 *      Author: curab
 */

#ifndef RANGESENSOR_H_
#define RANGESENSOR_H_

#include <stdint.h>

#define ANALOG_PIN(pin) (pin>=82 && pin<=97)
#define SAMPLES_SIZE 10
#define ERROR 10
#define MAX 900
#define MIN 100
class RangeSensor {
private:
	uint8_t pin;
	RangeSensor();
	uint16_t getValue();
public:
	uint8_t getDistance();
	RangeSensor(uint8_t pin);
	virtual ~RangeSensor();
};

#endif /* RANGESENSOR_H_ */
