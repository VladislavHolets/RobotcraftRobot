/*
 * RangeSensor.cpp
 *
 *  Created on: 17 ???. 2019 ?.
 *      Author: curab
 */

#include "RangeSensor.h"

#include <Arduino.h>
#include <stdint.h>

RangeSensor::RangeSensor() {

	Serial.println("No port in RangeSensor object -> while(1){}");
	while (1) {
	};
}

uint16_t RangeSensor::getValue() {
	uint16_t samples[SAMPLES_SIZE];
	uint16_t average;
	for (int i = 0; i < SAMPLES_SIZE; i++) {
		samples[i] = analogRead(this->pin);
		average += samples[i];
	}
	average /= SAMPLES_SIZE;
	if (average > MAX || average < MIN) {
		return 0;
	}
	for (int i = 0; i < SAMPLES_SIZE; i++) {
		if ((samples[i] > (average + ERROR))
				|| (samples[i] < (average - ERROR))) {
			//dangerous part TODO: safetify
			return getValue();
		}
	}
	return average;
}

uint8_t RangeSensor::getDistance() {
	return (float) (27.728
			* pow(map(this->getValue(), 0, 1023, 0, 5000) / 1000.0, -1.2045));
}

RangeSensor::RangeSensor(uint8_t pin) {
	if (ANALOG_PIN(pin)) {
		this->pin = pin;
	} else {
		Serial.println("Wrong port in RangeSensor object -> while(1){}");
		while (1) {
		};
	}
}

RangeSensor::~RangeSensor() {

}

