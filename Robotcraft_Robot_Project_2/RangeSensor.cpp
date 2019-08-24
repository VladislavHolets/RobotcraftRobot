/*
 * RangeSensor.cpp
 *
 *  Created on: 17 ???. 2019 ?.
 *      Author: curab
 */

#include "RangeSensor.h"

#include <Arduino.h>
#include <math.h>

uint16_t RangeSensor::getValue() {
	//uint16_t samples[SAMPLES_SIZE];
	uint16_t average = 0;
	for (int i = 0; i < SAMPLES_SIZE; i++) {
		//samples[i] = analogRead(this->pin);
		//average += samples[i];
		//noInterrupts();
		average += analogRead(this->pin);
		//interrupts();
	}
	average /= SAMPLES_SIZE;
	/*	if (average > MAX || average < MIN) {
	 *		return 0;
	 *	}
	 */
	/*	for (int i = 0; i < SAMPLES_SIZE; i++) {
	 *		if ((samples[i] > (average + ERROR))
	 *				|| (samples[i] < (average - ERROR))) {
	 *			//dangerous part TODO: safetify
	 *			// return getValue();
	 *			return 0;
	 *		}
	 *	}
	 */

	return average;

}

float RangeSensor::getDistance() {
	uint16_t value = this->getValue();
	if (value != 0) {
		return 155.27 * pow((float) this->getValue(), -1.195);
	} else {
		return 0;
	}
}

RangeSensor::RangeSensor(uint8_t pin) {
	if (ANALOG_PIN(pin)) {
		this->pin = pin;
	} else {
		//Serial.println("Wrong port in RangeSensor object -> while(1){}");
		while (1) {
		};
	}
}

RangeSensor::~RangeSensor() {

}

