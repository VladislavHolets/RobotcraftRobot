/*
 * ROSData.cpp
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#include "ROSData.h"

#include <Arduino.h>

ROSData::ROSData() {
	// TODO Auto-generated constructor stub

}

float ROSData::getV() const {
	return v;
}

void ROSData::setV(float v) {
	this->v = v;
}

void ROSData::getData() {
	while (Serial.available()) {
		char temp[2];
		temp[1] = Serial.read();
		temp[2] = Serial.read();
		if (temp[1] == 'v') {
			if (temp[2] == ':') {
				this->setV(Serial.parseFloat());
				while (Serial.available() && Serial.peek() == ' ') {
					Serial.read();
				}
			}
		} else if (temp[1] == 'w') {
			if (temp[2] == ':') {
				this->setW(Serial.parseFloat());
				while (Serial.available() && Serial.peek() == ' ') {
					Serial.read();
				}
			}
		} else {
			while (Serial.available()
					&& (Serial.peek() != 'v' || Serial.peek() != 'w')) {
				Serial.read();
			}

		}
	}
}

float ROSData::getW() const {
	return w;
}

void ROSData::setW(float w) {
	this->w = w;
}

ROSData::~ROSData() {
	// TODO Auto-generated destructor stub
}

