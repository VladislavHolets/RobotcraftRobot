/*
 * Error.cpp
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#include "Error.h"

Error::Error() {
	// TODO Auto-generated constructor stub
	this->type = P_TYPE;
	this->value = 0;
}

void Error::setType(uint8_t type) {
	this->type = type;
}

Error::~Error() {
	// TODO Auto-generated destructor stub
}

Error::Error(uint8_t type) {
	this->type = type;
	this->value = 0;
}

void Error::setValue(float value) {
	this->value = value;
}

float Error::getValue() {
	return this->value;
}

void Error::next(Error now, Error prev) {
	if (this->type == P_TYPE) {
		this->value = now.getValue();
	} else if (this->type == I_TYPE) {
		this->value += now.getValue();
	} else if (this->type == D_TYPE) {
		this->value = now.getValue() - prev.getValue();
	}
}
