/*
 * PID.cpp
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#include "PID.h"

#include <Arduino.h>

#include "Constants.h"

PID::PID(float p, float i, float d, uint8_t dirPin, uint8_t stepPin) :
		Motor(dirPin, stepPin) {
	this->p = p;
	this->i = i;
	this->d = d;
	this->ProportionalPrev.setType(P_TYPE);
	this->ProportionalPrev.setValue(0);
	this->Proportional.setType(P_TYPE);
	this->Proportional.setValue(0.0);
	this->Integral.setType(I_TYPE);
	this->Integral.setValue(0.0);
	this->Derivative.setType(D_TYPE);
	this->timeNew = millis();
	this->timePrev = millis();
}

float PID::calc(float desired, float real) {
	if (desired < 0.0001 && desired > -0.0001) {
		this->Proportional.setValue(0);
		this->ProportionalPrev.setValue(0);
		this->Integral.setValue(0);
		this->Derivative.setValue(0);
	}
	this->timePrev = this->timeNew;
	this->timeNew = millis();
	//TODO: make one time stamp for all of the objects
	this->updateErrors(desired, real);
	float result = this->p * this->Proportional.getValue()
			+ this->i * this->Integral.getValue() * DELTA_T
			+ this->d * this->Derivative.getValue() / DELTA_T;
	return result;
}

int16_t PID::normalize(float PIDresult) {
	if (abs(PIDresult) > 200) {
		PIDresult = PIDresult / abs(PIDresult) * 200;
	}
	return (int16_t) PIDresult;
//TODO:function to normalize this value to analogWrite();
}

void PID::updateErrors(float desired, float real) {
	this->ProportionalPrev.setValue(this->Proportional.getValue());
	this->Proportional.setValue(desired - real);
	this->Integral.next(this->Proportional);
	this->Derivative.next(this->Proportional, this->ProportionalPrev);
//	Serial.println(
//			"ProportionalPrev:" + String(ProportionalPrev.getValue()) +
//			" Proportional:" + String(Proportional.getValue()) +
//			" Integral:" + String(Integral.getValue()) +
//			" Derivative:" + String(Derivative.getValue()));
}

PID::PID(float p, float i, float d) {
	this->p = p;
	this->i = i;
	this->d = d;
	this->ProportionalPrev.setType(P_TYPE);
	this->ProportionalPrev.setValue(0);
	this->Proportional.setType(P_TYPE);
	this->Proportional.setValue(0.0);
	this->Integral.setType(I_TYPE);
	this->Integral.setValue(0.0);
	this->Derivative.setType(D_TYPE);
	this->timeNew = millis();
	this->timePrev = millis();

}

void PID::apply(int16_t PIDresult) {
	digitalWrite(this->Motor.getDirPin(), PIDresult < 0);
	analogWrite(this->Motor.getStepPin(), abs(PIDresult));
}

void PID::setD(float d) {
	this->d = d;
}

void PID::setI(float i) {
	this->i = i;
}

void PID::setP(float p) {
	this->p = p;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

