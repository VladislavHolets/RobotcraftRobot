/*
 * PID.h
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#include "Error.h"
#include "Motor.h"

//turns to second
class PID {

private:
	const uint8_t P_TYPE = 0;
	const uint8_t I_TYPE = 1;
	const uint8_t D_TYPE = 2;
	float p, i, d;
	uint32_t timePrev;
	uint32_t timeNew;

	Error Proportional;
	Error ProportionalPrev;
	Error Integral;
	Error Derivative;
	Motor Motor;
public:
	//old
	PID(float p, float i, float d);
	//new
	PID(float p, float i, float d, uint8_t dirPin, uint8_t stepPin);
	void apply(int16_t PIDresult);
	//old
	void updateErrors(float desired, float real);
	float calc(float desired, float real);

	int16_t normalize(float PIDresult);
	virtual ~PID();
	void setP(float p);
	void setI(float i);
	void setD(float d);

};

#endif /* PID_H_ */
