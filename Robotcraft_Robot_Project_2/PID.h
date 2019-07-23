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
#define MILLISECONDS /1000.0
//turns to second
class PID {
private:
	PID();
	float p,i,d;
	uint32_t timePrev;
	uint32_t timeNew;

    Error Proportional;
    Error ProportionalPrev;
    Error Integral;
    Error Derivative;
public:
	PID(float p,float i,float d);
	void updateErrors(float desired,float real);
	float calc(float desired,float real);
	uint8_t normalize(float PIDresult);
	virtual ~PID();
	void setP(float p);
	void setI(float i);
	void setD(float d);

};

#endif /* PID_H_ */
