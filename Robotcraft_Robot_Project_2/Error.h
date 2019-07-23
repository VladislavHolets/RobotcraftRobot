/*
 * Error.h
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#ifndef ERROR_H_
#define ERROR_H_

#include <Arduino.h>
#include <stdint.h>

#define P_TYPE 0
#define I_TYPE 1
#define D_TYPE 2

class Error {
private:

	uint8_t type;
	float value;
public:
	Error();
	Error( uint8_t type);
	void setValue(float value);
	void setType(uint8_t type);
	float getValue();
	void next(Error now, Error prev = NULL);
	virtual ~Error();
};

#endif /* ERROR_H_ */
