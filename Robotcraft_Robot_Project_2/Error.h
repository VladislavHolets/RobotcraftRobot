/*
 * Error.h
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#ifndef ERROR_H_
#define ERROR_H_

#include <stdint.h>

class Error {
private:
	const uint8_t P_TYPE = 0;
	const uint8_t I_TYPE = 1;
	const uint8_t D_TYPE = 2;
	uint8_t type;
	float value;
public:
	Error();
	Error(uint8_t type);
	void setValue(float value);
	void setType(uint8_t type);
	float getValue();
	void next(Error now, Error prev = 0);
	virtual ~Error();
};

#endif /* ERROR_H_ */
