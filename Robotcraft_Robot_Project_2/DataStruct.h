/*
 * DataStruct.h
 *
 *  Created on: 18 ???. 2019 ?.
 *      Author: curab
 */

#ifndef DATASTRUCT_H_
#define DATASTRUCT_H_

#include <stdint.h>


struct Data{
	uint8_t distance[3];
	int32_t encoders[2];
};
struct Velocity {
	Velocity() {
		v = 0;
		w = 0;
	}
	float v, w;
};
struct Position {
	Position() {
		x = 0;
		y = 0;
		t = 0;
	}
	float x, y, t;
};
struct Enc {
	int32_t left, right;
};
#endif /* DATASTRUCT_H_ */
