/*
 * DataStruct.h
 *
 *  Created on: 18 ???. 2019 ?.
 *      Author: curab
 */

#ifndef DATASTRUCT_H_
#define DATASTRUCT_H_

#include <stdint.h>

#define LEFT 0
#define RIGHT 1
#define FRONT 2
struct Data{
	uint8_t distance[3];
	int32_t encoders[2];
};


#endif /* DATASTRUCT_H_ */
