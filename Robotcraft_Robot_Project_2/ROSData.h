/*
 * ROSData.h
 *
 *  Created on: 23 ???. 2019 ?.
 *      Author: curab
 */

#ifndef ROSDATA_H_
#define ROSDATA_H_

class ROSData {
	float v,w;
	void setV(float v);
	void setW(float w);
public:
	ROSData();
	void getData();
	virtual ~ROSData();
	float getV() const;
	float getW() const;
};

#endif /* ROSDATA_H_ */
