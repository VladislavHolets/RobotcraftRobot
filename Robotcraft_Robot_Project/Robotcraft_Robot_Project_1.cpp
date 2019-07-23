#include <Arduino.h>

#include "DataStruct.h"
#include "TimerFive.h"
#include "Definitions.h"
#include "Encoder.h"
#include "RangeSensor.h"


volatile struct Data previous;
volatile struct Data current;

RangeSensor LeftS(LEFT_SENSOR_PIN);
RangeSensor RightS(RIGHT_SENSOR_PIN);
RangeSensor FrontS(FRONT_SENSOR_PIN);

Encoder LeftM(LEFT_MOTOR_ENCODER_2_PIN, LEFT_MOTOR_ENCODER_1_PIN);
Encoder RightM(RIGHT_MOTOR_ENCODER_1_PIN, RIGHT_MOTOR_ENCODER_2_PIN);
void TimerISR();
//TODO: create class Motor, compose Encoder object into Motor Object so that it will be possible to create setPosition() function.
void setup() {
	LeftM.write(0);
	RightM.write(0);
	Serial.begin(115200);
	Timer5.initialize(100000);
	Timer5.attachInterrupt(TimerISR);
	delay(2000);
}

void loop() {
//	Serial.print("\n\nLeftS.getValue:");
//	Serial.println(LeftS.getValue());
//	Serial.print("FrontS.getValue:");
//	Serial.println(FrontS.getValue());
//	Serial.print("RightS.getValue:");
//	Serial.println(RightS.getValue());
	analogWrite(4, 127);
	analogWrite(9, 127);
//	Serial.print("LeftM.read:");
//	Serial.println(LeftM.read());
//	Serial.print("RightM.read:");
//	Serial.println(RightM.read());
//	delay(500);
}

void TimerISR() {

	previous.encoders[LEFT] = current.encoders[LEFT];
	previous.encoders[RIGHT] = current.encoders[RIGHT];

	current.distance[FRONT] = FrontS.getDistance();
	current.distance[LEFT] =  LeftS.getDistance();
	current.distance[RIGHT] = RightS.getDistance();

	current.encoders[LEFT] = LeftM.read();
	current.encoders[RIGHT] = RightM.read();

	String msg = " : " +String(current.distance[LEFT])
			+ " : " + String(current.distance[FRONT])
			+ " : " + String(current.distance[RIGHT])
			+ " : " + String(current.encoders[LEFT] - previous.encoders[LEFT])
			+ " : " + String(current.encoders[RIGHT]- previous.encoders[RIGHT]);
	Serial.println(msg);
}
