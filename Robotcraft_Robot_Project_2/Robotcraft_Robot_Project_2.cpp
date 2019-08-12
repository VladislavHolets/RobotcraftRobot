#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <stdint.h>
#include <WString.h>

#include "DataStruct.h"
#include "Definitions.h"
#include "Encoder.h"
#include "PID.h"
#include "RangeSensor.h"
#include "ROSData.h"
#include "TimerFive.h"

ROSData Msg;

PID PIDL(LEFT_P, LEFT_I, LEFT_D);
PID PIDR(RIGHT_P, RIGHT_I, RIGHT_D);
struct Velocity  robotC = Velocity(), robotD = Velocity();
struct Position pos = Position();
struct Enc enc;

volatile struct Data previous;
volatile struct Data current;

RangeSensor LeftS(LEFT_SENSOR_PIN);
RangeSensor RightS(RIGHT_SENSOR_PIN);
RangeSensor FrontS(FRONT_SENSOR_PIN);

Encoder LeftM(LEFT_MOTOR_ENCODER_2_PIN, LEFT_MOTOR_ENCODER_1_PIN);
Encoder RightM(RIGHT_MOTOR_ENCODER_1_PIN, RIGHT_MOTOR_ENCODER_2_PIN);

float leftWC = 0, rightWC = 0, leftWD = 0, rightWD = 0;

void encUpdate();
void TimerISR();
//TODO: create class Motor, compose Encoder object into Motor Object so that it will be possible to create setPosition() function.
void setup() {
	LeftM.write(0);
	RightM.write(0);

	Serial.begin(115200);

	delay(2000);

	pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
	pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);

	Timer5.initialize(100000);
	Timer5.attachInterrupt(TimerISR);

}

void loop() {

//	if(Serial.available()>DATA_MIN_SIZE){
//		Msg.getData();
//	robotD.v=Msg.getV();
//	robotD.W=Msg.getW();
//	}
//	/\ /\ /\ /\ this is how it is to be finally (supposed)

}

void TimerISR() {
	current.distance[FRONT] = FrontS.getDistance() ;
	current.distance[LEFT] = LeftS.getDistance();
	current.distance[RIGHT] = RightS.getDistance();

	previous.encoders[LEFT] = current.encoders[LEFT];
	previous.encoders[RIGHT] = current.encoders[RIGHT];

	current.encoders[LEFT] = LeftM.read();
	current.encoders[RIGHT] = RightM.read();

	encUpdate();

	robotC.v = TWO_PI * ROBOT_R * (enc.left + enc.right) / ROBOT_C / 2.0
			/  DELTA_T;
	robotC.w = TWO_PI * ROBOT_R * (enc.right - enc.left) / ROBOT_C / 2.0
			/ DELTA_T;

	pos.t = atan2(sin(pos.t + robotC.w * DELTA_T),
			cos(pos.t + robotC.w * DELTA_T));
	pos.x = pos.x + robotC.v * DELTA_T * cos(pos.t);
	pos.y = pos.y + robotC.v * DELTA_T * sin(pos.t);

	leftWC = (robotC.v - (ROBOT_B )/ 2 * robotC.w) / ROBOT_R;
	rightWC = (robotC.v + (ROBOT_B )/ 2 * robotC.w) / ROBOT_R;

	leftWD = (robotD.v - ROBOT_B / 2 * robotD.w) / ROBOT_R;
	rightWD = (robotD.v + ROBOT_B / 2 * robotD.w) / ROBOT_R;

	int16_t forceL = PIDL.normalize(PIDL.calc(leftWD, leftWC));
	digitalWrite(LEFT_MOTOR_DIR_PIN, forceL < 0);
	analogWrite(LEFT_MOTOR_PIN, abs(forceL));

	int16_t forceR = PIDR.normalize(PIDR.calc(rightWD, rightWC));
	digitalWrite(RIGHT_MOTOR_DIR_PIN, forceL < 0);
	analogWrite(RIGHT_MOTOR_PIN, abs(forceR));
	Serial.println(
			"X:" + String(pos.x) + " Y:" + String(pos.y) + " T:"+ String(pos.t));

}
void encUpdate() {
	enc.left = current.encoders[LEFT] - previous.encoders[LEFT];
	enc.right = current.encoders[RIGHT] - previous.encoders[RIGHT];
}
