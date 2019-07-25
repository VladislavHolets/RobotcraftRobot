#include <Arduino.h>
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
struct Velocity {
	Velocity() {
		v = 0;
		w = 0;
	}
	float v, w;
} robotC = Velocity(), robotD = Velocity();
struct Position {
	Position() {
		x = 0;
		y = 0;
		t = 0;
	}
	float x, y, t;
} pos = Position();
struct Enc {
	float left, right;
} enc;

volatile struct Data previous;
volatile struct Data current;

RangeSensor LeftS(LEFT_SENSOR_PIN);
RangeSensor RightS(RIGHT_SENSOR_PIN);
RangeSensor FrontS(FRONT_SENSOR_PIN);

Encoder LeftM(LEFT_MOTOR_ENCODER_2_PIN, LEFT_MOTOR_ENCODER_1_PIN);
Encoder RightM(RIGHT_MOTOR_ENCODER_1_PIN, RIGHT_MOTOR_ENCODER_2_PIN);

uint32_t timePrev = millis();
uint32_t timeNew = millis();

float leftWC = 0, rightWC = 0, leftWD = 0, rightWD = 0;

void encUpdate();
void TimerISR();
//TODO: create class Motor, compose Encoder object into Motor Object so that it will be possible to create setPosition() function.
void setup() {
	LeftM.write(0);
	RightM.write(0);
	Serial.begin(115200);
	delay(2000);
	timeNew = millis();
	pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
	pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
//while(LeftM.read()<298*12*2.33){
//	analogWrite(LEFT_MOTOR_PIN,120);
//}
//analogWrite(LEFT_MOTOR_PIN,0);
	Timer5.initialize(100000);
	  Timer5.attachInterrupt(TimerISR);

}

void loop() {
//	analogWrite(LEFT_MOTOR_PIN, SPEED);
//	analogWrite(RIGHT_MOTOR_PIN, SPEED);

//	if(Serial.available()>DATA_MIN_SIZE){
//		Msg.getData();
//	robotD.v=Msg.getV();
//	robotD.W=Msg.getW();
//	}
//	/\ /\ /\ /\ this is how it is to be finally

	//robotD.w = 0.01;
	robotD.v = 0.03;
	//delay(5000);
	//robotD.v = 0.04;
	//delay(5000);
}

void TimerISR() {
//	current.distance[FRONT] = FrontS.getDistance();
//	current.distance[LEFT] = LeftS.getDistance();
//	current.distance[RIGHT] = RightS.getDistance();

	previous.encoders[LEFT] = current.encoders[LEFT];
	previous.encoders[RIGHT] = current.encoders[RIGHT];

	current.encoders[LEFT] = LeftM.read();
	current.encoders[RIGHT] = RightM.read();

	timePrev = timeNew;
	timeNew = millis();

	encUpdate();

	float deltaT = (100.0)MILLISECONDS;
//define from PID.h
	//Serial.println("e.l:" + String(enc.left) + "e.r:" + String(enc.right));
	robotC.v = TWO_PI * ROBOT_R * (enc.left + enc.right) / ROBOT_C / 2.0
			/  0.1;
	robotC.w = TWO_PI * ROBOT_R * (enc.right - enc.left) / ROBOT_C / 2.0
			/ 0.1;
	pos.t = atan2(sin(pos.t + robotC.w * deltaT),
			cos(pos.t + robotC.w * deltaT));
	pos.x = pos.x + robotC.v * deltaT * cos(pos.t);
	pos.y = pos.y + robotC.v * deltaT * sin(pos.t);
	leftWC = (robotC.v - (ROBOT_B )/ 2 * robotC.w) / ROBOT_R;
	rightWC = (robotC.v + (ROBOT_B )/ 2 * robotC.w) / ROBOT_R;
//	Serial.println(
//			"robotC.v" + String(robotC.v) + "robotC.w" + String(robotC.w));

//comment if ROS Active
//	leftWC=300;
//	rightWC=300;

	leftWD = (robotD.v - ROBOT_B / 2 * robotD.w) / ROBOT_R;
	rightWD = (robotD.v + ROBOT_B / 2 * robotD.w) / ROBOT_R;

//	analogWrite(LEFT_MOTOR_PIN, 255);
//	analogWrite(RIGHT_MOTOR_PIN, 255);

//	leftWD = 7.20;
//	rightWD = 7.20 ;
//	Serial.println("lD:" + String(leftWD) + "lC:" + String(leftWC));
//	Serial.print("L:");
	int16_t forceL = PIDL.normalize(PIDL.calc(leftWD, leftWC));
	digitalWrite(LEFT_MOTOR_DIR_PIN, forceL < 0);
	analogWrite(LEFT_MOTOR_PIN, abs(forceL));
//	Serial.println("rD:" + String(rightWD) + "rC:" + String(rightWC));
//	Serial.print("R:");
	int16_t forceR = PIDR.normalize(PIDR.calc(rightWD, rightWC));
	digitalWrite(RIGHT_MOTOR_DIR_PIN, forceL < 0);
	analogWrite(RIGHT_MOTOR_PIN, abs(forceR));
//	Serial.println(
//			"lf:" + String(forceL) + " rf:" + String(forceR));
	Serial.println(
			"X:" + String(pos.x) + " Y:" + String(pos.y) + " T:"+ String(pos.t));

}
void encUpdate() {
	enc.left = current.encoders[LEFT] - previous.encoders[LEFT];
	enc.right = current.encoders[RIGHT] - previous.encoders[RIGHT];
}
