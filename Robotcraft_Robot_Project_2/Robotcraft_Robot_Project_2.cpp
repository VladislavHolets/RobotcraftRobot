#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>

#include "Constants.h"
#include "Encoder.h"
#include "RangeSensor.h"
#include "Robot.h"
#include "TimerFive.h"
#include "Wheel.h"

std_msgs::Float32 front_distance_msg_, left_distance_msg_, right_distance_msg_;
geometry_msgs::Pose2D pose_msg_;

ros::Publisher front_distance_pub_("front_distance", &front_distance_msg_, 2),
		left_distance_pub_("left_distance", &left_distance_msg_, 2),
		right_distance_pub_("right_distance", &right_distance_msg_, 2),
		pose_pub_("pose", &pose_msg_, 2);

void rgbLedsMsgCallback(const std_msgs::UInt8MultiArray& msg);
void cmdVelMsgCallback(const geometry_msgs::Twist& msg);
void setPoseMsgCallback(const geometry_msgs::Pose2D& msg);

ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub_("rgb_leds",
		rgbLedsMsgCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub_("cmd_vel",
		cmdVelMsgCallback);
ros::Subscriber<geometry_msgs::Pose2D> set_pose_sub_("set_pose",
		setPoseMsgCallback);

ros::NodeHandle node_handler_;

Robot Odin(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, FRONT_SENSOR_PIN,
		LEFT_MOTOR_ENCODER_1_PIN, LEFT_MOTOR_ENCODER_2_PIN, LEFT_P, LEFT_I,
		LEFT_D, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_STEP_PIN,
		RIGHT_MOTOR_ENCODER_1_PIN, RIGHT_MOTOR_ENCODER_2_PIN, RIGHT_P, RIGHT_I,
		RIGHT_D, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_STEP_PIN, NEOPIXEL_PIN,
		NEOPIXEL_NUM);

void TimerISR();
//TODO: create class Motor, compose Encoder object into Motor Object so that it will be possible to create setPosition() function.
void setup() {
	node_handler_.initNode();
	node_handler_.getHardware()->setBaud(BAUD);
	node_handler_.advertise(pose_pub_);
	node_handler_.advertise(right_distance_pub_);
	node_handler_.advertise(front_distance_pub_);
	node_handler_.advertise(left_distance_pub_);
	node_handler_.subscribe(rgb_leds_sub_);
	node_handler_.subscribe(cmd_vel_sub_);
	node_handler_.subscribe(set_pose_sub_);

	delay(2000);
	Odin.leftWheel.encoder.write(0);
	Odin.rightWheel.encoder.write(0);
	Odin.leftWheel.encoder.updateChange();
	Odin.rightWheel.encoder.updateChange();
	Timer5.initialize(100000);
	Timer5.attachInterrupt(TimerISR);

}

void loop() {
	geometry_msgs::Twist velocity;
	velocity.linear.x = 0.05;
}

inline void rgbLedsMsgCallback(const std_msgs::UInt8MultiArray& msg) {
	Odin.pixels.setPixelColor(0, msg.data[0], msg.data[1], msg.data[2]);
	Odin.pixels.setPixelColor(1, msg.data[3], msg.data[4], msg.data[5]);
}

inline void cmdVelMsgCallback(const geometry_msgs::Twist& msg) {
	Odin.setDesiredVelocity(msg);
}

inline void setPoseMsgCallback(const geometry_msgs::Pose2D& msg) {
	Odin.setPosition(msg);
}

void TimerISR() {
	Odin.leftWheel.encoder.write(Odin.leftWheel.encoder.read() + 200);
	Odin.rightWheel.encoder.write(Odin.rightWheel.encoder.read() + 200);
	Odin.leftWheel.encoder.updateChange();
	Odin.rightWheel.encoder.updateChange();

	Odin.updateWheelsVelocities();
	Odin.updateRobotPose();
	Odin.updateWheelsPID();

	front_distance_msg_.data = Odin.frontSensor.getDistance();
	left_distance_msg_.data = Odin.leftSensor.getDistance();
	right_distance_msg_.data = Odin.rightSensor.getDistance();

	front_distance_pub_.publish(&front_distance_msg_);
	left_distance_pub_.publish(&left_distance_msg_);
	right_distance_pub_.publish(&right_distance_msg_);

	pose_pub_.publish(&Odin.getPosition());

}

