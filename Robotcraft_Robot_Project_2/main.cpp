#define ROSSERIAL_ARDUINO_TCP 1
#define ENC28J60 1
#include <Adafruit_MCP4728.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoTcpHardware.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <HardwareSerial.h>
#include <IPAddress.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros.h>
#include <rosserial_arduino/Test.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <stdint.h>

#include "Constants.h"
#include "Encoder.h"
#include "RangeSensor.h"
#include "Robot.h"
#include "Wheel.h"

byte mac[] = {0xDE, 0xEE, 0xEE, 0xAD, 0xEF, 0x01};
//TODO change ip of the server;
// Set the rosserial socket server IP address
IPAddress server(192, 168, 90, 131);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

Adafruit_MCP4728 DAC;

ros::NodeHandle node_handler_;

Robot Odin(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, FRONT_SENSOR_PIN, LEFT_MOTOR_ENCODER_1_PIN, LEFT_MOTOR_ENCODER_2_PIN,
           LEFT_P, LEFT_I, LEFT_D, LEFT_MOTOR_DIR_PIN, &DAC, MCP4728_CHANNEL_B, /*LEFT_MOTOR_STEP_PIN,*/
           RIGHT_MOTOR_ENCODER_1_PIN, RIGHT_MOTOR_ENCODER_2_PIN, RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_MOTOR_DIR_PIN, &DAC,
           MCP4728_CHANNEL_A,/* RIGHT_MOTOR_STEP_PIN,*/
           NEOPIXEL_PIN, NEOPIXEL_NUM, BEEP_PIN);

std_msgs::Float32 front_distance_msg_, left_distance_msg_, right_distance_msg_;

//std_msgs::Float32 left_encoder_msg_, right_encoder_msg_, time_msg_;

geometry_msgs::Pose2D pose_msg_;

ros::Publisher front_distance_pub_("front_distance", &front_distance_msg_), left_distance_pub_("left_distance",
                                                                                               &left_distance_msg_),
               right_distance_pub_("right_distance", &right_distance_msg_), pose_pub_("pose", &pose_msg_);
/*left_encoder_pub_("left_encoder_values",
 &left_encoder_msg_), right_encoder_pub_("right_encoder_values",
 &right_encoder_msg_), time_pub_("interupt_time", &time_msg_);*/

void rgbLedsMsgCallback(const std_msgs::UInt8MultiArray &msg);
void cmdVelMsgCallback(const geometry_msgs::Twist &msg);
void setPoseMsgCallback(const geometry_msgs::Pose2D &msg);

ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub_("rgb_leds", rgbLedsMsgCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub_("cmd_vel", cmdVelMsgCallback);
ros::Subscriber<geometry_msgs::Pose2D> set_pose_sub_("set_pose", setPoseMsgCallback);

void beepSrvCallback(const rosserial_arduino::Test::Request &request, rosserial_arduino::Test::Response &response);

//ros::ServiceServer<rosserial_arduino::Test::Request, rosserial_arduino::Test::Response> beep_srv_("arduino/beep_srv",&beepSrvCallback);

void TimerISR();
//TODO: create class Motor, compose Encoder object into Motor Object so that it will be possible to create setPosition() function.
void setup()
{
  Serial.begin(9600);
  Serial.println("Starting Ethernet");

  Ethernet.begin(mac);

  Serial.println("Starting Ethernet");

  node_handler_.getHardware()->setConnection(server, serverPort);
  node_handler_.initNode();

  Serial.println("Starting Ethernet");
//  Odin.pixels.begin();
//  Odin.pixels.show();
//  node_handler_.getHardware()->setBaud(BAUD);
//  node_handler_.initNode();
  node_handler_.advertise(pose_pub_);
  node_handler_.advertise(right_distance_pub_);
  node_handler_.advertise(front_distance_pub_);
  node_handler_.advertise(left_distance_pub_);
  //node_handler_.advertise(left_encoder_pub_);
  //node_handler_.advertise(right_encoder_pub_);
  //node_handler_.advertise(time_pub_);
  node_handler_.subscribe(rgb_leds_sub_);
  node_handler_.subscribe(cmd_vel_sub_);
  node_handler_.subscribe(set_pose_sub_);
  // node_handler_.advertiseService(beep_srv_);
  Odin.leftWheel.encoder.write(0);
  Odin.rightWheel.encoder.write(0);
  Odin.leftWheel.encoder.updateChange();
  Odin.rightWheel.encoder.updateChange();
//  Timer5.initialize(100000);
//  Timer5.attachInterrupt(TimerISR);
  Serial.begin(9600);
  int32_t resultL = 0, resultR = 0;
  while (1)
  {
    Odin.leftWheel.encoder.updateChange();
    Serial.print("LEFT:");
    resultL += Odin.leftWheel.encoder.getChange();
    Serial.println(resultL);

    Odin.rightWheel.encoder.updateChange();
    Serial.print("RIGHT:");
    resultR += Odin.rightWheel.encoder.getChange();
    Serial.println(resultR);

  }
//	digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH);
//	digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);
//	analogWrite(LEFT_MOTOR_STEP_PIN, 255);
//	analogWrite(RIGHT_MOTOR_STEP_PIN, 255);

}
uint32_t timeStamp = 0;
void loop()
{
  timeStamp = millis();
  //ros::Rate loop_rate(10);

  front_distance_msg_.data = Odin.frontSensor.getDistance();
  left_distance_msg_.data = Odin.leftSensor.getDistance();
  right_distance_msg_.data = Odin.rightSensor.getDistance();

  front_distance_pub_.publish(&front_distance_msg_);
  left_distance_pub_.publish(&left_distance_msg_);
  right_distance_pub_.publish(&right_distance_msg_);

  pose_pub_.publish(&Odin.getPosition());

  while ((millis() - timeStamp) < 100)
  {
    node_handler_.spinOnce();
  }
}

inline void rgbLedsMsgCallback(const std_msgs::UInt8MultiArray &msg)
{
  Odin.pixels.setPixelColor(0, msg.data[0], msg.data[1], msg.data[2]);
  Odin.pixels.setPixelColor(1, msg.data[3], msg.data[4], msg.data[5]);
}

inline void cmdVelMsgCallback(const geometry_msgs::Twist &msg)
{
  Odin.setDesiredVelocity(msg);
}

inline void setPoseMsgCallback(const geometry_msgs::Pose2D &msg)
{
  Odin.setPosition(msg);
}

inline void beepSrvCallback(const rosserial_arduino::Test::Request &request,
                            rosserial_arduino::Test::Response &response)
{
  digitalWrite(Odin.beepPin, !digitalRead(Odin.beepPin));
  if (digitalRead(Odin.beepPin))
  {
    response.output = "1";
  }
  else
  {
    response.output = "0";
  }
}

void TimerISR()
{
  //uint32_t timestamp = millis();
  Odin.leftWheel.encoder.updateChange();
  Odin.rightWheel.encoder.updateChange();
  //left_encoder_msg_.data = Odin.leftWheel.encoder.getChange();
  //right_encoder_msg_.data = Odin.rightWheel.encoder.getChange();
  Odin.updateWheelsVelocities();
  Odin.updateRobotPose();
  Odin.updateWheelsPID();
//	time_msg_.data = millis() - timestamp;
//	time_pub_.publish(&time_msg_);
}

