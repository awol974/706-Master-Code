#pragma once

//#define NO_READ_MPU //Uncomment of MPU is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int SONAR_TRIG_PIN = 48;
const int SONAR_ECHO_PIN = 49;

//IR Sensor pins
#define LONG_IR_FRONT_PIN A4
#define LONG_IR_REAR_PIN A5
#define MED_IR_LEFT_PIN A6
#define MED_IR_RIGHT_PIN A7

// Anything over 400 cm (23200 us pulse) is "out of range". Hint:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

//this value is multiplied by each motor gain in order to control speed of the robot
int speed_val = 10;

//custom motor gains
int left_front_gain = 12;
int right_front_gain = 10;
int left_rear_gain = 10;
int right_rear_gain = 10;

//Gyro parameters
//High pass filter cut off value
int gyroCutOff = 30;
int turnAngle = 82;
