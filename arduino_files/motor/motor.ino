#include <ros.h>

#include <mst_control/Velocity.h>
#include <std_msgs/UInt8.h>

/*******************************************************************************
 * Variables
 *******************************************************************************/

uint8_t lightMode = 0;

ros::NodeHandle nh;

/*******************************************************************************
 * Constants
 *******************************************************************************/

//Pin configurations
const int LEFT_VELOCITY = 10;
const int RIGHT_VELOCITY = 11;
const int LEFT_DIRECTION = 12;
const int RIGHT_DIRECTION = 13;
const int LIGHT = 7;

// Callback for Velocity which gets the left and right wheel velocity and direction
void VelocityCallback(const mst_control::Velocity &msg) {
    analogWrite(LEFT_VELOCITY, msg.left_vel);
    analogWrite(RIGHT_VELOCITY, msg.right_vel);
    digitalWrite(LEFT_DIRECTION, msg.left_dir);
    digitalWrite(RIGHT_DIRECTION, !msg.right_dir);
}

//Callback from LIGHT msg, changes mode of the LIGHT
void LightCallback(const std_msgs::UInt8 &msg) {
    lightMode = msg.data;
}

void updateLight() {
    
}

ros::Subscriber<mst_control::Velocity> velocitySub("cmd_vel", &VelocityCallback);
ros::Subscriber<std_msgs::UInt8> lightSub("indicator_light", &LightCallback);

void setup() {
    pinMode(LEFT_VELOCITY, OUTPUT);
    pinMode(RIGHT_VELOCITY, OUTPUT);
    pinMode(LEFT_DIRECTION, OUTPUT);
    pinMode(RIGHT_DIRECTION, OUTPUT);
    pinMode(LIGHT, OUTPUT);
    
    digitalWrite(LIGHT, LOW);
    
    nh.initNode();
    nh.subscribe(velocitySub);
    nh.subscribe(lightSub);
}

void loop() {
    nh.spinOnce();
    delay(10);
}

