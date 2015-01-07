// motor.ino
// Arduino sketch for controlling the motors and blinking the indicator light

#include <ros.h>

#include <mst_control/Velocity.h>
#include <std_msgs/UInt8.h>


/*******************************************************************************
 * Variables
 *******************************************************************************/

uint8_t lightMode = 0;
unsigned long lightLastMillis = 0;
bool lightState = LOW;

ros::NodeHandle nodeHandle;


/*******************************************************************************
 * Constants
 *******************************************************************************/

// Pin configurations
const int PIN_LEFT_VELOCITY = 10;
const int PIN_RIGHT_VELOCITY = 11;
const int PIN_LEFT_DIRECTION = 12;
const int PIN_RIGHT_DIRECTION = 13;
const int PIN_LIGHT = 7;

// light update interval
const unsigned long LIGHT_INTERVAL = 1000; // 1000 ms = 1 second


/*******************************************************************************
 * Callbacks
 *******************************************************************************/

// Callback for Velocity which gets the left and right wheel velocity and direction
void VelocityCallback(const mst_control::Velocity &msg) {
    analogWrite(PIN_LEFT_VELOCITY, msg.left_vel);
    analogWrite(PIN_RIGHT_VELOCITY, msg.right_vel);
    digitalWrite(PIN_LEFT_DIRECTION, !msg.left_dir);
    digitalWrite(PIN_RIGHT_DIRECTION, msg.right_dir);
}

// Callback from light msg, changes mode of the light
void LightCallback(const std_msgs::UInt8 &msg) {
    lightMode = msg.data;
}


/*******************************************************************************
 * Update Methods
 *******************************************************************************/

void updateLight() {
    if (lightMode == 1) {
        // blink without delay
        // based on: http://arduino.cc/en/Tutorial/BlinkWithoutDelay
        unsigned long currentMillis = millis();
         
        if((currentMillis - lightLastMillis) > LIGHT_INTERVAL) {
            // save the last time we blinked the LED 
            lightLastMillis  = currentMillis;
            
            // if the light is off turn it on and vice-versa:
            if (lightState == LOW)
                lightState = HIGH;
            else
                lightState = LOW;
            
            // set the output value on the light pin
            digitalWrite(PIN_LIGHT, lightState);
        }
    } else {
        // make sure the light is on solid
        if (lightState == LOW) {
            lightState = HIGH;
            digitalWrite(PIN_LIGHT, lightState);
        }
    }
}


/*******************************************************************************
 * Initial Setup
 *******************************************************************************/

ros::Subscriber<mst_control::Velocity> velocitySub("cmd_vel", &VelocityCallback);
ros::Subscriber<std_msgs::UInt8> lightSub("indicator_light", &LightCallback);

void setup() {
    pinMode(PIN_LEFT_VELOCITY, OUTPUT);
    pinMode(PIN_RIGHT_VELOCITY, OUTPUT);
    pinMode(PIN_LEFT_DIRECTION, OUTPUT);
    pinMode(PIN_RIGHT_DIRECTION, OUTPUT);
    pinMode(PIN_LIGHT, OUTPUT);
    
    digitalWrite(PIN_LIGHT, LOW);
    
    nodeHandle.initNode();
    nodeHandle.subscribe(velocitySub);
    nodeHandle.subscribe(lightSub);
}


/*******************************************************************************
 * Main Loop
 *******************************************************************************/

void loop() {
    nodeHandle.spinOnce();
    updateLight();
}

