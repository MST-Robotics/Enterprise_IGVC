/******************************************************************************
* @file moonshot.ino
* @brief The arduino code for moonshot's microcontroller. It is responsible
*           for controlling the motors, blinking the indicator light, 
*           and publishing IMU messages from the sensor stick found at 
*           https://www.sparkfun.com/products/10724.
* @author Matt Anderson <mia2n4>
* @author Islam Elnabarawy <ie3md>
* @version 1.0.0
******************************************************************************/

#include <ros.h>

#include <control/Velocity.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Quaternion.h>

#include <Wire.h>


/*******************************************************************************
 * Constants
 *******************************************************************************/
// Pin configurations
const int PIN_ENABLE_FL = 24;
const int PIN_ENABLE_FR = 26;
const int PIN_ENABLE_BL = 28;
const int PIN_ENABLE_BR = 30;
const int PIN_ENABLE_CON = 32;

const int PIN1_SET_FL = 2;
const int PIN2_SET_FL = 3;
const int PIN1_SET_FR = 4;
const int PIN2_SET_FR = 5;
const int PIN1_SET_BL = 6;
const int PIN2_SET_BL = 7;
const int PIN1_SET_BR = 8;
const int PIN2_SET_BR = 9;
const int PIN1_SET_CON = 10;
const int PIN2_SET_CON = 11;

const int PIN_LIGHT = 7;

// light update interval
const unsigned long LIGHT_INTERVAL = 1000; // milliseconds

// IMU update interval
const unsigned long IMU_INTERVAL = 500; // milliseconds

// IMU magnetometer address
const int ADDR_MAGNETOMETER = 0x1E;


/*******************************************************************************
 * Forward delerations
 *******************************************************************************/

void VelocityCallback(const control::Velocity &msg);
void LightCallback(const std_msgs::UInt8 &msg);


/*******************************************************************************
 * Variables
 *******************************************************************************/

// Light variables:

// light blinking mode (0 for solid and 1 for blinking)
uint8_t lightMode = 0;

// millisecond count when the last light blink occured [see: millis()]
unsigned long lightLastMillis = 0;

// current state of the light
bool lightState = LOW;


// IMU variables:

// Two arrays to store the last 5 x and y values for averaging
int x_trail[5] = {0, 0, 0, 0, 0};
int y_trail[5] = {0, 0, 0, 0, 0};

// The index of the oldest value in the array to be averaged 
// (this one will be replaced next)
int trail_index = 0;

// millisecond count when the last IMU update occured [see: millis()]
unsigned long imuLastMillis = 0;


// ROS objects:

// ROS node handle
ros::NodeHandle nodeHandle;

// ROS message used for publishing the IMU data
geometry_msgs::Quaternion imuMessage;

// ROS publisher for IMU message
ros::Publisher imuPub("imu", &imuMessage);

// ROS Subscriber for Velocity messages
ros::Subscriber<control::Velocity> velocitySub("cmd_vel", &VelocityCallback);

// ROS Subscriber for Light messages
ros::Subscriber<std_msgs::UInt8> lightSub("indicator_light", &LightCallback);


/*******************************************************************************
 * Callbacks
 *******************************************************************************/

// Callback for Velocity, gets the left and right wheel velocity and direction
void VelocityCallback(const control::Velocity &msg) {
  //Left motor not being used
  if(msg.left_vel == 0)
  {
    digitalWrite(PIN_ENABLE_FL, LOW);
    digitalWrite(PIN_ENABLE_BL, LOW);
  }
  //Compute direction and velocities of left motors
  else
  {
    digitalWrite(PIN_ENABLE_FL, HIGH);
    digitalWrite(PIN_ENABLE_BL, HIGH);
    if(!msg.left_dir)
    {
      analogWrite(PIN1_SET_FL, msg.left_vel);
      analogWrite(PIN2_SET_FL, 0);
      analogWrite(PIN1_SET_BL, msg.left_vel);
      analogWrite(PIN2_SET_BL, 0);
    }
    else
    {
      analogWrite(PIN1_SET_FL, 0);
      analogWrite(PIN2_SET_FL, msg.left_vel);
      analogWrite(PIN1_SET_BL, 0);
      analogWrite(PIN2_SET_BL, msg.left_vel);
    }
  }
  
  //Right motor not being used
  if(msg.right_vel == 0)
  {
    digitalWrite(PIN_ENABLE_FR, LOW);
    digitalWrite(PIN_ENABLE_BR, LOW);
  }
  //Compute direction and velocities of left motors
  else
  {
    digitalWrite(PIN_ENABLE_FR, HIGH);
    digitalWrite(PIN_ENABLE_BR, HIGH);
    if(!msg.left_dir)
    {
      analogWrite(PIN1_SET_FR, msg.right_vel);
      analogWrite(PIN2_SET_FR, 0);
      analogWrite(PIN1_SET_BR, msg.right_vel);
      analogWrite(PIN2_SET_BR, 0);
    }
    else
    {
      analogWrite(PIN1_SET_FR, 0);
      analogWrite(PIN2_SET_FR, msg.right_vel);
      analogWrite(PIN1_SET_BR, 0);
      analogWrite(PIN2_SET_BR, msg.right_vel);
    }
  }
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

void updateIMU() {
    unsigned long currentMillis = millis();
     
    if((currentMillis - imuLastMillis) > IMU_INTERVAL) {
        // save the last time we updated the IMU 
        imuLastMillis = currentMillis;
        
        int magx, magy, magz;
        
        Wire.beginTransmission(ADDR_MAGNETOMETER);
        Wire.write(0x03);
        Wire.endTransmission();
        
        //Recieve data from magnetometer
        Wire.requestFrom(ADDR_MAGNETOMETER, 6);
        if(Wire.available() >= 6)
        {
            magx = Wire.read() << 8;
            magx |= Wire.read();
            magz = Wire.read() << 8;
            magz |= Wire.read();
            magy = Wire.read() << 8;
            magy |= Wire.read();
        }
    
        x_trail[trail_index] = magx;
        y_trail[trail_index] = magy;
        trail_index++;
        if(trail_index == 5)
            trail_index = 0;
        
        // The x an y values are a moving average over the last 5 data points
        imuMessage.x = (x_trail[0] + x_trail[1] + x_trail[2] + x_trail[3] + x_trail[4]) / 5.0;
        imuMessage.y = (y_trail[0] + y_trail[1] + y_trail[2] + y_trail[3] + y_trail[4]) / 5.0;
        
        //publish message
        imuPub.publish(&imuMessage);
    }
}


/*******************************************************************************
 * Initial Arduino Setup
 *******************************************************************************/

void setup() {
    // Setup pins
    pinMode(PIN_LEFT_VELOCITY, OUTPUT);
    pinMode(PIN_RIGHT_VELOCITY, OUTPUT);
    pinMode(PIN_LEFT_DIRECTION, OUTPUT);
    pinMode(PIN_RIGHT_DIRECTION, OUTPUT);
    pinMode(PIN_LIGHT, OUTPUT);
    
    // Light starts out as off
    digitalWrite(PIN_LIGHT, LOW);
    
    // Setup ROS node and topics
    nodeHandle.initNode();
    nodeHandle.subscribe(velocitySub);
    nodeHandle.subscribe(lightSub);
    nodeHandle.advertise(imuPub);
    
    // Setup magnetometer
    Wire.begin();
    Wire.beginTransmission(ADDR_MAGNETOMETER);
    Wire.write(0x02); //select mode register
    Wire.write(0x00); //continuos measurement
    Wire.endTransmission();
}


/*******************************************************************************
 * Main Arduino Loop
 *******************************************************************************/

void loop() {
    nodeHandle.spinOnce();
    updateIMU();
    updateLight();
}

