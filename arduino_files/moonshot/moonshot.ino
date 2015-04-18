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
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>

//#include <Wire.h>

const int PIN_DUMP_UP1 = 31;
const int PIN_DUMP_UP2 = 35;
const int PIN_DUMP_DOWN1 = 33;
const int PIN_DUMP_DOWN2 = 37;


/*******************************************************************************
 * Constants
 *******************************************************************************/
// Pin configurations
const int PIN_ENABLE_FL = 22;  //Number 1
const int PIN_ENABLE_FR = 24;  //Number 2
const int PIN_ENABLE_BL = 26;  //Number 3
const int PIN_ENABLE_BR = 28;  //Number 4
const int PIN_ENABLE_CON = 30; //Number 5

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

const  int maxSpeed = 225;
float leftSpeedScaled;
float rightSpeedScaled;

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
void ConveyerCallback(const std_msgs::Int16 &msg);
void DumpCallback(const std_msgs::Int8 &msg);

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

// ROS Subscriber for Conveyer messages
ros::Subscriber<std_msgs::Int16> conveyerSub("conveyer", &ConveyerCallback);

// Ros Subscriber for Dump messages
ros::Subscriber<std_msgs::Int8> dumpSub("dump", &DumpCallback);

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
    leftSpeedScaled = map(msg.left_vel,0,255,0,maxSpeed);
    digitalWrite(PIN_ENABLE_FL, HIGH);
    digitalWrite(PIN_ENABLE_BL, HIGH);
    if(!msg.left_dir)
    {
      analogWrite(PIN1_SET_FL, leftSpeedScaled);
      analogWrite(PIN2_SET_FL, 0);
      analogWrite(PIN1_SET_BL, leftSpeedScaled);
      analogWrite(PIN2_SET_BL, 0);
    }
    else
    {
      analogWrite(PIN1_SET_FL, 0);
      analogWrite(PIN2_SET_FL, leftSpeedScaled);
      analogWrite(PIN1_SET_BL, 0);
      analogWrite(PIN2_SET_BL, leftSpeedScaled);
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
    rightSpeedScaled = map(msg.right_vel,0,255,0,maxSpeed);
    digitalWrite(PIN_ENABLE_FR, HIGH);
    digitalWrite(PIN_ENABLE_BR, HIGH);
    if(!msg.right_dir)
    {
      analogWrite(PIN1_SET_FR, rightSpeedScaled);
      analogWrite(PIN2_SET_FR, 0);
      analogWrite(PIN1_SET_BR, rightSpeedScaled);
      analogWrite(PIN2_SET_BR, 0);
    }
    else
    {
      analogWrite(PIN1_SET_FR, 0);
      analogWrite(PIN2_SET_FR, rightSpeedScaled);
      analogWrite(PIN1_SET_BR, 0);
      analogWrite(PIN2_SET_BR, rightSpeedScaled);
    }
  }
}

// Callback from light msg, changes mode of the light
void LightCallback(const std_msgs::UInt8 &msg) {
    lightMode = msg.data;
}

// Callback from conveyer msg, runs the conveyer
void ConveyerCallback(const std_msgs::Int16 &msg) {
    //Left motor not being used
    if(msg.data == 0)
    {
        digitalWrite(PIN_ENABLE_CON, LOW);
    }
    //Compute direction and velocities of left motors
    else
    {
        digitalWrite(PIN_ENABLE_CON, HIGH);
        if(msg.data > 0)
        {
            analogWrite(PIN1_SET_CON, msg.data);
            analogWrite(PIN2_SET_CON, 0);
        }
        else
        {
            analogWrite(PIN1_SET_CON, 0);
            analogWrite(PIN2_SET_CON, msg.data);
        }
    }
}

void DumpCallback(const std_msgs::Int8 &msg)
{
    switch(msg.data)
    {
         //Up
         case 1:
             digitalWrite(PIN_DUMP_UP1, LOW);
             digitalWrite(PIN_DUMP_UP2, LOW);
             digitalWrite(PIN_DUMP_DOWN1, HIGH);
             digitalWrite(PIN_DUMP_DOWN2, HIGH);
             break;
             
         //Down
         case -1:
             digitalWrite(PIN_DUMP_UP1, HIGH);
             digitalWrite(PIN_DUMP_UP2, HIGH);
             digitalWrite(PIN_DUMP_DOWN1, LOW);
             digitalWrite(PIN_DUMP_DOWN2, LOW);
             break;
         
         //off
         default:
             digitalWrite(PIN_DUMP_UP1, HIGH);
             digitalWrite(PIN_DUMP_UP2, HIGH);
             digitalWrite(PIN_DUMP_DOWN1, HIGH);
             digitalWrite(PIN_DUMP_DOWN2, HIGH);
             break;
           
    }
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

/*
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
}*/


/*******************************************************************************
 * Initial Arduino Setup
 *******************************************************************************/

void setup() {
    // Setup pins
    pinMode(PIN_ENABLE_FL, OUTPUT);
    pinMode(PIN_ENABLE_FR, OUTPUT);
    pinMode(PIN_ENABLE_BL, OUTPUT);
    pinMode(PIN_ENABLE_BR, OUTPUT);
    pinMode(PIN_ENABLE_CON, OUTPUT);

    pinMode(PIN1_SET_FL, OUTPUT);
    pinMode(PIN2_SET_FL, OUTPUT);
    pinMode(PIN1_SET_FR, OUTPUT);
    pinMode(PIN2_SET_FR, OUTPUT);
    pinMode(PIN1_SET_BL, OUTPUT);
    pinMode(PIN2_SET_BL, OUTPUT);
    pinMode(PIN1_SET_BR, OUTPUT);
    pinMode(PIN2_SET_BR, OUTPUT);
    pinMode(PIN1_SET_CON, OUTPUT);
    pinMode(PIN2_SET_CON, OUTPUT);
    pinMode(PIN_LIGHT, OUTPUT);
    
    pinMode(PIN_DUMP_UP1, OUTPUT);
    pinMode(PIN_DUMP_UP2, OUTPUT);
    pinMode(PIN_DUMP_DOWN1, OUTPUT);
    pinMode(PIN_DUMP_DOWN2, OUTPUT);
    
    // Light starts out as off
    digitalWrite(PIN_LIGHT, LOW);
    
    digitalWrite(PIN_DUMP_UP1, HIGH);
    digitalWrite(PIN_DUMP_UP2, HIGH);
    digitalWrite(PIN_DUMP_DOWN1, HIGH);
    digitalWrite(PIN_DUMP_DOWN2, HIGH);
    
    // Setup ROS node and topics
    nodeHandle.initNode();
    nodeHandle.subscribe(velocitySub);
    nodeHandle.subscribe(lightSub);
    nodeHandle.advertise(imuPub);
    nodeHandle.subscribe(conveyerSub);
    nodeHandle.subscribe(dumpSub);    
    // Setup magnetometer
    /*
    Wire.begin();
    Wire.beginTransmission(ADDR_MAGNETOMETER);
    Wire.write(0x02); //select mode register
    Wire.write(0x00); //continuos measurement
    Wire.endTransmission();*/
}


/*******************************************************************************
 * Main Arduino Loop
 *******************************************************************************/

void loop() {
    nodeHandle.spinOnce();
    //updateIMU();
    updateLight();
}

