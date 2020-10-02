/******************************************************************************
* @file moonshot.ino
* @brief The arduino code for moonshot's microcontroller. It is responsible
* for controlling the motors, blinking the indicator light,
* and publishing IMU messages from the sensor stick found at
* https://www.sparkfun.com/products/10724.
* @author Matt Anderson <mia2n4>
* @author Islam Elnabarawy <ie3md>
* @author Ryan Loeffelman <rjlt3c>
* @version 1.0.1
******************************************************************************/
#include <ros.h>
#include <control/Velocity.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

//#include <Wire.h>
ros::NodeHandle nh;

const int PIN_DUMP_UP1 = 29;
const int PIN_DUMP_UP2 = 25;
const int PIN_DUMP_DOWN1 = 27;
const int PIN_DUMP_DOWN2 = 23;
/*******************************************************************************
* Constants
*******************************************************************************/
// Pin configurations
const int PIN_ENABLE_FL = 22; //Number 1
const int PIN_ENABLE_FR = 24; //Number 2
const int PIN_ENABLE_BL = 26; //Number 3
const int PIN_ENABLE_BR = 28; //Number 4
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

const  int maxSpeed = 225;
float leftSpeedScaled;
float rightSpeedScaled;

const int estopPin = 38;
//bool torque = false;

/*******************************************************************************
* Forward delerations
*******************************************************************************/
void VelocityCallback(const control::Velocity &msg);
void ConveyerCallback(const std_msgs::Int16 &msg);
void DumpCallback(const std_msgs::Int8 &msg);
void HardStopCallback(const std_msgs::Bool &msg);
//void DiagCallback(const std_msgs::Bool &msg);

/*******************************************************************************
* Variables
*******************************************************************************/
// ROS node handle
ros::NodeHandle nodeHandle;
// ROS Subscriber for Velocity messages
ros::Subscriber<control::Velocity> velocitySub("cmd_vel", &VelocityCallback);

/*******************************************************************************
 * Variables
 *******************************************************************************/

// ROS Subscriber for Conveyer messages
ros::Subscriber<std_msgs::Int16> conveyerSub("conveyer", &ConveyerCallback);
// Ros Subscriber for Dump messages
ros::Subscriber<std_msgs::Int8> dumpSub("dump", &DumpCallback);

// Ros Subscriber for Estop messages
ros::Subscriber<std_msgs::Bool> hardStopSub("hardStop", &HardStopCallback);
//ros::Subscriber<std_msgs::Bool> diagSub("diag", &DiagCallback);

/*******************************************************************************
* Callbacks
*******************************************************************************/
// Callback for Velocity, gets the left and right wheel velocity and direction
void VelocityCallback(const control::Velocity &msg) 
{
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
    //SerialUSB.print("DumpCallback has been run\n");
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

void HardStopCallback(const std_msgs::Bool &msg)
{
  if(msg.data)
  {
    //all enables low, write 0 to all analogWrite() pins
    //switch relay on!
    //this section turns dump off
     digitalWrite(PIN_DUMP_UP1, HIGH);
     digitalWrite(PIN_DUMP_UP2, HIGH);
     digitalWrite(PIN_DUMP_DOWN1, HIGH);
     digitalWrite(PIN_DUMP_DOWN2, HIGH);
     
     
     //this section writes 0 to all analogWrite pins
     analogWrite(PIN1_SET_FL,0);
     analogWrite(PIN2_SET_FL,0);
     analogWrite(PIN1_SET_FR,0);
     analogWrite(PIN2_SET_FR,0);
     analogWrite(PIN1_SET_BL,0);
     analogWrite(PIN2_SET_BL,0);
     analogWrite(PIN1_SET_BR,0);
     analogWrite(PIN2_SET_BR,0);
     analogWrite(PIN1_SET_CON,0);
     analogWrite(PIN2_SET_CON,0);
     
     //this section turns off enables
     digitalWrite(PIN_ENABLE_FL,LOW);
     digitalWrite(PIN_ENABLE_FR,LOW);
     digitalWrite(PIN_ENABLE_BL,LOW);
     digitalWrite(PIN_ENABLE_BR,LOW);
     digitalWrite(PIN_ENABLE_CON,LOW);
     
     //this section turns the relay on
     digitalWrite(estopPin, LOW);
  }
  else
  {
    digitalWrite(estopPin, HIGH);
  }
}

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
    
    pinMode(PIN_DUMP_UP1, OUTPUT);
    pinMode(PIN_DUMP_UP2, OUTPUT);
    pinMode(PIN_DUMP_DOWN1, OUTPUT);
    pinMode(PIN_DUMP_DOWN2, OUTPUT);
    
    digitalWrite(PIN_DUMP_UP1, HIGH);
    digitalWrite(PIN_DUMP_UP2, HIGH);
    digitalWrite(PIN_DUMP_DOWN1, HIGH);
    digitalWrite(PIN_DUMP_DOWN2, HIGH);
    
    pinMode(estopPin, OUTPUT);
    digitalWrite(estopPin, HIGH);
   
    
    // Setup ROS node and topics
    nh.initNode();
    nh.subscribe(velocitySub);
    nh.subscribe(conveyerSub);
    nh.subscribe(dumpSub);  
    nh.subscribe(hardStopSub);  
    //nh.subscribe(diagSub);
    
    //nodeHandle.getHardware()->setBaud(115200);
    //SerialUSB.begin(115200);
}
/*******************************************************************************
 * Main Arduino Loop
 *******************************************************************************/

void loop() {
    nh.spinOnce();
}

