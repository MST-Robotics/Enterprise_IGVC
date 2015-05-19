/******************************************************************************
* @file moonshot_master.ino
* @brief The arduino code for moonshot's microcontroller. It is responsible
*           for controlling the dump, the conveyer, and sending motor velocity 
*           msgs to the slave arduino. This arduino is the slave arduino 
*           sending data to the master when the master requests it.
* @author Matt Anderson <mia2n4>
* @author Islam Elnabarawy <ie3md>
* @author Ryan Loeffelman  <rjlt3c>
* &@version 1.0.1
******************************************************************************/

#include <ros.h>
#include<Wire.h>

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

bool dontGoNowhere = true;

const int PIN_ENABLE_CON = 30; //Number 5
const int PIN1_SET_CON = 10;
const int PIN2_SET_CON = 11;

const int estopPin = 12;

int leftSpeed; 
int rightSpeed;



/*******************************************************************************
 * Forward delerations
 *******************************************************************************/

void VelocityCallback(const control::Velocity &msg);
void ConveyerCallback(const std_msgs::Int16 &msg);
void DumpCallback(const std_msgs::Int8 &msg);
void HardStopCallback(const std_msgs::Bool &msg);

/*******************************************************************************
 * Variables
 *******************************************************************************/

// ROS node handle
ros::NodeHandle nodeHandle;

// ROS Subscriber for Velocity messages
ros::Subscriber<control::Velocity> velocitySub("cmd_vel", &VelocityCallback);

// ROS Subscriber for Conveyer messages
ros::Subscriber<std_msgs::Int16> conveyerSub("conveyer", &ConveyerCallback);

// Ros Subscriber for Dump messages
ros::Subscriber<std_msgs::Int8> dumpSub("dump", &DumpCallback);

// Ros Subscriber for Estop messages
ros::Subscriber<std_msgs::Bool> hardStopSub("hardStop", &HardStopCallback);

/*******************************************************************************
 * Callbacks
 *******************************************************************************/

// Callback for Velocity, gets the left and right wheel velocity and direction
void VelocityCallback(const control::Velocity &msg) {
  bool leftDir = msg.left_dir;
  bool rightDir = msg.right_dir;
  
  //for the ability to transmit over I2C easier convert to only two variables.
  if(leftDir)
  {
    leftSpeed = msg.left_vel;
  }
  else
  {
    leftSpeed = -msg.left_vel;
  }
  if(rightDir)
  {
    rightSpeed = msg.right_vel;
  }
  else
  {
    rightSpeed = -msg.right_vel;
  }
  
  if(!dontGoNowhere)
  {
    

    
  }
}

// Callback from conveyer msg, runs the conveyer
void ConveyerCallback(const std_msgs::Int16 &msg) {
    //Left motor not being used
    if(msg.data == 0)
    {
        analogWrite(PIN1_SET_CON, 0);
        analogWrite(PIN2_SET_CON, 0);
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
     
     leftSpeed = 0; 
     rightSpeed = 0;
    
     dontGoNowhere = true;
     
     digitalWrite(estopPin, LOW);
     
  }
  else
  {
    dontGoNowhere = false;
    digitalWrite(estopPin, HIGH);
  }
}

void requestEvent()
{
  Wire.write(leftSpeed);
  Wire.write(rightSpeed);
}

/*******************************************************************************
 * Initial Arduino Setup
 *******************************************************************************/

void setup() {
    // Setup pins
    pinMode(PIN_ENABLE_CON, OUTPUT);

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
    
    //Begin the I2C transmission, with this arduino as slave adress 2
    Wire.begin(2);
    Wire.onRequest(requestEvent);
}


/*******************************************************************************
 * Main Arduino Loop
 ******************************************************************************/

void loop() {
    nh.spinOnce();
}
