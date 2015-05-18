/******************************************************************************
* @file moonshot_master.ino
* @brief The arduino code for moonshot's microcontroller. It is responsible
*           for controlling the motors, encoders, and PID Controls. This arduino
*           is the master arduino polling the slave for the joy data needed.
* @author Ryan Loeffelman  <rjlt3c>
* @version 1.0.1
******************************************************************************/
#include<Wire.h>

/*******************************************************************************
 * Constants
 *******************************************************************************/
// Pin configurations
const int PIN_ENABLE_FL = 22;  //Number 1
const int PIN_ENABLE_FR = 24;  //Number 2
const int PIN_ENABLE_BL = 26;  //Number 3
const int PIN_ENABLE_BR = 28;  //Number 4


const int PIN1_SET_FL = 2;
const int PIN2_SET_FL = 3;
const int PIN1_SET_FR = 4;
const int PIN2_SET_FR = 5;
const int PIN1_SET_BL = 6;
const int PIN2_SET_BL = 7;
const int PIN1_SET_BR = 8;
const int PIN2_SET_BR = 9;

const  int maxSpeed = 225;
float leftSpeedScaled;
float rightSpeedScaled;

bool leftDir;
bool rightDir;
unsigned int leftSpeed;
unsigned int rightSpeed;
int recieved[2];



/*******************************************************************************
 * Callbacks
 *******************************************************************************/

// Callback for Velocity, gets the left and right wheel velocity and direction
void updateMotors()
{
  
  //Left motor not being used
  if(leftSpeed == 0)
  {
    digitalWrite(PIN_ENABLE_FL, LOW);
    digitalWrite(PIN_ENABLE_BL, LOW);
  }
  //Compute direction and velocities of left motors
  else
  {
    leftSpeedScaled = map(leftSpeed,0,255,0,maxSpeed);
    digitalWrite(PIN_ENABLE_FL, HIGH);
    digitalWrite(PIN_ENABLE_BL, HIGH);
    if(!leftSpeed < 0)
    {
      analogWrite(PIN1_SET_FL, (leftSpeedScaled*-1));
      analogWrite(PIN2_SET_FL, 0);
      analogWrite(PIN1_SET_BL, (leftSpeedScaled*-1));
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
  if(rightSpeed == 0)
  {
    digitalWrite(PIN_ENABLE_FR, LOW);
    digitalWrite(PIN_ENABLE_BR, LOW);
  }
  //Compute direction and velocities of left motors
  else
  {
    rightSpeedScaled = map(rightSpeed,0,255,0,maxSpeed);
    digitalWrite(PIN_ENABLE_FR, HIGH);
    digitalWrite(PIN_ENABLE_BR, HIGH);
    if(rightSpeed < 0)
    {
      analogWrite(PIN1_SET_FR, (rightSpeedScaled*-1));
      analogWrite(PIN2_SET_FR, 0);
      analogWrite(PIN1_SET_BR, (rightSpeedScaled*-1));
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

/*******************************************************************************
 * Initial Arduino Setup
 *******************************************************************************/

void setup() 
{
    // Setup pins
    pinMode(PIN_ENABLE_FL, OUTPUT);
    pinMode(PIN_ENABLE_FR, OUTPUT);
    pinMode(PIN_ENABLE_BL, OUTPUT);
    pinMode(PIN_ENABLE_BR, OUTPUT);

    pinMode(PIN1_SET_FL, OUTPUT);
    pinMode(PIN2_SET_FL, OUTPUT);
    pinMode(PIN1_SET_FR, OUTPUT);
    pinMode(PIN2_SET_FR, OUTPUT);
    pinMode(PIN1_SET_BL, OUTPUT);
    pinMode(PIN2_SET_BL, OUTPUT);
    pinMode(PIN1_SET_BR, OUTPUT);
    pinMode(PIN2_SET_BR, OUTPUT);
    
    Wire.begin();
   
}


/*******************************************************************************
 * Main Arduino Loop
 *******************************************************************************/

void loop() 
{
  
    Wire.requestFrom(2, 4);
    while(Wire.available())
    {
      leftSpeed = Wire.read();
      rightSpeed = Wire.read();
    }
  
  updateMotors();
    
}
