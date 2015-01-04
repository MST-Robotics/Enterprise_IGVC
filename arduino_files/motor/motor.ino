
//#include <ArduinoHardware.h>
#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

float linearVel;
float angularVel;
geometry_msgs::Twist test;
ros::NodeHandle nh;
int leftVel, rightVel, leftDir, rightDir, lightMode = 0;
/*******************************************************************************
* Constants
*******************************************************************************/
const float ROBOT_RAD  = 0.28; //meters
const float WHEEL_RAD  = 0.20; //meters
const float GEAR_RATIO = 21.952;    //don't know
const int   MAX_PUB    = 256;  //This is the max value that can be published

int loopCount = 0;

//Pin
const int
Lmotor   = 3,
Rmotor   = 11,
Ldir     = 13,
Rdir     = 12,
Light    = 7,
SNS_A    = A0;

//Callback from twist which gets a linear and angular velocity 
void TwistCallback(const geometry_msgs::Twist &msg)
{
  //taking subscribed messages from Twist  
  angularVel = msg.angular.z;
  linearVel = msg.linear.x;
  return;
} 

/* For 4 mode light
void switchMode(int mode)
{
  if(lightMode != mode)
  {  
    for(int i = 0; i < 2; i++)
    {
      digitalWrite(Light, LOW);
      delay(200);
      digitalWrite(Light, HIGH);
      delay(200);
    }
    lightMode = mode;
  }
}*/

//Callback from light msg, changes mode of the light
void LightCallback(const std_msgs::Int8 &msg)
{  
  lightMode = msg.data;
  /* Four mode light
  switch(msg.data)
  {
    case 0:
      switchMode(0);
      break;
    case 1:
      switchMode(1);
      break;
  }*/
}
  
const float TURNS_PER_SEC = GEAR_RATIO / (2.0 * WHEEL_RAD * M_PI);
const float TURN_OFFSET = TURNS_PER_SEC * ROBOT_RAD;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_throttle",&TwistCallback);
ros::Subscriber<std_msgs::Int8> lightSub("indicator_light_throttle", &LightCallback);
ros::Publisher p("deBug", &test);

void setup()
{
  //nh.getHardware()->setBaud(115200);
  pinMode(Lmotor, OUTPUT);
  pinMode(Rmotor, OUTPUT);
  pinMode(Ldir, OUTPUT);
  pinMode(Rdir, OUTPUT);
  pinMode(Light, OUTPUT);
  digitalWrite(Light, LOW);
  lightMode = 0;
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(lightSub);
  nh.advertise(p);
}

void loop()
{ 
  //Left and right velocities
  /*leftVel = (linearVel * TURNS_PER_SEC) - (TURN_OFFSET * angularVel);
  rightVel = (linearVel * TURNS_PER_SEC) + (TURN_OFFSET * angularVel);*/
  rightVel = (2 * linearVel + angularVel * 2 * ROBOT_RAD) / ( 2 * WHEEL_RAD); 
  leftVel = (2 * linearVel - angularVel * 2 * ROBOT_RAD) / ( 2 * WHEEL_RAD);
  
  //Scale motor values
  rightVel *= (MAX_PUB / 76);
  leftVel  *= (MAX_PUB / 76);
  
  test.linear.x = rightVel;
  test.linear.y = leftVel;
  test.angular.x = linearVel;
  test.angular.y = angularVel;
  
  /*
  if(lightMode == 1)
  {
    loopCount++;
    if(loopCount == 5)
    {
      digitalWrite(Light, HIGH);
      delay(500); 
      digitalWrite(Light, LOW);
      loopCount = 0;
    }
  }*/
    
  //take out any negative values
  if(abs(leftVel) > MAX_PUB)
    leftVel = MAX_PUB - 1;
  if(abs(rightVel) > MAX_PUB)
    rightVel = MAX_PUB - 1;
      
  if(leftVel < 0)
    leftDir = 0;
  else
    leftDir = 1;
    
  if(rightVel < 0)
    rightDir = 1;
  else
    rightDir = 0;
  /*
  if(leftDir == rightDir && leftVel <.1 || rightVel <.1)
  {
    if (leftDir == 0)
    {
      leftVel = 75;
      rightVel = 0;
    }
    if (leftDir == 1)
    {
      rightVel = 75;
      leftVel = 0;
    }
  }*/
  
  leftVel  = abs(leftVel);
  rightVel = abs(rightVel);
  //nh.loginfo("updating"); 
  analogWrite(Lmotor, leftVel);
  analogWrite(Rmotor, rightVel);
  digitalWrite(Ldir, leftDir);
  digitalWrite(Rdir, rightDir);
  p.publish(&test);   
  nh.spinOnce();
  delay(10);
}

