/******************************************************************************
* @file imu.ino
* @brief The arduino code to publish IMU messages from the sensor stick found
*        at https://www.sparkfun.com/products/10724.
* @author Matt Anderson <mia2n4>
* @version 0.0.1
******************************************************************************/

#include <ros.h>
#include <geometry_msgs/Quaternion.h>

#include <Wire.h>

#define MAG_ADDR 0x1E 

//Create a nodehandler
ros::NodeHandle nh;

//Create the imu message which will be published by this node
geometry_msgs::Quaternion imu_msg;

//Create the publisher to send out the messages
ros::Publisher pub("imu", &imu_msg);

//Setup function which runs once on the arduino before loop()
void setup()
{
    //Initialize the node and advertise the message
    nh.initNode();
    nh.advertise(pub);
    
    //Setup magnetometer
    Wire.begin();
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x02); //select mode register
    Wire.write(0x00); //continuos measurement
    Wire.endTransmission();
}

void loop()
{
    int magx, magy, magz;
    
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x03);
    Wire.endTransmission();
    
    //Recieve data from magnetometer
    Wire.requestFrom(MAG_ADDR, 6);
    if(Wire.available() >= 6)
    {
        magx = Wire.read() << 8;
        magx |= Wire.read();
        magz = Wire.read() << 8;
        magz |= Wire.read();
        magy = Wire.read() << 8;
        magy |= Wire.read();
    }
    
    //populate message
    imu_msg.x = magx;
    imu_msg.y = magy;
    
    //publish message
    pub.publish(&imu_msg);
    
    /* Debugging Purposes
    Serial.print("x: ");
    Serial.print(magx);
    Serial.print("y: ");
    Serial.print(magy);
    Serial.print(" \n");
    */
    
    nh.spinOnce();
    delay(500);
}

