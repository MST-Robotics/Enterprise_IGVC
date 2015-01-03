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

// Two arrays to store the last 5 x and y values for averaging
int x_trail[5] = {0, 0, 0, 0, 0};
int y_trail[5] = {0, 0, 0, 0, 0};

// The index of the oldest value in the array to be averaged - this one will be
//   replaced next
int trail_index = 0;

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

    x_trail[trail_index] = magx;
    y_trail[trail_index] = magy;
    trail_index++;
    if(trail_index == 5)
        trail_index = 0;
    
    // The x an y values are a moving average over the last 5 data points
    imu_msg.x = (x_trail[0] + x_trail[1] + x_trail[2] + x_trail[3] + x_trail[4]) / 5.0;
    imu_msg.y = (y_trail[0] + y_trail[1] + y_trail[2] + y_trail[3] + y_trail[4]) / 5.0;

    // Old code - Delete after making sure the averaging code above works
    //populate message
    //imu_msg.x = magx;
    //imu_msg.y = magy;
    
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

