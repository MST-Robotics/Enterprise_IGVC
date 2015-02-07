/*******************************************************************************
* @file tf_transforms.cpp
* @brief This contains the transforms for enterprise and will take in sensor 
*        data and publish joint states
* @author Matt Anderson <mia2n4@mst.edu>
*******************************************************************************/

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

//Global Constants
double heading;

//Callback functions
/***********************************************************
 * @fn imuCallback(const geometry_msgs::Quaternion::ConstPtr
 * & imu)
 * @brief gets the position information from the imu 
 * (magnatometer), sets the current_head = heading 
 * observed by the magnatometer.
 * @pre imu publishing Quaternion
 * @post current_head = robot's heading
 * @param takes in a imu publishing Quaternion
 ***********************************************************/
void imu_callback(const geometry_msgs::Quaternion::ConstPtr& imu) 
{
    heading = atan2(imu->y, imu->x);
}

int main(int argc, char** argv)
{
    //Ros initialization
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    
    //Create subscriber to IMU data
    ros::Subscriber imu_sub;
    imu_sub = n.subscribe<geometry_msgs::Quaternion>("/imu", 20, &imu_callback);
    
    ros::Publisher joint_pub;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);
    
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    while(ros::ok())
    {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="Camera";
        joint_state.position[0] = 0;
        joint_state.name[1] ="laser";
        joint_state.position[1] = 0;

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(heading);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // This will adjust as needed per iteration
        ros::spinOnce();
    }
    
    return 0;
}
