/******************************************************************************
 * @file TrackingNode.h
 * @author Matt Anderson <mia2n4@mst.edu>
 * @brief The header file for the TrackingNode class
 ******************************************************************************/

#ifndef TRACKING_NODE_H
#define TRACKING_NODE_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include "navigation/TrackingNode_ParamsConfig.h"

const double ROS_RATE = 30; //Hz

class TrackingNode {
private:
    ros::NodeHandle nh;
    geometry_msgs::Twist cmd_vel;

    //Publishers
    ros::Publisher twist_pub;

    //Subscribers
    ros::Subscriber goal_sub;

    //PID control variables
    double prev_error;
    double prev_integ;

    typedef navigation::TrackingNode_ParamsConfig Params;
    typedef dynamic_reconfigure::Server<Params> ParamsServer;
    typedef dynamic_reconfigure::Server<Params>::CallbackType ParamsCallback;

    Params config;
    ParamsServer cfg_server;
    ParamsCallback cfg_callback;

    void setparamsCallback(Params &config, uint32_t level);

    static double find_rotation(geometry_msgs::Point goal);

public:
    TrackingNode();

    double calculate_angular();

    void update();
};

#endif // TRACKING_NODE_H
