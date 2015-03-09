/******************************************************************************
 * @file navigation.h
 * @author Matt Anderson <mia2n4@mst.edu>
 * @brief The header file for the navigation code, contains class
 ******************************************************************************/

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include "mst_navigation/Navigation_ParamsConfig.h"

const double ROS_RATE = 30; //Hz

class NavigationServer {
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
    double prop_gain;
    double deriv_gain;
    double integ_gain;

    typedef mst_navigation::Navigation_ParamsConfig Params;
    typedef dynamic_reconfigure::Server<Params> ParamsServer;
    typedef dynamic_reconfigure::Server<Params>::CallbackType ParamsCallback;

    Params config;
    ParamsServer cfg_server;
    ParamsCallback cfg_callback;

    void setparamsCallback(Params &config, uint32_t level);

public:
    NavigationServer();

    double calculate_angular();

    void update();
};

#endif // NAVIGATION_H
