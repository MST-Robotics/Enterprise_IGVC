/**
 * @file PathPlanningNode.h
 * @author Matt Anderson <mia2n4@mst.edu>
 * @brief The header file for the PathPlanningNode class
 */

#ifndef PATH_PLANNING_NODE_H
#define PATH_PLANNING_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <queue>
#include <fstream>
#include <cstdlib>

const std::string FILENAME = "/home/enterprise/waypoints.txt";

class PathPlanningNode {
private:
    ros::NodeHandle nh;

    ros::Subscriber gps_sub;
    ros::Publisher goal_pub;

    std::queue<geometry_msgs::Point> waypoints;
    geometry_msgs::Point current_goal;

    double findDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
    double findAngle(geometry_msgs::Point p1, geometry_msgs::Point p2);
    geometry_msgs::Point toRobotFrame(geometry_msgs::Point p1);
    void readWaypoints();

public:
    PathPlanningNode();

    void update();
};

#endif
