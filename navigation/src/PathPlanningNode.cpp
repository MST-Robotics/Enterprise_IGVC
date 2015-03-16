/**
 * @file PathPlanningNode.cpp
 * @author Matt Anderson <mia2n4@mst.edu>
 * @brief The definition file for the PathPlanningNode class
 */

#include "PathPlanningNode.h"

geometry_msgs::Point robot;

/**
 * @fn PathPlanningNode::findDistance(geometry_msgs::Point p1
 *                                    geometry_msgs::Point p2)
 * @brief Finds the x, y distance between two points in space
 * @post Finds the distance b/w two points in space
 * @param p1 a point in space
 * @param p2 a point in space
 * @return the x, y distance b/w two points in space
 */
double PathPlanningNode::findDistance(geometry_msgs::Point p1,
        geometry_msgs::Point p2) {
    double dist;

    dist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

    return dist;
}

/**
 * @fn PathPlanningNode::findAngle(geometry_msgs::Point p1,
 *                                 geometry_msgs::Point p2)
 * @brief Finds the angle b/w two points in free space
 * @pre p1 and p2 are on the same plane
 * @post Finds the angle b/w p1 and p2
 * @param p1 a point in space
 * @param p2 a point in space
 * @return The angle b/w p1 and p2
 */
double PathPlanningNode::findAngle(geometry_msgs::Point p1,
        geometry_msgs::Point p2) {
    double angle;

    angle = atan2(p2.y - p1.y, p2.x - p1.x);

    return angle;
}

/**
 * @fn PathPlanningNode::toRobotFrame(geometry_msgs::Point p1)
 * @brief Converts a latitude and longitude to a point that exists in the
 *        robot's frame of reference.  This point assumes that robot is located
 *        at the origin of it's coordinate system.
 * @pre The geometry_msgs::Point robot variable must be set to the latitude and
 *      longitude of the robot. p1 must also be a latitude and longitude
 * @post Returns a point that is in the robots frame of reference
 * @param p1 the point to translate to the robot's frame of reference
 * @return a point that uses the robots current position as the origin
 */
geometry_msgs::Point PathPlanningNode::toRobotFrame(geometry_msgs::Point p1) {
    geometry_msgs::Point p;

    double angle = findAngle(robot, p1);
    double dist = findDistance(robot, p1);

    tf::StampedTransform transform;
    tf::TransformListener listener;

    //Find the transform from "odom" frame to "map" frame
    try {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/odom", "/base_link", now, ros::Duration(3.0));
        listener.lookupTransform("/odom", "/base_link", ros::Time::now(), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    p.x = dist * cos(angle) - transform.getOrigin().x();
    p.y = dist * sin(angle) - transform.getOrigin().y();
    p.z = 0;

    return p;
}

/**
 * @fn gps_callback(sensor_msgs::NavSatFix msg)
 * @brief Callback for the gps, treats the current location as a point on a
 *        cartesian plane
 * @pre GPS is transmitting
 * @post sets the robot point to the latitude and longitude of the robot
 * @param msg a message from the gps
 */
void gps_callback(sensor_msgs::NavSatFix msg) {
    robot.x = msg.longitude;
    robot.y = msg.latitude;
    robot.z = 0;
}

/**
 * @fn PathPlanningNode::PathPlanningNode()
 * @brief Default constructor for PathPlanningNode
 */
PathPlanningNode::PathPlanningNode() {
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, gps_callback);

    goal_pub = nh.advertise<geometry_msgs::Point>("/target_pub", 1);

    //set goal, static for now
    goal.y =  37.956714;
    goal.x = -91.774622;
}

/**
 * @fn PathPlanningNode::update()
 * @brief Publishes a new goal for the robot
 * @pre goal_pub has been initialized
 * @post A new goal is published
 */
void PathPlanningNode::update() {
    goal_pub.publish(toRobotFrame(goal));
}
