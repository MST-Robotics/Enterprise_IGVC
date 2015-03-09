/******************************************************************************
 * @file navigation.h
 * @author Matt Anderson <mia2n4@mst.edu>
 * @brief The definition file for the NavigationServer class
 ******************************************************************************/

#include "navigation.h"

geometry_msgs::PoseStamped goal_pose;

/*
 * @fn target_callback(geometry_msgs::PoseStamped pose)
 * @brief extracts the information from pose and stores it in goal_pose which is
 *        a global variable that can always be accessed
 * @pre there should be a subscriber that is subscribed to a PoseStamped message
 *      that represents the pose of the goal
 * @post goal_pose is the pose of the goal (gps waypoint)
 * @param geometry_msgs::PoseStamped pose - The pose of the goal that was recieved
 *        via a message
 */
void target_callback(geometry_msgs::PoseStamped pose) {
    goal_pose.pose.position.x = pose.pose.position.x;
    goal_pose.pose.position.y = pose.pose.position.y;
    goal_pose.pose.position.z = pose.pose.position.z;

    goal_pose.pose.orientation.x = pose.pose.orientation.x;
    goal_pose.pose.orientation.y = pose.pose.orientation.y;
    goal_pose.pose.orientation.z = pose.pose.orientation.z;
}

/***********************************************************
 * @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
 * @brief callback for the reconfigure gui
 * @pre has to have the setup for the reconfigure gui
 * @post changes the parameters on navigation to those from the
 * parameter server
 ***********************************************************/
void NavigationServer::setparamsCallback(Params &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %d %d %d", config.prop_gain,
            config.deriv_gain, config.integ_gain, config.linear_velocity);

    // set params
    this->config = config;
}

/*
 * @fn NavigationServer::NavigationServer()
 * @brief The default constructor for the NavigationServer class
 */
NavigationServer::NavigationServer() {
    //Create the twist publisher which publishes to cmd_vel
    twist_pub = nh.advertise<geometry_msgs::Twist>("nav_twist", 1);

    //Create the target subscriber which subscribes to target_pub
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("target_pub", 1,
            target_callback);

    //Set initial PID variables
    prev_error = M_PI;
    prev_integ = 0;
    prop_gain = config.prop_gain;
    deriv_gain = 1;
    integ_gain = 1;

    //setup dynamic reconfigure
    cfg_callback = boost::bind(&NavigationServer::setparamsCallback, this, _1,
            _2);
    cfg_server.setCallback(cfg_callback);
}

/*
 * @fn double NavigationServer::find_rotation(geometry_msgs::PoseStamped goal)
 * @brief Finds the rotation angle
 * @pre A valid transform from odom to map
 * @post returns an angle which signifies the angle between the robot and the
 *       goal with respect to the map frame
 * @param geometry_msgs::PoseStamped goal This is the pose of the goal which
 *        only needs the position.
 */
double find_rotation(geometry_msgs::PoseStamped goal) {
    double theta;
    tf::StampedTransform transform;
    tf::TransformListener listener;

    //Find the transform from "odom" frame to "map" frame
    try {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/map", "/odom", now, ros::Duration(3.0));
        listener.lookupTransform("/map", "/odom", ros::Time::now(), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    //Find the angle that the robot must turn to in order to be facing the goal
    theta = atan2((goal.pose.position.y - transform.getOrigin().y()),
            (goal.pose.position.x - transform.getOrigin().x()));

    return theta;
}

/*
 * @fn double NavigationServer::calculate_angular()
 * @brief Finds the twist based on the target location and the robot location
 *        using a PID controller
 * @pre a message is being published which contains the pose of the goal
 * @post calculates the angular velocity for the robot to navigate to a point
 */
double NavigationServer::calculate_angular() {
    double error, deriv, integ, prop;
    double angular_vel;     //rad/s

    error = find_rotation(goal_pose);
    deriv = (ROS_RATE) * (error - prev_error);
    integ = (1 / ROS_RATE) * error + prev_integ;

    angular_vel = prop_gain * error + integ_gain * integ + deriv_gain * deriv;

    prev_error = error;
    prev_integ = integ;

    return angular_vel;
}

/*
 * @fn void NavigationServer::update()
 * @brief updates the cmd_vel
 * @pre there should be a subscriber that is subscribed to a PoseStamped message
 *      that represents the pose of the goal
 * @post cmd_vel is the twist to send to the robot
 */
void NavigationServer::update() {
    cmd_vel.angular.z = calculate_angular();
    cmd_vel.linear.x = .5;
    twist_pub.publish(cmd_vel);
}

