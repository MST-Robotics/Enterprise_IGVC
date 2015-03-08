/******************************************************************************
* @file navigation.h
* @author Matt Anderson <mia2n4@mst.edu>
* @brief The header file for the navigation code, contains class
******************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#define PI 3.14159

class NavigationServer
{
	private:
		ros::NodeHandle nh;
		geometry_msgs::Twist cmd_vel;

		//Publishers
		ros::Publisher twist_pub;

		//Subscribers
		ros::Subscriber goal_sub;

		//PID control variables
		static const double ROS_RATE = 30; //Hz
		double prev_error;
		double prev_integ;
		double prop_gain;
		double deriv_gain;
		double integ_gain;


	public:
		NavigationServer();
		
		double calculate_angular();

		void update();
};

