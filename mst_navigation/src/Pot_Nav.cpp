/*******************************************************************************
* @file Pot_Nav.cpp
* @author James Anderson <jra798>
* @version 1.0
* @brief finds the forward and angular velocity to move the robot at
* given a fuzzy model
******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <mst_position/target_heading.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <mst_navigation/Pot_Nav_ParamsConfig.h>
#include <opencv/cv.h>

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;


/***********************************************************
* Global variables
***********************************************************/
//computational map, the one we take information from and 
//perform calculations on
cv_bridge::CvImage              map;

//map which is displayed, the map we draw debuginging
//information on
cv_bridge::CvImage              map_dis;

//holds green information to be then added to map
cv::Mat                         stat;
//not used at this point
cv::Mat                         edges;

/*

std::queue<cv::Mat>             edges_q;
*/

image_transport::Subscriber     image_sub_edges;
image_transport::Subscriber     image_sub_line;
image_transport::Subscriber     image_sub_flags;
image_transport::Subscriber     image_sub_obst;
image_transport::Subscriber     image_sub_grass;
image_transport::Subscriber     image_sub_stat;

ros::Subscriber                 target_sub;

ros::Publisher                  twist_pub;
image_transport::Publisher      map_pub;

mst_position::target_heading    target;

//makes sure that if the map isn't changed initially, functions past 
//stat callback don't try to do anything with invalid data
bool                            map_changed = 0;
std::queue<ros::Time>           edges_time_q ;
std::queue<ros::Time>           stat_time_q ;

//
bool                            first_callback = 1;

mst_navigation::Pot_Nav_ParamsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/
geometry_msgs::Twist find_twist();

/***********************************************************
* Defines
***********************************************************/
#define pi 3.14159265

/***********************************************************
* Message Callbacks
***********************************************************/


// Edges are not currently used
/***********************************************************
* @fn edgesCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief adds the edges image to map using a weight 
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
/*
void edgesCallback( const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO("Pot_Nav: Edges image received");
	
	cv_bridge::CvImagePtr cv_ptr_src;
	
	
	//takes in the image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    if(first_callback)
    {
		//initialize map
		map.image = cv::Mat::zeros(cv_ptr_src->image.size(),CV_32FC1);
		map.encoding = "32FC1";
		
		first_callback = 0;
	}

    

    //adds them together using weighted percentage

	edges_q.push(cv_ptr_src->image * params.edges_per/100); 
	edges_time_q.push(cv_ptr_src->header.stamp) ;
	
	
	map.header = cv_ptr_src->header;

}
*/

/***********************************************************
* @fn void statCallback( const sensor_msgs::ImageConstPtr& msg)
* @brief adds the color stat image to the map image
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void statCallback( const sensor_msgs::ImageConstPtr& msg)
{
	//why do we need cv_bridge?  cv_bridge is the bridge
	//between OpenCV images and ROS images (we need to
	//convert from one another). 
	cv_bridge::CvImagePtr cv_ptr_src;
	std::vector<cv::Mat> Channels;
	
	//takes in the image
	try
	{
		//taking the ROS msg image and making it a cv image
		cv_ptr_src = cv_bridge::toCvCopy(msg, "rgb8");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if(first_callback)
	{
		//initalize map
		//CV_32FC1 is a 32 bit floating point signed depth in one channel
		//image
		map.image = cv::Mat::zeros(cv_ptr_src->image.size(),CV_32FC1);
		map.encoding = "32FC1";
		
		first_callback = 0;
	}
	
	//splitting the channels from cv_ptr_src to Channels (turns into 3 channels)
	cv::split(cv_ptr_src->image, Channels);
        
	//taking the green component of the vector of matricies that represent
	//channels, 1 corrisponds to green since opencv uses BGR color format, 
	//not rbg.
	Channels[1].convertTo(Channels[1], CV_32FC1);

	//dampens the value of every element in green channel
	//by the percentage given by stat_per(centage)
	stat = Channels[1] * params.stat_per/100;
	
	//takes the metadata from the image processed from the
	//origional ROS ImageConstPtr& msg Image
	map.header = cv_ptr_src->header;


	map_changed = 1;
}

/***********************************************************
* @fn targetCallback( const MST_Position::Target_Heading::ConstPtr& msg)
* @brief copies position data for target to target in Pot_Nav
***********************************************************/
void targetCallback( const mst_position::target_heading::ConstPtr& msg)
{
	target.target_heading = msg->target_heading;
	target.waypoint = msg->waypoint;
	target.distance = msg->distance;
	target.stop_robot = msg->stop_robot;
	target.done = msg->done;
}


/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters on potnav to those from the 
* parameter server
***********************************************************/
void setparamsCallback(mst_navigation::Pot_Nav_ParamsConfig &config, uint32_t level)
{
  // set params
  params = config;
}

/***********************************************************
* Private Functions
***********************************************************/

/***********************************************************
* @fn find_twist()
* @brief computes the twists (angular and linear velocities) 
* using the map
* @pre needs a globally defined map it uses to compute twist
* @post computes the twist
* @return returns the geometry messages standard twist and
* draws debug info on map_dis 
***********************************************************/
geometry_msgs::Twist find_twist()
{
	//Point2i is a typedef of Point (which has an two
	//dimensions, x and y) that is of type integer
	cv::Point2i robot_center;
	geometry_msgs::Twist twist;
	cv::Point2i box_1;
	cv::Point2i box_2;

	map.image.copyTo(map_dis.image);
	map_dis.header = map.header;
	map_dis.encoding = map.encoding;

	//find bottom center of the image
	robot_center.x = map.image.cols / 2;
	robot_center.y = map.image.rows - 1;

	//Find the corners of the box which will be ignored as part of the robot
	box_1.x = robot_center.x - params.robot_x;
	box_1.y = robot_center.y - params.robot_y;

	box_2.x = robot_center.x + params.robot_x;
	box_2.y = robot_center.y + params.robot_y;

	//draw box in place of robot
	cv::rectangle(map_dis.image, box_1, box_2 , 255 ,-2*params.robot_filled+1);

	//remove robot from the map
	cv::rectangle(map.image, box_1, box_2 , 0,-1);

	//draw the search radius
	cv::circle(map_dis.image, robot_center, params.search_radius, 255);

	//initialize twist and apply forward forcing function
	twist.linear.y = 0;
	twist.linear.x = params.carrot_on_a_stick;
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	//add  in target
	//ignoring y for now
	/*twist.linear.x += (params.target_weight_y/100.0) * sin(target.target_heading) / (target.distance * params.target_dist_scale/1000) ;*/
	
	//sign to determine angular twist added to z based on what direction the robot is facing (in radians)
	int sign = 1;
	if (cos(target.target_heading) < 0)
	{
		sign = -1;
	}
	
	//adding one to avoid dividing by zero in some cases
	twist.angular.z += (params.target_weight_z/100.0) * ((twist.linear.x * params.target_y_scale) +1) *
			    pow(fabs(cos(target.target_heading)), params.target_x_exp) * 
			    sign / ((target.distance * params.target_dist_scale/1000) + 1);

	ROS_INFO("y: %f x: %f twist: %f ", sin(target.target_heading) , cos(target.target_heading), twist.angular.z );

	//compute twist
	//Iterate over every ray on the map image
	for(int deg = 0;  deg <= params.search_res ; deg++ )
	{
		float degree = deg;
		//radian resolution = res
		float res = params.search_res;
		float theta = pi*(degree/res);
		cv::Point2f search_edge;


		//compute the edge point
		search_edge.x = (robot_center.x + ((cos(theta) * params.search_radius)));
		search_edge.y = (robot_center.y - ((sin(theta) * params.search_radius)));
		
		//draw rays
		/*
		if(params.display_rays)
		{
			line(map_dis.image, robot_center, search_edge , 255);
		}
		
		*/
		
		cv::LineIterator ray(map.image, robot_center, search_edge , 8);
		
		//Iterate over each point in the ray
		for(int pos = 0 ; pos < ray.count ; pos++ , ++ray)
		{

			if(ray.pos().x > robot_center.x + params.robot_x || 
			   ray.pos().x < robot_center.x - params.robot_x ||
			   ray.pos().y < robot_center.y - params.robot_y )
			{
				/*
				@todo This is most definitely the wrong way to access pixels on
				the iterator since it should return a pointer
				but I give up for now
				*/
				//drawing the white lines on the screen where the raytraces happen
				if(params.display_rays)
				{
					map_dis.image.at<float>( ray.pos().y , ray.pos().x ) = 200; 
				}
				
				//magnitude divided by the distance squared
				double dist = (params.dist_scale_x/1000 * pow(abs(ray.pos().x - robot_center.x ),2) +  params.dist_scale_y/1000 * pow(abs(ray.pos().y -robot_center.y ),2));
				//if the color is detected or not
				float mag = map.image.at<float>( ray.pos().y , ray.pos().x );
				
				//Slow down based on the distance to an object
				twist.linear.x -= (params.twist_scalar_y/1000  * mag/dist * sin(theta));

				//Adjust turning speed based on distance to an object
				twist.angular.z += (params.twist_scalar_z/1000  * mag/dist * cos(theta));	   
			}
			//twist.linear.x = twist.linear.x*100/params.search_res ;
			//twist.angular.z = twist.angular.z*100/params.search_res ;

		}	
	}

	/* DRAWING THE COMPASS LINE ON THE DISPLAY IMAGE */
	cv::Point2i line;

	line.x = robot_center.x - params.compas_length * twist.angular.z;
	line.y = robot_center.y - params.compas_length * twist.linear.x;
			
	//draw the robots heading on the map
	cv::line(map_dis.image, robot_center, line , 255 , 5);
	
	//draw target
	line.y = robot_center.y - params.compas_length * sin(target.target_heading);
	line.x = robot_center.x - params.compas_length * cos(target.target_heading);
	
	//Draw the destination line on the map
	cv::line(map_dis.image, robot_center,line,255 , 3);   
	/* END OF DRAWING THE COMPASS LINE ON THE DISPLAY IMAGE */
	return twist;
}

/***********************************************************
* @fn stop_robot()
* @brief sends a velocity of zero
* @post returns a twist of zero
***********************************************************/
geometry_msgs::Twist stop_robot()
{
	geometry_msgs::Twist twist;
	
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
    
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;
    
	return twist;
}

/* This has to do with edges and should be reintegrated
void check_queue()
{

	if(stat_time_q.size() != 0 && edges_time_q.size() != 0)
	{
		if(stat_time_q.front() == edges_time_q.front())
		{
			map_changed = 1;
			//ROS_INFO("2");
		}
		else if(stat_time_q.front() > edges_time_q.front())
		{
			edges_time_q.pop();
			edges_q.pop();
			check_queue();
			//ROS_INFO("3");
		}
		else if(stat_time_q.front() < edges_time_q.front())
		{
			stat_time_q.pop();
			stat_q.pop();
			check_queue();
			//ROS_INFO("4");
		}
	}
	
}
*/

/***********************************************************
* @fn main(int argc, char **argv)
* @brief starts the Pot_Nav node and publishes twist when
* it gets a new image assuming 30Hz
***********************************************************/
int main(int argc, char **argv)
{
	//Create ros objects
	ros::init(argc, argv, "Pot_Nav");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	//Setup the first target
	target.target_heading = pi/2;
	target.distance = 0;
	target.waypoint = 0;
	target.stop_robot = false;
	target.done = false;

	//setup dynamic reconfigure
	dynamic_reconfigure::Server<mst_navigation::Pot_Nav_ParamsConfig> srv;
	dynamic_reconfigure::Server<mst_navigation::Pot_Nav_ParamsConfig>::CallbackType f;
	f = boost::bind(&setparamsCallback, _1, _2);
	srv.setCallback(f);

	//Create subscriptions
	//Edges are not currently used
	//image_sub_edges = it.subscribe( n.resolveName("image_edges") , 10, edgesCallback );
	image_sub_stat = it.subscribe( n.resolveName("image_stat") , 10, statCallback );
	target_sub = n.subscribe( "/target" , 5, targetCallback );

	//Create publishers
	twist_pub = n.advertise<geometry_msgs::Twist>( "nav_twist" , 5 );
	map_pub = it.advertise( "map_image" , 5 );
    
	//set rate to 30Hz
	ros::Rate loop_rate(60);
    
	//run main loop
	while (ros::ok())
	{
		//Check for any new messages
		ros::spinOnce();
		
		//finds and publishes new twist
		//check_queue();
		if(map_changed)
		{
			geometry_msgs::Twist twist;
			
			/* This block has to do with edges
			map.image = map.image * params.previous_per/100 + stat_q.front() + edges_q.front() ;
			
			stat_time_q.pop();
			stat_q.pop();
			edges_time_q.pop();
			edges_q.pop();
			*/
			
			//we are adding the stat (which contains the green value) to the 
			//map.image
			map.image = map.image * params.previous_per/100 + stat;
			
			//Find optimal direction to next target
			twist = find_twist();

			//Check if the robot has reached the last waypoint
			if(target.stop_robot)
			{
				//twist = find_twist();
				twist = stop_robot();
			}
			//else
			//{
			//
			//	twist = find_twist();
			//}
			
			//publish twist
			twist_pub.publish(twist);
			
			//publish map
			map_pub.publish(map_dis.toImageMsg());
			
			map_changed = 0;
		}
		
		loop_rate.sleep();
  }
   
  return 0;
}

