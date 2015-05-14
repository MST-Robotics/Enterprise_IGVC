/**
 * @file stopService.cpp
 * @author Ryan Loeffelman <rjlt3c>
 * @brief The node to make sure the controller is still conected
 */
 
#include <ros/ros.h>
#include <std_srvs/Empty.h>

bool check(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stopService");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("stopService", check);

    ros::spin();
    
    return 0;
}
