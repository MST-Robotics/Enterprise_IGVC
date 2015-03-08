/******************************************************************************
* @file navigation.cpp
* @author Matt Anderson <mia2n4@mst.edu>
* @brief The navigation code written for Enterprise
******************************************************************************/

#include "navigation.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");
    
    NavigationServer server;

    ros::Rate rate(ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        server.update();
        rate.sleep();
    }

    return 0;
}
