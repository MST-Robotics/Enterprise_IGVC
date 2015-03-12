/******************************************************************************
* @file tracking.cpp
* @author Matt Anderson <mia2n4@mst.edu>
* @brief The tracking code for Enterprise
******************************************************************************/

#include "TrackingNode.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking");
    
    TrackingNode node;

    ros::Rate rate(ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        node.update();
        rate.sleep();
    }

    return 0;
}
