/**
 * @file pathPlanning.cpp
 * @author Matt Anderson <mia2n4@mst.edu>
 * @brief This is the executable file for the path planning node
 */

#include "PathPlanningNode.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planing");

    PathPlanningNode node;

    while(ros::ok())
    {
        ros::spinOnce();
        node.update();
    }

    return 0;
}
