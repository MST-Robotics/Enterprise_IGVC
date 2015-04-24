/**
 * @file moonshot.cpp
 * @author Matt Anderson <mia2n4>
 * @brief The control node for moonshot
 */

#include <control.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <string_constants.h>

ros::Publisher conveyer_pub;
ros::Publisher dump_pub;
const float maxConveyerSpeed = 225;

/*******************************************************************************
 * @fn joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
 * @brief callback function for the xbox controller, maps the inputs from joy to
 *       global variables
 * @param joy the message from joy node
 * @pre A valid joy message has been recieved
 * @post The robot state will switch or the cmd_vel message will change
 ******************************************************************************/
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
    std_msgs::Int16 conveyer;
    std_msgs::Int8 dump;
    
    conveyer.data = 0;
    
    float left_trig = (joy->axes[2] * (-1) + 1)/2;
    float right_trig = (joy->axes[5] * (-1) + 1)/2;
    
    //Check for triggers to run the conveyer
    if(right_trig > 0)
    {
        conveyer.data = right_trig*maxConveyerSpeed;
    }
    else if(left_trig > 0)
    {
        conveyer.data = -left_trig*maxConveyerSpeed;
    }

    //Check for dpad buttons to run the dump
    if(joy->buttons[joy_dpad_l])
    {
        dump.data = -1;
    }
    else if(joy->buttons[joy_dpad_r])
    {
        dump.data = 1;
    }

    conveyer_pub.publish(conveyer);
    dump_pub.publish(dump);
}

/*******************************************************************************
 * @fn main(int argc, char **argv)state.buttons[state.MSG_BTN_HOME]
 * @brief starts the Control node and publishes motor commands
 ******************************************************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "Moonshot_Control");
    ros::NodeHandle n;
    
    ros::Subscriber joy_sub;

    //Create subscriptions
    joy_sub = n.subscribe<sensor_msgs::Joy>(TOPIC_JOY, 1, joy_callback);

    //Create publishers
    conveyer_pub = n.advertise<std_msgs::Int16>(TOPIC_CONVEYER, 1);
    dump_pub = n.advertise<std_msgs::Int8>(TOPIC_DUMP, 1);

    //Set ros loop rate to 30Hz
    ros::Rate loop_rate(30);

    ros::spin();

    return 0;
}






