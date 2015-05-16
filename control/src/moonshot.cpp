/**
 * @file moonshot.cpp
 * @author Matt Anderson <mia2n4>
 * @author Ryan Loeffelman <rjlt3c>
 * @brief The control node for moonshot
 */

#include <control.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <string_constants.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>

ros::Publisher conveyer_pub;
ros::Publisher dump_pub;
ros::Publisher hardStop_pub;
const float maxConveyerSpeed = 225;
//check to see if robot should be frozen
bool freeze = false;

std_msgs::Int16 conveyer;
std_msgs::Int8 dump;
std_msgs::Bool hardStop;
//Store controller data to send to arduino 

/*******************************************************************************
 * @fn estop()
 * @brief cease's all action if the robot is told to break
 * @post The Robot will cease all action upon calling this function
 ******************************************************************************/
void estop()
{
        //This is where you put things to stop the robot.  You could also do it 
        //in control, up to you
    conveyer.data = 0;
    dump.data = 0;
    
    hardStop.data = true;
    hardStop_pub.publish(hardStop);
}


/*******************************************************************************
 * @fn joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
 * @brief callback function for the xbox controller, maps the inputs from joy to
 *       global variables
 * @param joy the message from joy node
 * @pre A valid joy message has been recieved
 * @post The robot state will switch or the cmd_vel message will change
 ******************************************************************************/
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {

    float left_trig = (joy->axes[2] * (-1.0) + 1.0)/2.0;
    float right_trig = (joy->axes[5] * (-1.0) + 1.0)/2.0;  

    conveyer.data = 0;
    dump.data = 0;  

	//Checks to see if the stop button on the controller has been pressed
    if(joy->buttons[1] == 1)
    {
		freeze = true;
    }
    else if(joy->buttons[2] == 1)
	{
		freeze = false;
	}

	//if the above button has been pressed, then freeze robot
    if(!freeze)
    {
	//Check for dpad buttons to run the dump 
    	dump.data = -joy->axes[6];
 
        //Check for triggers to run the conveyer
        if(right_trig >0.05)
        {
            conveyer.data = right_trig*maxConveyerSpeed;
        }
        else if(left_trig > 0.05)
        {
            conveyer.data = -left_trig*maxConveyerSpeed;
        }
    }
    else
    {
        estop();
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
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("stopService");
    std_srvs::Empty srv;
    bool errorMsg = false;
    

    //Create subscriptions
    joy_sub = n.subscribe<sensor_msgs::Joy>(TOPIC_JOY, 1, joy_callback);

    //Create publishers
    conveyer_pub = n.advertise<std_msgs::Int8>(TOPIC_CONVEYER, 1);
    dump_pub = n.advertise<std_msgs::Int8>(TOPIC_DUMP, 1);
    hardStop_pub = n.advertise<std_msgs::Bool>(TOPIC_HARDSTOP, 1);

    //Set ros loop rate to 30Hz
    ros::Rate loop_rate(15);

//repeate as long as ros is running
    //while(ros::ok())
	do    
	{
		//if the compputer dissconects the send an error msg to the screen and freeze robot
        if(!client.call(srv))
        {
	    	if(!errorMsg)
	    	{
            	ROS_ERROR("Computer has disconected from robot!");
				errorMsg = true;
	    	}
            	estop();
        }

		//as long as its connected, say so
		else if(errorMsg)
		{
	    	ROS_INFO("Computer Connected");
	   		errorMsg = false;
		}
    
		//update ros    
        ros::spinOnce();
        
		//loop_rate.sleep();
    }while(ros::ok());
    
    return 0;
}

