/*******************************************************************************
 * @file control.h
 * @author Michael Lester
 * @version 1.1
 * @date 4/3/13
 * @brief controlls which output goes to motors 
 * @Re-made for xbox_Mode
 ******************************************************************************/
 
#ifndef CONTROL_H
#define CONTROL_H
 
/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"
/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sound_play/sound_play.h>
#include <mst_position/target_heading.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/UInt8.h>
#include <control/Velocity.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <control/Control_ParamsConfig.h>

/***********************************************************
* Subscribers
***********************************************************/
ros::Subscriber                 xbox_state_sub;
ros::Subscriber                 nav_sub;


/***********************************************************
* Publishers
***********************************************************/
ros::Publisher                  motor_pub;
ros::Publisher                  sound_pub;
ros::Publisher                  light_pub;



/***********************************************************
* Constants
***********************************************************/
//Soundplay constants for TTS
//In order for it to be global it must be used as a pointer and initialized 
//in main
boost::shared_ptr<sound_play::SoundClient> sc;

const float ROBOT_BASE  = 0.62; //meters
const float WHEEL_RADIUS  = 0.19; //meters

const float JOY_TRIGGER_MAX = 1.0f;
const float JOY_TRIGGER_MIN = -1.0f;

const uint8_t MOTOR_SPEED_MAX = 255;

const float ROTATION_MAX = (1.0f + ROBOT_BASE / 2.0f) / WHEEL_RADIUS;
const float ROTATION_MIN = -ROTATION_MAX;

//Values used to minimize the contoler response when nothing is pressed
const float DEADZONE_JOYSTICK = .05;
const float DEADZONE_TRIGGER = .1;

/*-----------------------------------
	Velocity and sensor data variables
	-----------------------------------*/
    double m_linear; //These are for simulation
    double m_angular;
    //xbox movement x=linear y=angular
    float  joy_rightstick_x;
    float  joy_rightstick_y;
    float  joy_leftstick_x; 
    float  joy_leftstick_y;
    float  joy_r_trigger;
    float  joy_l_trigger;
    //xbox buttons to assign cmds
    #define  joy_button_A   0
    #define  joy_button_B   1
    #define  joy_button_X   2
    #define  joy_button_Y   3
    #define  joy_r_button   5
    #define  joy_l_button   4
    #define  joy_start_b    7
    #define  joy_back_b     6
    #define  joy_dpad_up    13
    #define  joy_dpad_dwn   14
    #define  joy_dpad_l     11
    #define  joy_dpad_r     12
    #define  joy_light      8
    #define  joy_l_stick    9
    #define  joy_r_stick    10

    static bool check_togg(bool, int);
    
//Enumorator for each mode 
enum Mode
{
    standby,
    arcade_mode,
    autonomous,
	diff_mode
};
Mode robot_mode;

//Enumerator for autonomous mode
enum Autonomous_Mode
{
    navigation,
    autonomous_waypoints
};
Autonomous_Mode autonomous_mode;

control::Control_ParamsConfig params;

/***************************************************************** 
*This is where we changed it from wii_twist to geometry_twist.
*If using a different remote in the future, assign geometry_twist
*so we have a universal twist for any controller
*
*See joy_callback
******************************************************************/
bool xtogg[30];
bool robot_init;
bool done_togg = 0;
bool stop = true;
int last_msg_waypoint = 0;
float speed_mult = 0.4f;	//Speed multiplier for robot, shifts up and down

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;
using namespace std;

/***********************************************************
* Function prototypes
***********************************************************/
void change_mode(Mode new_mode);
void stop_robot();
bool check_togg(bool, int);

float mapminmax(float x, float xmax, float xmin, float ymax, float ymin) 
{
    return (ymax - ymin) * (x - xmin) / (xmax - xmin) + ymin;
}

/*******************************************************************************
 * @fn check_togg(bool button_state, int button_position)
 * @brief Toggles an xbox buttons value in xtogg array to button state
 * @pre An xbox controller should be enabled and a button should be pressed
 * @post The value of the button in xtogg array is toggled based on its previous
 *       value
 * @param bool button_state The previous state of the button
 * @param bool button_position The number assigned to the button by the joy node
 *******************************************************************************/
bool check_togg(bool button_state, int button_position) {
    bool togg = false;

    if (button_state && !xtogg[button_position]) {
        xtogg[button_position] = true;

        togg = true;
    } else if (!button_state) {
        xtogg[button_position] = false;
    }

    return togg;
}

#endif
    

