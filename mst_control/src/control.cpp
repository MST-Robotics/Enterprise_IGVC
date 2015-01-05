/*******************************************************************************
 * @file control.cpp
 * @author Michael Lester
 * @version 1.1
 * @date 4/3/13
 * @brief controlls which output goes to motors
 ******************************************************************************/
#include "control.h"

/*******************************************************************************
 * Message Callbacks
 *******************************************************************************/

/*******************************************************************************
 * @fn pos_callback(const mst_position::Target_Heading::ConstPtr& msg)
 * @brief callback function for the target heading
 * @param mst_position::Target_Heading::ConstPtr& msg A Target_Heading message
 *        which checks if the robot has reached the last waypoint
 * @pre Waypoints exist in parameter server
 * @post The robot will travel to each waypoint before finishing
 *******************************************************************************/
void pos_callback(const mst_position::target_heading::ConstPtr& msg) {
    if (msg->done && !done_togg) {
        change_mode(standby);
        done_togg = true;
    } else if (!msg->done) {
        done_togg = false;
    }

    if (msg->waypoint != last_msg_waypoint) {
        say("moving to next waypoint");
        last_msg_waypoint = msg->waypoint;
    }
}

/*******************************************************************************
 * @fn check_mode(const sensor_msgs::Joy::ConstPtr& joy)
 * @brief Check to see if the mode needs to be changed
 * @param joy the message from joy node
 * @pre A valid joy message has been recieved
 * @post The robot state will switch if needed
 *******************************************************************************/
void check_mode(const sensor_msgs::Joy::ConstPtr& joy) {
    //Check to see if the mode needs to be changed
    if (check_togg(joy->buttons[joy_button_Y], joy_button_Y)) {
        change_mode(autonomous);
    } else if (check_togg(joy->buttons[joy_button_B], joy_button_B)) {
        change_mode(standby);
    } else if (check_togg(joy->buttons[joy_button_A], joy_button_A)) {
        change_mode(xbox_mode);
    } else if (check_togg(joy->buttons[joy_button_X], joy_button_X)) {
        change_mode(diff_mode);
    }
}

/*******************************************************************************
 * @fn xbox_callback(const sensor_msgs::Joy::ConstPtr& joy)
 * @brief callback function for the xbox controller, maps the inputs from joy to
 * 		 global variables
 * @param joy the message from joy node
 * @pre A valid joy message has been recieved
 * @post The robot state will switch or the cmd_vel message will change
 *******************************************************************************/
void xbox_callback(const sensor_msgs::Joy::ConstPtr& joy) {
    //Xbox buttons are defined in the header

    //xbox controller axes
    joy_rightstick_x = joy->axes[4];
    joy_rightstick_y = joy->axes[3];

    joy_leftstick_x = joy->axes[1];
    joy_leftstick_y = joy->axes[0];

    joy_r_trigger = joy->axes[5];
    joy_l_trigger = joy->axes[2];

    switch (mode_) {
    case xbox_mode:
        //initalize twist
        geometry_twist.angular.x = 0;
        geometry_twist.angular.y = 0;
        geometry_twist.angular.z = 0;
        geometry_twist.linear.x = 0;
        geometry_twist.linear.y = 0;
        geometry_twist.linear.z = 0;

        //Controller Behavior in controller mode
        //Check to see if the mode needs to be changed
        check_mode(joy);

        //Modify the twist message to send to the motors
        geometry_twist.angular.z = (joy_rightstick_y) * ROT_SPEED;
        geometry_twist.linear.x = (joy_leftstick_x) * LINEAR_SPEED;
        break;

    case diff_mode:
        //initalize twist
        geometry_twist.angular.x = 0;
        geometry_twist.angular.y = 0;
        geometry_twist.angular.z = 0;
        geometry_twist.linear.x = 0;
        geometry_twist.linear.y = 0;
        geometry_twist.linear.z = 0;

        //Controller Behavior in controller mode
        //Check to see if the mode needs to be changed
        check_mode(joy);

        // normalize the trigger values from [-1.0, 1.0] to [0.0, 1.0]
        joy_r_trigger = (1.0f - joy_r_trigger) / (2.0f);
        joy_l_trigger = (1.0f - joy_l_trigger) / (2.0f);

        //Modify the twist message to send to the motors
        geometry_twist.linear.x = -(joy_r_trigger) * LINEAR_SPEED;
        geometry_twist.linear.x += (joy_l_trigger) * LINEAR_SPEED;
        break;

    case autonomous:
    case standby:
        //Check to see if the mode should be changed
        check_mode(joy);
        break;
    }
}

/***********************************************************
 * @fn navigation_callback(const geometry_msgs::Twist::ConstPtr& twist)
 * @brief decides on forwarding navigation comands to the motors
 * @pre takes in a ros message of a twist from navigation and
 *      needs mode variable to be initalized
 * @post publishes a CV_32FC1 image using cv_bridge
 * @param takes in a ros message of a raw or cv image
 ***********************************************************/
void navigation_callback(const geometry_msgs::Twist twist) {
    if (mode_ == autonomous) {
        //initalize twist
        nav_twist.angular.x = 0;
        nav_twist.angular.y = 0;
        nav_twist.angular.z = twist.angular.z;
        nav_twist.linear.x = twist.linear.x;
        nav_twist.linear.y = 0;
        nav_twist.linear.z = 0;
    }
}

/***********************************************************
 * @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
 * @brief callback for the reconfigure gui
 * @pre has to have the setup for the reconfigure gui
 * @post changes the parameters
 ***********************************************************/
void setparamsCallback(mst_control::Control_ParamsConfig &config,
        uint32_t level) {
    // Get the parameters from the parameter server
    params = config;
}

/*******************************************************************************
 * @fn main(int argc, char **argv)state.buttons[state.MSG_BTN_HOME]
 * @brief starts the Control node and publishes motor commands
 *******************************************************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "Control");
    ros::NodeHandle n;

    //Light Information
    std_msgs::Int8 lightPulse;
    lightPulse.data = 0;

    //Setup initial robot state variables
    bool stopped = true;
    robot_init = true;
    mode_ = standby;
    autonomous_mode = navigation;

    //Find the topic name
    std::string nav;
    nav = n.resolveName("nav_twist");

    //Ensure that the twist topic from autonomous is named nav_twist
    if (nav == "nav_twist") {
        ROS_WARN(
                "Control: navigation twist has not been remapped! Typical command-line usage:\n"
                        "\t$ ./Contestop_pubrol twist:=<twist topic> [transport]");
    }

    //Create subscriptions
    nav_sub = n.subscribe(nav, 100, navigation_callback);
    xbox_state_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, xbox_callback);

    //Create publishers
    motor_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    light_pub = n.advertise<std_msgs::Int8>("indicator_light", 1);

    //Set ros loop rate to 30Hz
    ros::Rate loop_rate(30);

    //Main loop
    while (ros::ok()) {
        //Check for new messages
        ros::spinOnce();

        if (robot_init) {
            //First time initialization
            say(
                    "Hello World. My name is S and T Enterprise. Please press the EX box button to connect");
            robot_init = false;
        }

        switch (mode_) {
        case standby:
            //Light should be solid
            lightPulse.data = 0;
            light_pub.publish(lightPulse);

            //Ensure the robot isn't moving
            stop_robot();
            stopped = true;

            break;

        case xbox_mode:
        case diff_mode:
            //Light should be solid
            lightPulse.data = 0;
            light_pub.publish(lightPulse);

            //Publish the message created from joy
            motor_pub.publish(geometry_twist);
            stopped = false;

            break;

        case autonomous:
            //Light should blink
            lightPulse.data = 1;
            light_pub.publish(lightPulse);

            //Publish the twist message created from navigation
            motor_pub.publish(nav_twist);
            stopped = false;

            break;

        default:
            stop_robot();
            stopped = true;
        }

        //Wait until the 30Hz interval has ended
        loop_rate.sleep();
    }

    return 0;
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

/*******************************************************************************
 * @fn change_mode(Mode new_mode)
 * @brief Changes the global variable for robot mode
 * @post Robot mode is changed to new_mode
 * @param Mode new_mode the new mode to change the robot to
 *******************************************************************************/
void change_mode(Mode new_mode) {
    mode_ = new_mode;

    if (mode_ == standby) {
        ROS_INFO("Control: Standby Mode");
    } else if (mode_ == autonomous) {
        ROS_INFO("Control: Autonomous Mode");
    } else if (mode_ == xbox_mode) {
        ROS_INFO("Control: Xbox Mode");
    } else if (mode_ == diff_mode) {
        ROS_INFO("Control: Manual Differential Drive Mode");
    }
}

/*******************************************************************************
 * @fn say(std::string say)
 * @brief This will have the robot use its speakers to say the string with TTS
 * @pre sound_play is running
 * @post The robot plays the text through the speakers
 * @param std::string say The string that should be read aloud using TTS
 *******************************************************************************/
void say(std::string say) {
    /*sound_play::SoundRequest sound;

    sound.sound = sound.SAY;
    sound.command = sound.PLAY_ONCE;
    sound.arg = say;

    sound_pub.publish(sound);*/
}

/*******************************************************************************
 * @fn stop_robot()
 * @brief stops the robot from moving by sending a twist message populated by 0s
 * @pre Robot should be initialized
 * @post The robot is no longer moving
 *******************************************************************************/
void stop_robot() {
    geometry_msgs::Twist stop_twist;

    stop_twist.angular.x = 0;
    stop_twist.angular.y = 0;
    stop_twist.angular.z = 0;
    stop_twist.linear.x = 0;
    stop_twist.linear.y = 0;
    stop_twist.linear.z = 0;

    motor_pub.publish(stop_twist);
}
