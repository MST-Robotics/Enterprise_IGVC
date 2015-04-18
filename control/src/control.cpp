/*******************************************************************************
 * @file control.cpp
 * @author Michael Lester
 * @version 1.1
 * @date 4/3/13
 * @brief controlls which output goes to motors
 ******************************************************************************/
#include <control.h>

#include <string_constants.h>


/*******************************************************************************
 * Message Callbacks
 *******************************************************************************/

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
        change_mode(arcade_mode);
    } else if (check_togg(joy->buttons[joy_button_X], joy_button_X)) {
        change_mode(diff_mode);
    }
}

/*******************************************************************************
 * @fn check_shift(const sensor_msgs::Joy::ConstPtr& joy)
 * @brief Check to see if the speed of the robot should be changed
 * @param joy the message from joy node
 * @pre A valid joy message has been recieved
 * @post The robot will either shift up or down, this will increase the speed if
 *       rb is pressed and will decrease the speed if lb is pressed.
 ******************************************************************************/
void check_shift(const sensor_msgs::Joy::ConstPtr& joy) {
    //Up shift
    if (check_togg(joy->buttons[joy_r_button], joy_r_button)) {
        speed_mult += 0.1f;

        //Make sure that speed does not shift higher than 100%
        if (speed_mult >= 1.0f) {
            speed_mult = 1.0f;
        }
    }

    //Down shift
    else if (check_togg(joy->buttons[joy_l_button], joy_l_button)) {
        speed_mult -= 0.1f;

        //Make sure that speed does not shift lower than 10%
        if (speed_mult <= 0.1f) {
            speed_mult = 0.1f;
        }
    }
}

void update_velocity(float right_vel, float left_vel) {

    control::Velocity velocity;

    //Modify the velocity message to send to the motors
    velocity.right_vel = abs(right_vel) * MOTOR_SPEED_MAX * speed_mult;
    velocity.right_dir = right_vel > 0.0f;

    velocity.left_vel = abs(left_vel) * MOTOR_SPEED_MAX * speed_mult;
    velocity.left_dir = left_vel > 0.0f;

    //Publish the message to the motor controller
    motor_pub.publish(velocity);

}

float get_right_velocity(float linearVelocity, float angularVelocity) {
    float v_right = (2 * linearVelocity + angularVelocity * ROBOT_BASE)
            / (2 * WHEEL_RADIUS);
    return mapminmax(v_right, ROTATION_MAX, ROTATION_MIN, 1.0f, -1.0f);
}

float get_left_velocity(float linearVelocity, float angularVelocity) {
    float v_left = (2 * linearVelocity - angularVelocity * ROBOT_BASE)
            / (2 * WHEEL_RADIUS);
    return mapminmax(v_left, ROTATION_MAX, ROTATION_MIN, 1.0f, -1.0f);
}

/*******************************************************************************
 * @fn joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
 * @brief callback function for the xbox controller, maps the inputs from joy to
 * 		 global variables
 * @param joy the message from joy node
 * @pre A valid joy message has been recieved
 * @post The robot state will switch or the cmd_vel message will change
 *******************************************************************************/
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
    //Xbox buttons are defined in the header

    //xbox controller axes
    joy_rightstick_x = joy->axes[4];
    joy_rightstick_y = joy->axes[3];

    joy_leftstick_x = joy->axes[1];
    joy_leftstick_y = joy->axes[0];

    joy_r_trigger = joy->axes[5];
    joy_l_trigger = joy->axes[2];

    // Say stuff when you press the d-pad buttons
    if (check_togg(joy->buttons[joy_dpad_up], joy_dpad_up))
    {
	sc->say("My name is Enterprise");
    }
    else if (check_togg(joy->buttons[joy_dpad_dwn], joy_dpad_dwn))
    {
    sc->say("Welcome to Missouri S and T");
    }
    else if (check_togg(joy->buttons[joy_dpad_l], joy_dpad_l))
    {
    sc->say("It smells like up dog in here");
    }
    else if (check_togg(joy->buttons[joy_dpad_r], joy_dpad_r))
    {
	sc->say(" Hello");
    }

    switch (robot_mode) {
    case arcade_mode:

        check_shift(joy);

        // send the updated velocity to the motor controller
        update_velocity(
            get_right_velocity(joy_leftstick_x, joy_rightstick_y),
            get_left_velocity(joy_leftstick_x, joy_rightstick_y)
        );

        //Check to see if the mode needs to be changed
        check_mode(joy);

        break;

    case diff_mode:

        check_shift(joy);

        // send the updated velocity to the motor controller
        update_velocity(joy_rightstick_x, joy_leftstick_x);

        //Check to see if the mode needs to be changed
        check_mode(joy);

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
 * @post updates the nav_twist variable if we're in autonomous mode
 * @param a ros message of a twist from navigation
 ***********************************************************/
void navigation_callback(const geometry_msgs::Twist twist) {
    if (robot_mode == autonomous) {
        // send the updated velocity to the motor controller
        update_velocity(
            get_right_velocity(twist.linear.x, twist.angular.z),
            get_left_velocity(twist.linear.x, twist.angular.z)
        );
    }
}

/***********************************************************
 * @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
 * @brief callback for the reconfigure gui
 * @pre has to have the setup for the reconfigure gui
 * @post changes the parameters
 ***********************************************************/
void setparamsCallback(control::Control_ParamsConfig &config, uint32_t level) {
    // Get the parameters from the parameter server
    params = config;
}

/*******************************************************************************
 * @fn main(int argc, char **argv)state.buttons[state.MSG_BTN_HOME]
 * @brief starts the Control node and publishes motor commands
 ******************************************************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "Control");
    ros::NodeHandle n;
    
    //Setup sound client for TTS
    sc.reset(new sound_play::SoundClient());

    //Setup initial robot state variables
    robot_init = true;
    robot_mode = standby;
    autonomous_mode = navigation;

    //Find the topic name
    std::string nav;
    nav = n.resolveName(TOPIC_NAVIGATION_TWIST);

    //Ensure that the twist topic from autonomous is named nav_twist
    if (nav == TOPIC_NAVIGATION_TWIST) {
        ROS_WARN("Control: navigation twist has not been remapped! "
                "Typical command-line usage:\n"
                "\t$ ./Contestop_pubrol twist:=<twist topic> [transport]");
    }

    //Create subscriptions
    nav_sub = n.subscribe(nav, 100, navigation_callback);
    xbox_state_sub = n.subscribe<sensor_msgs::Joy>(TOPIC_JOY, 1, joy_callback);

    //Create publishers
    motor_pub = n.advertise<control::Velocity>(TOPIC_VELOCITY, 1);
    light_pub = n.advertise<std_msgs::UInt8>(TOPIC_INDICATOR_LIGHT, 1);

    //Set ros loop rate to 30Hz
    ros::Rate loop_rate(30);

    //Main loop
    while (ros::ok()) {
        //Check for new messages
        ros::spinOnce();

        if (robot_init) {
            //First time initialization
            sc->say("Hello World. My name is S and T Enterprise.");
            robot_init = false;
        }

        if (robot_mode == standby)
            stop_robot();

        //Wait until the 30Hz interval has ended
        loop_rate.sleep();
    }

    return 0;
}

/*******************************************************************************
 * @fn update_light(uint8_t value)
 * @brief Update the light to the specified value and publish a message
 * @post A new message is published with the specified light value
 * @param uint8_t value the new value for the light
 *******************************************************************************/
void update_light(uint8_t value) {
    std_msgs::UInt8 lightPulse;
    lightPulse.data = value;
    light_pub.publish(lightPulse);
}

/*******************************************************************************
 * @fn change_mode(Mode new_mode)
 * @brief Changes the global variable for robot mode and updates the light
 * @post Robot mode is changed to new_mode and light is updated correctly
 * @param Mode new_mode the new mode to change the robot to
 *******************************************************************************/
void change_mode(Mode new_mode) {
    robot_mode = new_mode;

    switch (robot_mode) {
    case standby:
        ROS_INFO("Control: Standby Mode");
        sc->say("Entering Standby");
        update_light(0);
        break;
    case autonomous:
        ROS_INFO("Control: Autonomous Mode");
        sc->say("Entering Autonomous mode.");
        update_light(1);
        break;
    case arcade_mode:
        ROS_INFO("Control: Manual Xbox Arcade Drive Mode");
        sc->say("Entering arcade drive manual control");
        update_light(0);
        break;
    case diff_mode:
        ROS_INFO("Control: Manual Xbox Differential Drive Mode");
        sc->say("Entering tank drive manual control");
        update_light(0);
        break;
    }
}

/*******************************************************************************
 * @fn stop_robot()
 * @brief stops the robot from moving by sending a velocity message with 0s
 * @pre Robot should be initialized
 * @post The robot is no longer moving
 *******************************************************************************/
void stop_robot() {
    update_velocity(0.0f, 0.0f);
}
