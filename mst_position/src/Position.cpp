/*******************************************************************************
 * @file Position.cpp
 * @author James Anderson <jra798>
 * @maintainer Matt Anderson <mia2n4>
 * @version 1.0
 * @brief publishes transform tree using gps and headings taken in from garmin
 * and provides a service to transform gps coardinates into the world
 * frame
 * @todo change degree to radian conversion to a constant variables and create a
 * function for the conversion, use a point 2d object to represent lat long 
 * variables
 * need to also do comments for complicated functions (like find heading)
 * fill out description for Odometry
 ******************************************************************************/

/***********************************************************
 * ROS specific includes
 ***********************************************************/
#include <ros/ros.h>

/***********************************************************
 * Message includes
 ***********************************************************/
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "mst_position/target_heading.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

/***********************************************************
 * Other includes
 ***********************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <dynamic_reconfigure/server.h>
#include <mst_position/Position_ParamsConfig.h>
#include <tf/transform_broadcaster.h>

/***********************************************************
 * Global variables
 ***********************************************************/

ros::Subscriber                 imu_sub;
ros::Subscriber                 garmin_sub;

ros::Publisher                  target_pub;
ros::Publisher                  odom_pub;
ros::Publisher                  goal_pub;

ros::ServiceServer              gps_to_pose;

bool                            map_changed = 0;
bool                            first_callback = 1;

bool                            gps_fix;

ros::Time                       current_time;
double                          current_lat;
double                          current_lon;
double                          current_head;

double                          inital_lat;
double                          inital_lon;
double                          inital_head;

struct waypoint {
    double lat, lng;
    int priority;
    double limit;
    bool complete;
};

waypoint                        way[10];

bool                            skip_waypoint;
bool                            stoped;

int                             current_priority;

bool inital_gps = true;

double chosen_lat;
double chosen_lon;

double chosen_dist = -1;
double chosen_brng = 0;
int chosen_index = 99;

int numPts = 0;
int avgY = 0;
int avgX = 0;

mst_position::target_heading    target_heading;

mst_position::Position_ParamsConfig    params;

/***********************************************************
 * Function prototypes
 ***********************************************************/
//void send_target(bool);
double toRads(double degrees);
void reset_waypoints();
void read_waypoints();
double find_dist(double, double, double, double);
double find_heading(double, double, double, double);
int find_target();
mst_position::target_heading compute_msg(int);
nav_msgs::Odometry odom_msg();
void odom_set_origin();

/***********************************************************
 * Namespace Changes
 ***********************************************************/
using namespace std;

/***********************************************************
 * Defines
 ***********************************************************/

#define pi M_PI
//ratio for conversion to radians
const double rc = (pi/180);
/***********************************************************
 * Message Callbacks
 ***********************************************************/

/***********************************************************
 * @fn imuCallback(const geometry_msgs::Quaternion::ConstPtr
 * & imu)
 * @brief gets the position information from the imu 
 * (magnatometer), sets the current_head = heading 
 * observed by the magnatometer.
 * @pre imu publishing Quaternion
 * @post current_head = robot's heading
 * @param takes in a imu publishing Quaternion
 ***********************************************************/
void imuCallback(const geometry_msgs::Quaternion::ConstPtr& imu) {
    current_head = atan2(imu->y, imu->x);
}

/***********************************************************
 * @fn gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& 
 * fix)
 * @brief setting the current longitude and latitude from the 
 * GPS data
 * @pre takes a GPS fix message 
 * @post current_lon and current_lat updated
 * @param takes a GPS fix message
 ***********************************************************/
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& fix) {
    if (params.use_gpsd && !params.use_dummy) {
        current_lon = fix->longitude;
        current_lat = fix->latitude;
    }
    
}

/***********************************************************
 * @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
 * @brief callback for the reconfigure gui
 * @pre has to have the setup for the reconfigure gui
 * @post changes the parameters
 ***********************************************************/
void setparamsCallback(mst_position::Position_ParamsConfig &config,
        uint32_t level) {

    if (config.skip_waypoint) {
        config.skip_waypoint = false;
        skip_waypoint = true;
    }

    //if using dummy
    if (config.use_dummy) {
        gps_fix = true;
        current_time = ros::Time::now();
        //current_lat = config.dummy_latitude; // 180 * pi;
        //current_lon = config.dummy_longitude; // 180 * pi;
        current_head = config.dummy_heading;
    }

    if (config.reset_waypoints) {
        config.reset_waypoints = false;
        reset_waypoints();
    }

    if (config.pause) {
        if (stoped) {
            stoped = false;
        } else {
            stoped = true;
        }
        config.pause = false;
    }

    // set params
    params = config;

    read_waypoints();

}

/***********************************************************
 * Private Functions
 ***********************************************************/
/***********************************************************
 * @fn toRads(double degrees){
 * @brief converts to radians from degrees
 * @pre degrees is in degrees
 * @post returns the parameters converted to radians
 ***********************************************************/
 double toRads(double degrees){ 
   degrees *= rc; 
   
   return degrees;
   
 }
 /***********************************************************
 * @fn send_target(bool skip)
 * @brief finds the best target to go to
 * @pre takes in a bool to skip the current target
 * @post publishes a message to target_heading
 ***********************************************************/
void send_target(bool skip) {

    vector<int> potential_waypoints;
    //gps_fix = false;

    for (int i = 0; i < 10; i++) {
        if (way[i].priority == current_priority && !way[i].complete) {
            potential_waypoints.push_back(i);
        }
    }
    
    if (potential_waypoints.size() <= 0) {

        if (current_priority >= 10) {
            target_heading.done = true;
            target_heading.target_heading = 0;
            target_heading.distance = 0;
            target_heading.stop_robot = true;
            target_heading.waypoint = 0;

            target_pub.publish(target_heading);
        } else {
            //increment priority and try again
            current_priority++;
            send_target(false);
        }

    } else {

        for (unsigned int j = 0; j < potential_waypoints.size(); j++) {

            // find distance using haversine formula
            int index = potential_waypoints[j];
            double lat = toRads(way[index].lat);
            double lon = toRads(way[index].lng);
            //earths radius times c
            double dist = find_dist(current_lat, current_lon, chosen_lat,
                    chosen_lon);

            if (dist < chosen_dist || chosen_dist < 0) {
                chosen_dist = dist;
                chosen_index = index;
                chosen_lat = lat;
                chosen_lon = lon;
                //find the bearing
                chosen_brng = find_heading(current_lat, current_lon, chosen_lat,
                        chosen_lon);
            }
        }

        if (chosen_dist <= params.waypoint_radius || skip) {
            way[chosen_index].complete = true;
            skip_waypoint = false;
            send_target(false);
        } else {
            target_heading.distance = find_dist(current_lat, current_lon,
                    chosen_lat, chosen_lon);
            target_heading.waypoint = chosen_index + 1;
            //output in radians
            target_heading.target_heading = chosen_brng - current_head;
            target_heading.done = false;
            target_heading.stop_robot = false;

            target_pub.publish(target_heading);
        }
    }

}

/***********************************************************
 * @fn reset_waypoints()
 * @brief resets waypoints list
 * @post priority and compleat waypoints reset
 ***********************************************************/
void reset_waypoints() {
    read_waypoints();
    for (int i = 0; i < 10; i++) {
        //reset the waypoint complete values
        way[i].complete = false;
    }

    if (params.reverse_order) {
        current_priority = 10;
    } else {
        current_priority = 1;
    }

    inital_gps = true;
}

/***********************************************************
 * @fn double find_dist(double lat1, double lon1 , double lat2 ,double lon2)
 * @brief computes distace between two gps points
 * @pre takes in lat and lon for two points
 * @post returns a double with the distace in meters between points
 ***********************************************************/
double find_dist(double lat1, double lon1, double lat2, double lon2) {
    // find distance using haversine formula, equation from:
    // www.movable-type.co.uk/scripts/latlong.htm
    // a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
    // c = 2 ⋅ atan2( √a, √(1−a) )
    // d = R ⋅ c 
  
    // radius of the earth in meters
    double R = 6371000;
    double delta_lat = lat2 - lat1;
    double delta_lon = lon2 - lon1;
    double a = sin(delta_lat / 2) * sin(delta_lat / 2)
            + cos(lat1) * cos(lat2) * sin(delta_lon / 2) * sin(delta_lon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // earths radius times c
    return R * c;
}

/***********************************************************
 * @fn double find_heading(double lat1, double lon1, double lat2 ,double lon2)
 * @brief computes heading between two gps points
 * @pre takes in latitude and longitude for two points
 * @post returns a double with the heading in radians from radian = 0
 ***********************************************************/
double find_heading(double lat1, double lon1, double lat2, double lon2) {
    // bearing is the change needed to head in the direction of interest
    // from a current heading.
    // heading is the current direction heading, though the heading returned
    // might simply be the bearing depending on the input

    double delta_lon = lon2 - lon1;
    double heading;

    //find the bearing using the equation, the angle between north and the line
    //from lat1, lon1 to lat2, lon2:
    //θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
    //equation from www.movable-type.co.uk/scripts/latlong.html
    double x = sin(delta_lon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat1) * cos(delta_lon);
    double bearing = atan2(x, y);

    //subtracting from pi/2 because pi/2 is "north" in our case
    //so what we end up with is an angle with respect to 0 radians
    //giving a "heading" in direction to lat2 and lon2
    heading = ( pi / 2) - bearing;

    return heading;

}

/***********************************************************
 * @fn read_waypoints()
 * @brief reads gps waypoints from dynamic reconfigure
 * @pre global array way must be of size 10
 * @todo this should be replaced with reading a file
 ***********************************************************/
void read_waypoints() {
    //read in waypoint 1
    way[0].lat = params.way_1_latitude;
    way[0].lng = params.way_1_longitude;
    way[0].priority = params.way_1_priority;
    way[0].limit = params.way_1_limit;

    //read in waypoint 2
    way[1].lat = params.way_2_latitude;
    way[1].lng = params.way_2_longitude;
    way[1].priority = params.way_2_priority;
    way[1].limit = params.way_2_limit;

    //read in waypoint 3
    way[2].lat = params.way_3_latitude;
    way[2].lng = params.way_3_longitude;
    way[2].priority = params.way_3_priority;
    way[2].limit = params.way_3_limit;

    //read in waypoint 4
    way[3].lat = params.way_4_latitude;
    way[3].lng = params.way_4_longitude;
    way[3].priority = params.way_4_priority;
    way[3].limit = params.way_4_limit;

    //read in waypoint 5
    way[4].lat = params.way_5_latitude;
    way[4].lng = params.way_5_longitude;
    way[4].priority = params.way_5_priority;
    way[4].limit = params.way_5_limit;

    //read in waypoint 6
    way[5].lat = params.way_6_latitude;
    way[5].lng = params.way_6_longitude;
    way[5].priority = params.way_6_priority;
    way[5].limit = params.way_5_limit;

    //read in waypoint 7
    way[6].lat = params.way_7_latitude;
    way[6].lng = params.way_7_longitude;
    way[6].priority = params.way_7_priority;
    way[6].limit = params.way_7_limit;

    //read in waypoint 8
    way[7].lat = params.way_8_latitude;
    way[7].lng = params.way_8_longitude;
    way[7].priority = params.way_8_priority;
    way[7].limit = params.way_7_limit;

    //read in waypoint 9
    way[8].lat = params.way_9_latitude;
    way[8].lng = params.way_9_longitude;
    way[8].priority = params.way_9_priority;
    way[8].limit = params.way_8_limit;

    //read in waypoint 10
    way[9].lat = params.way_10_latitude;
    way[9].lng = params.way_10_longitude;
    way[9].priority = params.way_10_priority;
    way[9].limit = params.way_10_limit;

}

/***********************************************************
 * @fn find_target()
 * @brief finds the closest waypoint to our position
 * @pre way (waypoint array) must be of size 10
 * @post returns an index that corrisponds to the closest waypoint
 * @todo might look into removing the need for a precondition 
 * for array size
 ***********************************************************/
int find_target() {

    //intitial used to keep track if the closest target and
    //lowest distance have changed, if they haven't they
    //stay -1
    double lowest_dist = -1;
    int closest_target = -1;
    
    //while closest target is not a valid target and proirty is valid, check 
    //to find the closest target.
    while (closest_target == -1
          && ((current_priority <= 10 && !params.reverse_order) 
	  || (current_priority >= 1 && params.reverse_order))) 
    {
      
        for (int i = 0; i < 10; i++) {
	    //statement shows we found a closest target canidate
            if (!way[i].complete && way[i].priority != 0
                && (((way[i].priority <= current_priority)
                && !params.reverse_order)
		|| ((way[i].priority >= current_priority)
                && params.reverse_order))) 
	    {
	      
                double lat = toRads(way[i].lat);
                double lon = toRads(way[i].lng);

                double dist = find_dist(current_lat, current_lon, lat, lon);

		//we then check to see if it is closer than the last canidate
                if (dist <= lowest_dist || lowest_dist == -1) {
                    lowest_dist = dist;
                    closest_target = i;
                }

            }
        }
        //if closest target still not found modify the priority accordingly to 
        //allow next itteration in the while loop
        if (closest_target == -1) {
            if (params.reverse_order) {
                current_priority--;
            } else {
                current_priority++;
            }
        }
    }

    return closest_target;

}


/***********************************************************
 * @fn mst_position::target_heading compute_msg(int target)
 * @brief creates a new heading ros message and initializes 
 * the variables to the current information with respect to 
 * the current target and heading.
 * @pre target is within the size of way array [0 to 9]
 * @post heading with all variables initialized is returned
 ***********************************************************/
mst_position::target_heading compute_msg(int target) {
    mst_position::target_heading heading;
    double lat = toRads(way[target].lat);
    double lon = toRads(way[target].lng);

    //trying to find how much the robot would need to turn
    //in order to turn in the direction of the given target
    //waypoint
    heading.target_heading = current_head
		  - find_heading(current_lat, current_lon, lat, lon);
    heading.distance = find_dist(current_lat, current_lon, lat, lon);
    heading.waypoint = target;
    heading.done = false;
    heading.stop_robot = false;

    return heading;
}

/***********************************************************
 * @fn nav_msgs::Odometry odom_msg()
 * @brief 
 * @pre 
 * @post 
 ***********************************************************/
// Programmer: Jason Gassel  Date: 3-5-12
// Descr: Generates Odometry message
nav_msgs::Odometry odom_msg() {
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "imu_link";

    //Calculate local coord from lat/lon
    double dist = find_dist(inital_lat, inital_lon, current_lat, current_lon);
    double theta = find_heading(inital_lat, inital_lon, current_lat, current_lon);
    double dist_x = cos(theta) * dist;
    double dist_y = sin(theta) * dist;
    double heading = current_head - inital_head;

    //Fill in odom message
    odom.pose.pose.position.x = dist_x;
    odom.pose.pose.position.y = dist_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
    double cov_x = 9999999, cov_y = 9999999, cov_z = 9999999;
    double cov_qx = 10, cov_qy = 10, cov_qz = 10;
    if (gps_fix) {
        cov_x = 10;
        cov_y = 10;
        cov_z = 10;
    }
    double temp[] = {cov_x, 0, 0, 0, 0, 0,
                     0, cov_y, 0, 0, 0, 0,
                     0, 0, cov_z, 0, 0, 0,
                     0, 0, 0, cov_qx, 0, 0,
                     0, 0, 0, 0, cov_qy, 0,
                     0, 0, 0, 0, 0, cov_qz};
    for(int i=0; i<36; ++i)
        odom.pose.covariance[i] = temp[i];
    double temp2[] = {9999999, 0, 0, 0, 0, 0,
                     0, 9999999, 0, 0, 0, 0,
                     0, 0, 9999999, 0, 0, 0,
                     0, 0, 0, 9999999, 0, 0,
                     0, 0, 0, 0, 9999999, 0,
                     0, 0, 0, 0, 0, 9999999};
    for(int i=0; i<36; ++i)
        odom.twist.covariance[i] = temp2[i];

    return odom;
}


/***********************************************************
 * @fn odom_set_origin()
 * @brief 
 * @pre 
 * @post 
 ***********************************************************/
void odom_set_origin() {
    inital_lat = current_lat;
    inital_lon = current_lon;
    inital_head = current_head;

    return;
}

/***********************************************************
 * @fn geometry_msgs::PoseStamped relative_waypoint(int target)
 * @brief sets the timestamp, frame_id, and the pose.position 
 * variables of the PoseStamped message (a default ros message)
 * @pre target is within the size of way array [0 to 9]
 * @post returns a waypoint with out id initialized or 
 * pos.orientation initialized for waypoint.
 ***********************************************************/
geometry_msgs::PoseStamped relative_waypoint(int target) {
    geometry_msgs::PoseStamped waypoint;
    //header is metadata, needs time stamp (current_time) and
    //a label (goal)
    waypoint.header.stamp = current_time;
    waypoint.header.frame_id = "goal";

    //distance and angle calculations in order to get x,y,z components 
    //for pose.position
    double dist = find_dist(current_lat, current_lon, way[target].lat,
            way[target].lng);
    double theta = find_heading(current_lat, current_lon, way[target].lat,
            way[target].lng);
    
    waypoint.pose.position.x = cos(theta) * dist;
    waypoint.pose.position.y = sin(theta) * dist;
    waypoint.pose.position.z = 0;

    return waypoint;
}

/***********************************************************
 * @fn main(int argc, char **argv)
 * @brief starts the Pot_Nav node and publishises twist when
 * it gets a new image asuming 30 hz
 ***********************************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "Position");
    ros::NodeHandle n;

    gps_fix = false;
    skip_waypoint = false;
    int target_waypoint = -1;

    //setup dynamic reconfigure
    dynamic_reconfigure::Server<mst_position::Position_ParamsConfig> srv;
    dynamic_reconfigure::Server<mst_position::Position_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
    srv.setCallback(f);

    //create subsctiptions
    imu_sub = n.subscribe(n.resolveName("/imu"), 20, imuCallback);

    garmin_sub = n.subscribe("/fix", 20, gpsCallback);

    //create publishers
    target_pub = n.advertise < mst_position::target_heading > ("/target", 5);

    odom_pub = n.advertise < nav_msgs::Odometry > ("vo", 5);

    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 5);

    reset_waypoints();

    //set rate to 30 hz
    ros::Rate loop_rate(30);

    //run main loop
    while (ros::ok()) {
        //check calbacks
        ros::spinOnce();

        //want heding to update with last so were going to greenwich
        if (gps_fix || true) {
            //send the target to navigation
            if (inital_gps) {
                target_waypoint = find_target();
                odom_set_origin();
                inital_gps = false;
            }

            if (target_waypoint != -1) {
                target_heading = compute_msg(target_waypoint);
            }

            if (target_heading.distance <= params.waypoint_radius
                    || skip_waypoint
                    || (way[target_waypoint].limit == 0
                            && target_heading.distance
                                    <= params.dummy_point_radius)) {
                way[target_waypoint].complete = true;
                target_waypoint = find_target();
                if (target_waypoint == -1) {
                    //robot is done
                    target_heading.done = true;
                    if (params.continue_when_done) {
                        target_heading.target_heading = pi / 2;
                        target_heading.distance = 2200000;

                    }
                    target_heading.stop_robot = false;
                } else {
                    //go to next
                    target_heading = compute_msg(target_waypoint);
                }
                skip_waypoint = false;
            }

            //just go straight
            if (!params.go_to_waypoints) {
                target_heading.target_heading = pi / 2;
                target_heading.distance = 2200000;
            }

            if (way[target_waypoint].limit == 0) {
                target_heading.target_heading = pi / 2;
            }

            target_pub.publish(target_heading);

            odom_pub.publish(odom_msg());

            goal_pub.publish(relative_waypoint(target_waypoint));
        }

        loop_rate.sleep();
    }

    return 0;
}

