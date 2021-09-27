#ifndef HUSKY_PATH_FOLLOWER_H
#define HUSKY_PATH_FOLLOWER_H

#include <ros/ros.h>                // Must include for all ROS C++
#include <nav_msgs/Odometry.h>      // Msg from /odometry/filtered
#include <tf/tf.h>                  // For Convertion from Quartenion to Euler
#include "mur_common/path_msg.h"    // path msg from mur_common
#include "mur_common/cone_msg.h"    // cone messages 
#include <visualization_msgs/Marker.h> // for RVIZ markers
#include <visualization_msgs/MarkerArray.h> //fro RVIZ markers
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include "cone.h"
#include "path_point.h"

// ROS topics
#define ODOM_TOPIC "/mur/slam/Odom"//etry/filtered"                     //"/mur/slam/Odom" in murSim  
#define TRUE_CONES "/mur/slam/true_cones"
#define CONE_TOPIC "/mur/slam/cones"
#define RVIZ_CONES "cone_markers_sim"
#define FRAME "map" //"map"

#define SENSOR_RANGE 12
#define CERTAIN_RANGE 5.5
#define HZ 10

const bool DEBUG = false;              //to show debug messages in terminal, switch to false to turn off
bool EUFS = false; //switch to true if using the eufs small track
class ConesPublisher
{
public:
    ConesPublisher(ros::NodeHandle);
    void spin();
    bool last_cone = false;

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_cones;
    ros::Publisher pub_cones;
    ros::Publisher markers_pub;

    PathPoint car_pose;
    double car_yaw;                      // car yaw in Euler angle
    
    bool odom_msg_received = false;
    bool trueCones_msg_received = false;
    bool new_centre_points = false;

    std::vector<Cone> true_cones;  
    std::vector<Cone> seen_cones;
    std::vector<int> cones_list;       
    int index = 0;
    
    
    // *** functions *** //
    //standard ROS functions:
    void waitForMsgs();
    void clearTemps();
    int launchSubscribers();
    int launchPublishers();
    void odomCallback(const nav_msgs::Odometry &msg);
    void trueConesCallback(const mur_common::cone_msg &msg);
    void publishCones();
    void setMarkerProperties(visualization_msgs::Marker *marker,PathPoint pos,
                            int n,std::string colour,std::string frame_id);
    double getAngleFromCar(PathPoint& );
    double getDistFromCar(PathPoint&);
    void detectCones();
    void makeUncertain();            
};



#endif