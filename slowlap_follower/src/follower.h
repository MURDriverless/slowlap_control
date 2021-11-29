#ifndef HUSKY_PATH_FOLLOWER_H
#define HUSKY_PATH_FOLLOWER_H

#include <ros/ros.h>                    // Must include for all ROS C++
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>  // rviz msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Accel.h>        //
#include <geometry_msgs/Twist.h>        // To control linear and angular velocity of Husky
#include <nav_msgs/Odometry.h>          // Msg from /odometry/filtered
#include <tf/tf.h>                      // For Convertion from Quartenion to Euler
#include "mur_common/path_msg.h"        // path msg from mur_common
#include "mur_common/actuation_msg.h"
#include "mur_common/transition_msg.h"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include "path_point.h"             // path point class/struct
#include <nav_msgs/Path.h>          // for cubic splining of path points
#include "spline.h"

#define LENGTH 2.95                 // length of vehicle (front to rear wheel)
#define G  9.81                     // gravity
#define MAX_ACC 11.772              // 1.2*G, copied from Dennis (MURauto20)
#define MAX_DECEL -17.658           // -1.8*Gg copied from Dennis (MURauto20)
#define MAX_STEER 0.5//0.8          // Copied from Dennis  (MURauto20)
#define STEPSIZE 0.1                // spline step size 
#define SPLINE_N 6                  // number of points to spline
#define DT 0.05
#define STOP_INDEX 2                // centre point where the car should stop
#define DELTA_STEER 0.05            // change in steering angle 

//PID gains:
#define KP 2   
#define KI 1   
#define KD  1  

//pure pursuit gains
#define K 0.1
#define LFV  0.1                // look forward gain
#define LFC  3.5                // look ahead distance 
#define V_CONST 3               // constant velocity 3m/s (for now)
#define MAX_V  3                // for Husky, test only, should be 1m/s to match mur car
#define MAX_W 30                // for Husky, angular velo in degrees
#define HZ 20                   // ROS spin frequency (can increase to 20)

#define FRAME "map"
// ROS topics
#define ODOM_TOPIC "/mur/slam/Odom"
#define PATH_TOPIC "/mur/planner/path"
#define CONTROL_TOPIC "/mur/control/actuation"
#define ACCEL_TOPIC "/mur/accel_desired"
#define STEER_TOPIC "/mur/control_desired"
#define PATH_VIZ_TOPIC "/mur/follower/path_viz"
#define GOALPT_VIZ_TOPIC "/mur/follower/goalpt_viz"
#define FASTLAP_READY_TOPIC "/mur/control/transition"

bool DEBUG = true;              //to show debug messages in terminal, switch to false to turn off

class PathFollower
{
public:
    PathFollower(ros::NodeHandle n, double max_v, double max_w);
    void spin();
    bool slowLapFinish = false;
    bool fastLapReady = false;

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_path;
    ros::Subscriber sub_transition;
    ros::Publisher pub_control;
    ros::Publisher pub_accel;
    ros::Publisher pub_steer;
    ros::Publisher pub_target;
    ros::Publisher pub_path_viz;
    ros::Publisher pub_goalPt;

    double max_v;
    double max_w;
    double KP_dist;
    double KP_angle;
    double Lf = LFC;                    // look ahead distance, can be adjusted, see code

    double car_x;                       // car pose x
    double car_y;                       // car pose y
    double car_v;                       // car linear velocity
    double car_yaw;                     // car yaw in Euler angle
    double car_yaw2;                    // different formula for yaw. works somehow
    double initX = 0;                   // initial pos x
    double initY = 0;                   // initial pos y
    double initYaw = 0;                 // initial yaw
    double rearX, rearY;
    bool initialised = false;

    bool odom_msg_received = false;
    bool new_centre_points = false;
    int cenPoints_updated = 0;

    std::vector<PathPoint> centre_points;       // centre line points of race tack, from path planner
    std::vector<PathPoint> centre_splined;      // splined centre line points, see func generateSpline()
    PathPoint currentGoalPoint = PathPoint(0,0);

    // temp vectors for splining
    std::vector<double> xp;
    std::vector<double> yp;
    std::vector<double> T;

    bool path_msg_received = false;
    bool newGP = false;
    bool endOfPath = false;
    bool endOfLap = false;
    bool stopSpline = false; 
    bool plannerComplete = false;   
    int index = -1;                              // index in centre_splined for goal point
    int oldIndex = -1;                           // index in centre_splined for goal point
    int index_endOfLap = 1/STEPSIZE;             // index in centre_endOfLap for goal point

    // actuation commands, publish to actuator
    double acceleration=0;
    double steering=0;
    
    // *** functions *** //
    //standard ROS functions:
    void waitForMsgs();
    int launchSubscribers();
    int launchPublishers();
    void transitionCallback(const mur_common::transition_msg &msg);
    void odomCallback(const nav_msgs::Odometry &msg);
    void pathCallback(const mur_common::path_msg &msg);
    void publishCtrl();
    void pushPathViz(); 
    void pushDesiredCtrl();
    void pushDesiredAccel();

    void DrivingControl();             // acceleration and steering. see cpp file for description
    void generateSplines();             // see cpp file for description
    void updateRearPos();
    double getDistFromCar(PathPoint&);    // to compute distance of point to current car pose
    double getAngleFromCar(PathPoint&);   // to compute angle differene of a point to current car yaw 
    double calcDist(const PathPoint &p1, const PathPoint &p2);
    void clearVars();                   // clear temporary variables, vectors
    void getGoalPoint();                // see cpp file for description
    double getSign(double&);  
    void shut_down();                   // when slow lap is complete            
};



#endif