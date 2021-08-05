#ifndef SRC_PATH_FOLLOWER_H
#define SRC_PATH_FOLLOWER_H

#include <ros/ros.h>
#include <vector>
#include <cstdint>
#include <memory>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <utility>
#include <string>
#include <tf/tf.h>                  // For Convertion from Quartenion to Euler

#include "spline.h"
#include "path_point.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Accel.h> //
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include "mur_common/path_msg.h"
#include "mur_common/actuation_msg.h"

//#include "cone.h"
//#include "path_point.h"

#define PI 3.14159265359
#define LENGTH 2.95 //length of vehicle (front to rear wheel)  /////
#define G  9.81 //gravity
#define MAX_ACC 11.772 //1.2*G
#define MAX_DECEL -17.658 //-1.8*Gg
#define MAX_STEER 0.8 //
#define STEPSIZE 0.2 //spline step size   /////
#define DT 0.05
//PID gains:
#define KP 1   /////
#define KI 1   /////
#define KD  1   /////
//pure pursuit gains
#define K 0.1
#define LFV  0.1  ///// look forward gain
#define LFC  3    ///// look ahead distance
#define V_CONST 1.0 //constant velocity 1m/s (for now)
#define SPLINE_N 10
#define STOP_INDEX 2

#define ODOM_TOPIC "/mur/slam/Odom"
#define PATH_TOPIC "/mur/planner/path"
#define CONTROL_TOPIC "/mur/control/actuation"
#define ACCEL_TOPIC "/mur/accel_desired"
#define STEER_TOPIC "/mur/control_desired"
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"

const bool DEBUG = true;

class PathFollower
{
public:
    PathFollower(ros::NodeHandle);
    void spin();

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_path;
    ros::Publisher pub_control;
    ros::Publisher pub_accel;
    ros::Publisher pub_steer;
    ros::Publisher pub_target;
    ros::Publisher pub_path_viz;

    float car_x;
    float car_y;
    float car_v;
    float car_yaw;
    bool odom_msg_received = false;
    bool new_centre_points = false;
    float rearX;
    float rearY;
    float Lf = LFC;
    PathPoint currentGoalPoint;
    bool slowDown = false;
    bool stopCar = false;
    bool endOfPath = false;
    bool endOfLap = false;
    bool stopSpline = false;
    bool slowLapFinish = false;

    std::vector<float> path_x;
    std::vector<float> path_y;
    bool path_msg_received = false;

    std::vector<PathPoint> centre_points; //centre line points of race tack
    std::vector<PathPoint> centre_splined;
    std::vector<PathPoint> centre_endOfLap;
    int index = 1; //centre splined index
    int index_endOfLap = 5;
    std::vector<float> xp;
    std::vector<float> yp;
    std::vector<float> T;

    float steering = 0;
    float acceleration = 0;
    PathPoint p;

    void waitForMsgs();
    int launchSubscribers();
    int launchPublishers();
    void odomCallback(const nav_msgs::Odometry &msg);
    void pathCallback(const mur_common::path_msg &msg);
    void pushcontrol();
    void generateSplines();
    void accelerationControl();
    void steeringControl();
    void updateRearPos();
    float getDistFromCar(PathPoint&);
    void clearVars();
    float getGoalPoint();

    void pushDesiredCtrl();
    void pushDesiredAccel();
    void pushTarget();
    float Quart2EulerYaw(float, float, float, float);
    float getSign(float&);
    void pushPathViz();
};



#endif
