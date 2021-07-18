#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Accel.h> //
#include "mur_common/cone_msg.h"
#include "mur_common/path_msg.h"
#include "mur_common/diagnostic_msg.h"
#include "mur_common/actuation_msg.h" //
#include "cone.h"
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include "path_point.h"

#define ODOM_TOPIC "/mur/slam/Odom"
#define CONE_TOPIC "/mur/slam/cones"
#define PATH_TOPIC "/mur/planner/path"
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"
#define HEALTH_TOPIC "/mur/planner/topic_health"
#define CONTROL_TOPIC "/mur/control/actuation"
#define SORTED_LCONES_TOPIC "/mur/planner/left_sorted_cones"
#define SORTED_RCONES_TOPIC "/mur/planner/right_sorted_cones"
#define DT 0.05
/////
#define PI 3.14159265359
#define LENGTH 2.95 //length of vehicle (fron to rear wheel)  /////
#define G  9.81
#define MAX_ACC 11.772//1.2*G
#define MAX_DECEL -17.658//-1.8*Gg
#define MAX_STEER 0.8 //
#define STEPSIZE 0.2 //spline step size   /////
//PID gains:
#define KP 1   /////
#define KI 1   /////
#define KD  1   /////
//pure pursuit gains
#define LFV  0.1  /////
#define LFC  2    /////


typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::high_resolution_clock::time_point ClockTP;

class PlannerNode
{
public:
    PlannerNode(ros::NodeHandle, bool, float, float, float);
    void spinThread();

private:

    bool const_velocity;
    float v_max;
    float v_const;
    float max_f_gain;

    int launchSubscribers();
    int launchPublishers();
    void clearTempVectors();
    void pushPathViz();
    void pushPath();
    void pushHealth(ClockTP&, ClockTP&, ClockTP&, ClockTP&);
    void pushSortedCones();
    void waitForMsgs();
    void odomCallback(const nav_msgs::Odometry&);
    void coneCallback(const mur_common::cone_msg&);
    void callback(const nav_msgs::Odometry&, const mur_common::cone_msg&);
    void syncMsgs();
    void pushControl();
    void pushDesiredCtrl();
    void pushDesiredAccel();
    bool cone_msg_received = false;
    bool odom_msg_received = false;
    uint64_t compute_time;

    std::unique_ptr<PathPlanner> planner;

    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> V;
    std::vector<Cone> Left;
    std::vector<Cone> Right;
    
    std::vector<Cone> cones;
    float car_x;
    float car_y;
    float car_v;
    float yaw;
    float z; //for quaternion conversion
    float w;

    ros::NodeHandle nh;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_cones;
    
    //ros::Publisher pub_path;
    ros::Publisher pub_path_viz;
    ros::Publisher pub_health;
    ros::Publisher pub_control;
    ros::Publisher pub_lcones;
    ros::Publisher pub_rcones;

    std::vector<uint32_t> times;
    std::vector<uint32_t> rtimes;

    ros::Time now;

    void printVectors();
///////////path follower////////
    float acceleration;
    float steering;

    void AccelerationControl();
    void SteeringControl(float, float);

    float Quart2EulerYaw(float, float, float, float);
   
};

#endif // SRC_NODE_H
