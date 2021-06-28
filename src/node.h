#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point.h>
#include "mur_common/cone_msg.h"
#include "mur_common/path_msg.h"
#include "mur_common/diagnostic_msg.h"
#include "mur_common/actuation_msg.h"
#include "cone.h"
#include "path_point.h"
#include "path_follower.h"
#include <string>
#include <vector>
#include <memory>
#include <chrono>

#define ODOM_TOPIC "/mur/slam/Odom"
#define CONE_TOPIC "/mur/slam/cones"
#define PATH_TOPIC "/mur/planner/path"
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"
#define HEALTH_TOPIC "/mur/planner/topic_health"
#define SORTED_CONES_LEFT_TOPIC "/mur/planner/left_sorted_cones"
#define SORTED_CONES_RIGHT_TOPIC "/mur/planner/right_sorted_cones"
#define ActuationData "/mur/control/actuation"
#define DT 0.05

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
    void pushPath(); //not a publisher
    void pushControl();
    void pushDesiredCtrl();
    void pushDesiredAccel();
    void pushHealth(ClockTP&, ClockTP&, ClockTP&, ClockTP&);
    void pushSortedCones() const;
    void waitForMsgs();
    void odomCallback(const nav_msgs::Odometry&);
    void coneCallback(const mur_common::cone_msg&);
    void callback(const nav_msgs::Odometry&, const mur_common::cone_msg&);
    void syncMsgs();
    float Sgn(float x);
    bool cone_msg_received = false;
    bool odom_msg_received = false;
    uint64_t compute_time;

    std::unique_ptr<PathPlanner> planner;
    PathFollower follower;
    PathPoint path_point;

    std::vector<float> X; //path x
    std::vector<float> Y; //path y
    std::vector<float> V; //velocity
    std::vector<float> cone_lx;
    std::vector<float> cone_ly;
    std::vector<char> cone_lcolour;
    std::vector<float> cone_rx;
    std::vector<float> cone_ry;
    std::vector<char> cone_rcolour;
    
    
    std::vector<Cone> cones;
    float car_x;
    float car_y;
    float car_v;
    float yaw;

    ros::NodeHandle nh;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_cones;
    
    //ros::Publisher pub_path;
    ros::Publisher pub_control;
    ros::Publisher pub_path_viz;
    ros::Publisher pub_health;
    ros::Publisher pub_lcones;
    ros::Publisher pub_rcones;

    std::vector<uint32_t> times;
    std::vector<uint32_t> rtimes;

    ros::Time now;

    void printVectors();
};

#endif // SRC_NODE_H
