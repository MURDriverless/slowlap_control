#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <ros/ros.h>                        // ROS
#include <nav_msgs/Odometry.h>              // odometry messages
#include <nav_msgs/Path.h>                  // path messages for rviz
#include <geometry_msgs/PoseStamped.h>      // path messages for rviz
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "mur_common/cone_msg.h"            // cone messages from slam
#include "mur_common/path_msg.h"            // path message from planner
#include "mur_common/diagnostic_msg.h"  
#include "cone.h"                           // cone class
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include "path_point.h"
#include "path_planner.h"
#include <iostream>
#include <vector>
#include <assert.h>
#include <limits>
#include <numeric>

#define ODOM_TOPIC "/mur/slam/Odom"
#define CONE_TOPIC "/mur/slam/cones"
#define PATH_TOPIC "/mur/planner/path"
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"
#define HEALTH_TOPIC "/mur/planner/topic_health"
#define SORTED_LCONES_TOPIC "/mur/planner/left_sorted_cones"
#define SORTED_RCONES_TOPIC "/mur/planner/right_sorted_cones"



typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::high_resolution_clock::time_point ClockTP;

class PlannerNode
{
public:
    PlannerNode(ros::NodeHandle, bool, float, float, float);
    void spinThread();
    std::unique_ptr<PathPlanner> planner;

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

    
    // ROS standard variables:
    ros::NodeHandle nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_cones;  
    ros::Publisher pub_path;
    ros::Publisher pub_path_viz;
    ros::Publisher pub_health;
    ros::Publisher pub_control;
    ros::Publisher pub_lcones;
    ros::Publisher pub_rcones;

    ros::Time now;                  // diagnostic stuff
    std::vector<uint32_t> times;    // diagnostic stuff
    std::vector<uint32_t> rtimes;   // diagnostic stuff
    bool slowLapDone = false;

    bool cone_msg_received = false;
    bool odom_msg_received = false;
    uint64_t compute_time;

    

    std::vector<float> X;               // path_x
    std::vector<float> Y;               // path_y
    std::vector<float> V;               // velocity (not used, we are using constant v)
    std::vector<Cone> Left;             // sorted left cones (blue)
    std::vector<Cone> Right;            // sorted right cones (yellow)
    std::vector<Cone> cones;            // raw cones
    
    float car_x;                        // car current pos x
    float car_y;                        // car current pos y
    float car_v;                        // car current v
    float yaw;                          // car current yaw
    float z;                            // quaternion conversion
    float w;                            // quaternion stuff
   
};

#endif // SRC_NODE_H
