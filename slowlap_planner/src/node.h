/**
 * This is the planner node header file
 * see comments for description of each member
*/

#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <ros/ros.h>                        // ROS
#include <nav_msgs/Odometry.h>              // odometry messages
#include <nav_msgs/Path.h>                  // path messages for rviz
#include <geometry_msgs/PoseStamped.h>      // path messages for rviz
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>      // rviz msgs
#include <visualization_msgs/MarkerArray.h> // rviz msgs
#include "mur_common/cone_msg.h"            // cone messages from slam
#include "mur_common/path_msg.h"            // path message from planner
#include "mur_common/diagnostic_msg.h"      // diagnostic mgs (MURauto20)
#include "mur_common/transition_msg.h"      // msg to transition to fast lap
#include "mur_common/map_msg.h"             // msg for complete map
#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "path_planner.h"
#include <iostream>
#include <vector>
#include <assert.h>
#include <limits>
#include <numeric>
#include "cone.h"                           // cone class
#include "path_point.h"                     // path point class

// ROS topics:
#define HUSKY_ODOM_TOPIC "/odometry/filtered"
#define MUR_ODOM_TOPIC "/mur/slam/Odom"
#define CONE_TOPIC "/mur/slam/cones"
#define PATH_TOPIC "/mur/planner/path"
#define PATH_CONES_TOPIC "/mur/planner/path_cones"
#define PATH_VIZ_TOPIC "/mur/planner/path_viz"
#define HEALTH_TOPIC "/mur/planner/topic_health"
#define SORTED_LCONES_TOPIC "/mur/planner/left_sorted_cones"
#define SORTED_RCONES_TOPIC "/mur/planner/right_sorted_cones"
#define FASTLAP_READY_TOPIC "/mur/control/transition"
#define FINISHED_MAP_TOPIC "/mur/planner/map"

#define HZ 12   // publish frequency
#define FRAME "map"

typedef std::chrono::high_resolution_clock Clock;               // (MURauto20)
typedef std::chrono::high_resolution_clock::time_point ClockTP; // (MURauto20)

class PlannerNode
{
public:
    PlannerNode(ros::NodeHandle, bool, float, float, float);
    void spinThread();
    std::unique_ptr<PathPlanner> planner;
    void shut_down();
    bool fastLapReady = false;

private:

    bool const_velocity;
    float v_max;
    float v_const;
    float max_f_gain;

    //see .cpp for function description
    void initialisePlanner();
    int launchSubscribers();
    int launchPublishers();
    void clearTempVectors();
    void pushPathViz();
    void pushPath();
    void pushHealth(ClockTP&, ClockTP&, ClockTP&, ClockTP&);
    void pushSortedCones();
    void pushSortingMarkers();
    void waitForMsgs();
    void odomCallback(const nav_msgs::Odometry&);
    void coneCallback(const mur_common::cone_msg&);
    void transitionCallback(const mur_common::transition_msg&);
    void pushMarkers();
    void callback(const nav_msgs::Odometry&, const mur_common::cone_msg&);
    void setMarkerProperties(visualization_msgs::Marker *marker,PathPoint cone1,
                                        PathPoint cone2,int n,bool accepted);
    void setMarkerProperties2(visualization_msgs::Marker *marker,PathPoint cone,int id,int n,char c);
    void SlowLapFinished();
    

    
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
    ros::Publisher pub_pathCones;
    ros::Publisher pub_map;
    ros::Subscriber sub_transition;
    ros::Publisher pub_sorting_markers;
    ros::Publisher pub_path_marks;

    ros::Time now;                  // diagnostic stuff (MURauto20)
    std::vector<uint32_t> times;    // diagnostic stuff (MURauto20)
    std::vector<uint32_t> rtimes;   // diagnostic stuff (MURauto20)
    uint64_t compute_time;          // diagnostic stuff (MURauto20)

    bool slowLapDone = false;           // flag when slo lap is done
    bool cone_msg_received = false;     // flag when cone msgs are received by subscriber
    bool odom_msg_received = false;     // flag when odom msgs are received by subscriber
    bool plannerInitialised = false;    // flag when planner is initialised
    bool plannerComplete = false;       // flag when planner is done 
            
    std::vector<PathPoint> Path;        // centre line points 
    std::vector<Cone> Left;             // sorted left cones (blue)
    std::vector<Cone> Right;            // sorted right cones (yellow)
    std::vector<Cone> cones;            // raw cones
    std::vector<PathPoint> Markers;     // rviz
    std::vector<PathPoint> sortMarks;   // rviz
    
    PathPoint startFin;                 // start/finishline midpoint
    float car_x;                        // car current pos x
    float car_y;                        // car current pos y
    float car_v;                        // car current v
    float yaw;                          // car current yaw
    float z;                            // quaternion conversion
    float w;                            // quaternion stuff
   
};

#endif // SRC_NODE_H
