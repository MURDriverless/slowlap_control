#ifndef SRC_PATH_PLANNER_H
#define SRC_PATH_PLANNER_H

#include "path_point.h"
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <utility>
#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include "cone.h"
#include "path_point.h"

#define PI 3.14159265359
#define TRACKWIDTH 5.5
#define MAX_PATH_ANGLE 70
#define MAX_POINT_DIST 10
#define MIN_POINT_DIST 0.5


extern const uint16_t MIN_ANGLE = 90;
extern const uint16_t MAX_ANGLE = 275;
extern const uint8_t CP_DIST = 12;
extern constexpr uint8_t OPP_CP_DIST = 12;

const bool DEBUG = true;  //to show debug messages

class PathPlanner 
{
public:
    PathPlanner(float, float, const std::vector<Cone>&, bool, float, float, float);
    void update(const std::vector<Cone>&, const float, const float, std::vector<float>&, std::vector<float>&, std::vector<float>&,std::vector<Cone> &,std::vector<Cone> &);
    void shutdown();
    bool complete = false;

private:
    std::vector<Cone> raw_cones;
    std::vector<Cone*> left_cones;		// Cones on left-side of track (sorted)
    std::vector<Cone*> right_cones;		// Cones on right-side of track (sorted)
    std::vector<PathPoint> centre_points;
    std::vector<Cone*> timing_cones;		
    std::vector<Cone*> l_cones_to_add;      //new cones, unsorted
    std::vector<Cone*> r_cones_to_add;      //new cones, unsorted
    std::vector<Cone*> oppSide_cone;        //(temp) opposite cone, used for cone sorting and generating path points
    std::vector<Cone*> thisSide_cone;       //(temp) same cone, used for cone sorting and generating path points
    std::vector<PathPoint> velocity_points;
    std::vector<float> radius_points;
    std::vector<int> addedIDLeft;
    std::vector<int> addedIDRight;
    std::vector<int> addedIDRed;
    PathPoint car_pos;
    bool timingCalc = false;
    bool newConesFound = false;
    bool timingEmpty = true;
    bool len_changed;
    bool l_cones_sorted = false;
    bool r_cones_sorted = false;
    bool set_final_points = false;
    bool left_start_zone = false;
    bool reached_end_zone = false;
    
    bool const_velocity;
    bool first_run = true;
    float v_max;
    float v_const;
    float f_gain;
    int findOppositeClosest(const Cone&, const std::vector<Cone*>&);
    void addFirstCentrePoints();
    void addCentrePoints();
    void sortConesByDist(const PathPoint&);
    static bool compareConeDist(Cone* const&, Cone* const&);
    void popConesToAdd();
    void calcSpline();
    void addCones(const std::vector<Cone>&);
    void addVelocityPoints();
    float calcRadius(const PathPoint&, const PathPoint&, const PathPoint&);
    float calcDist(const PathPoint&, const PathPoint&);
    void removeFirstPtr(std::vector<Cone*>&);
    void resetTempConeVectors();
    void returnResult(std::vector<float>&, std::vector<float>&, std::vector<float>&,std::vector<Cone>&, std::vector<Cone>&);
    void centralizeTimingCones();
    static float calcAngle(const PathPoint&, const PathPoint&, const PathPoint&);
    static float calcRelativeAngle(const PathPoint&, const PathPoint&);
    bool joinFeasible(const float&, const float&);
    PathPoint generateCentrePoint(const Cone*, const Cone*, bool&);
    void updateConePos(const std::vector<Cone> &new_cones);
    float computeCost1(Cone* &cn1, Cone* &cn2);
    float computeCost2(Cone* &cn1, Cone* &cn2);
    float computeCost3(Cone* &cn1, std::vector<Cone*> &cn2);
    void sortAndPushCone(std::vector<Cone*> &cn);


};

#endif // SRC_PATH_PLANNER_H
