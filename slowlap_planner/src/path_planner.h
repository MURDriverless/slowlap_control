/**
 * This is the Path planner header file
 * see comments for description of each member
*/

#ifndef SRC_PATH_PLANNER_H
#define SRC_PATH_PLANNER_H


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

#define TRACKWIDTH 4
#define MAX_PATH_ANGLE1 50      // angle constraint for the path point formed
#define MAX_PATH_ANGLE2 310     // angle constraint for the path point formed
#define MAX_POINT_DIST 6       // distance constraint for path point formed
#define MIN_POINT_DIST 0.5      // distance constraint for path point formed
#define CERTAIN_RANGE 4         // if cone is within this range, cone positions are certain and no longer updated


const bool DEBUG = true;        //  to show debug messages, switch to false to turn off

class PathPlanner 
{
public:
    PathPlanner(float, float, std::vector<Cone>&, bool, float, float, float, std::vector<PathPoint>&);
    void update(std::vector<Cone>&, const float, const float, std::vector<PathPoint>&,
    std::vector<Cone> &,std::vector<Cone> &,std::vector<PathPoint>&, bool&);
    bool complete = false;

private:
    std::vector<PathPoint> centre_points;                   // vector of path points
    std::vector<PathPoint> cenPoints_temp1,cenPoints_temp2 ;// temporary vector
    std::vector<PathPoint> rejected_points;                 // rejected path points, for visualisation purposes
    std::vector<Cone> raw_cones;                            // copy of cones passed by SLAM
    std::vector<Cone*> future_cones;                        // pointer to cones to be sorted
    std::vector<Cone*> left_cones;		    // Cones on left-side of track (sorted)
    std::vector<Cone*> right_cones;		    // Cones on right-side of track (sorted)
    std::vector<Cone*> timing_cones;        // pointer to Orange cones		
    std::vector<Cone*> left_unsorted;       // pointer to left cones (unsorted)
    std::vector<Cone*> right_unsorted;      // pointer to right cones (unsorted)
    std::vector<Cone*> thisSide_cone;       // (temporary var) pointer to cones on one side, used for cone sorting and generating path points
    std::vector<Cone*> oppSide_cone;        // (temporary var) pointer to opposite side cones, used for cone sorting and generating path points
    std::vector<Cone*> oppSide_cone2;       // (temporary var) poiner to opposite cone, used for cone sorting and generating path points
    
    PathPoint car_pos;              // current car position
    PathPoint init_pos;             // initial position of car
    PathPoint startFinish;          // mid point of star/finish line(orange cones)
    bool timingCalc = false;        // flag when orange cones have given a path point
    bool needToSort = false;
    bool newConesToSort = false;    // flag when there are new cones to be sorted
    bool newConesSorted = false;    // flag when cones are sorted
    bool timingEmpty = true;        // flag when no range cones have been seen
    bool l_cones_sorted = false;    // flag when left cones are sorted
    bool r_cones_sorted = false;    // flag when right cones are sorted
    bool left_start_zone = false;   // flag when car is x metres away from orange cones
    bool reached_end_zone = false;  // flag wheh near the end, slow lap almost finished
    bool gotNewCones = false;       // flag when there are new cones
    bool passedByAll = false;       // flag when all cones have been passed by
    int leftIndx = 0;               // index of left cones vector
    int rightIndx = 0;              // index of right cones vector            
    int passedByIndex = 0;          // index used for raw_cones
    int passedByPntIndx = 2;        // index used for centre_points, does not need to start at 0
    int rejectCount = 0;            // visulisation of rejected points
    
    bool const_velocity;
    bool first_run = true;
    float v_max;
    float v_const;
    float f_gain;

    // see .cpp file for function descriptions
    Cone* findOppositeClosest(const Cone&, const std::vector<Cone*>&);
    void addFirstCentrePoints();
    void addCentrePoints();
    void sortConesByDist(const PathPoint&);
    static bool compareConeDist(Cone* const&, Cone* const&);
    void popConesToAdd();
    void calcSpline();
    void addCones(std::vector<Cone>&);
    void addVelocityPoints();
    float calcRadius(const PathPoint&, const PathPoint&, const PathPoint&);
    float calcDist(const PathPoint&, const PathPoint&);
    void removeFirstPtr(std::vector<Cone*>&);
    void resetTempConeVectors();
    void returnResult(std::vector<PathPoint>&,std::vector<Cone>&,
                                                    std::vector<Cone>&,std::vector<PathPoint>&);
    void centralizeTimingCones();
    static float calcAngle(const PathPoint&, const PathPoint&, const PathPoint&);
    static float calcRelativeAngle(const PathPoint&, const PathPoint&);
    bool joinFeasible(const float&, const float&);
    PathPoint generateCentrePoint(Cone*, Cone*, bool&, std::vector<PathPoint>&);
    void updateStoredCones(std::vector<Cone>&);
    void updateCentrePoints();
    float computeCost1(Cone* &cn1, Cone* &cn2);
    float computeCost2a(Cone* &cn1, std::vector<Cone*> &oppCone1,std::vector<Cone*> &oppCone2);
    float computeCost2b(Cone* &cn1, std::vector<Cone*> &oppCone);
    float computeCost3(Cone* &cn1, std::vector<Cone*> &cn2);
    static bool compareConeCost(Cone* const&, Cone* const&);
    static bool comparePointDist(PathPoint& pt1, PathPoint& pt2);
    void sortAndPushCone(std::vector<Cone*> &cn);
    void sortPathPoints(std::vector<PathPoint>&,PathPoint&);

};

#endif // SRC_PATH_PLANNER_H
