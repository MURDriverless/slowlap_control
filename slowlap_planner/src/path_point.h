/**
 * This is the Path point class
 * all the necessary information about a point in the map is here
 * 
*/

/**
 * To do:
 * check not used member and see if ok to delete
*/

#ifndef SRC_PATH_POINT_H
#define SRC_PATH_POINT_H

#include "cone.h"

struct Cone; //incomplete type for cyclic dependency problem

struct PathPoint
{
    PathPoint();                // Constructor
    PathPoint(float, float);    // Constructor
    float x;			        // Corresponding to x position on map
    float y;			        // Corresponding to y position on map
    float z = 0;                //
    float radius = 0;           // not used
    float velocity = 0;         // not yet used
    float angle = 0;            // not used
    float dist;                 // distance to car, used for sorting
    Cone* cone1 = NULL;         // to determine from which cone the point was formed
    Cone* cone2 = NULL;         // to determine from which cone the point was formed
    bool accepted = false;      // to determine if path point is acceptable
};

PathPoint::PathPoint() {}
PathPoint::PathPoint(float X, float Y)
	: x(X), y(Y) {}



#endif // SRC_PATH_POINT_H
