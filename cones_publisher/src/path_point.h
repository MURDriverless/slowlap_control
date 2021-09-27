#ifndef SRC_PATH_POINT_H
#define SRC_PATH_POINT_H

#include "cone.h"

struct Cone; //incomplete type for cyclic dependency problem

struct PathPoint
{
    PathPoint();
    PathPoint(float, float); 
    float x;			//  x position on map
    float y;			//  y position on map
    float z = 0.01;
    float radius = 0;
    float velocity = 0; 
    float angle = 0;
    Cone* cone1 = NULL;        //to determine from which cone the point was formed
    Cone* cone2 = NULL;
};

PathPoint::PathPoint() {}
PathPoint::PathPoint(float X, float Y)
	: x(X), y(Y) {}



#endif // SRC_PATH_POINT_H
