#ifndef SRC_PATH_POINT_H
#define SRC_PATH_POINT_H

struct PathPoint
{
    PathPoint();
    PathPoint(float, float); 
    float x;			// Corresponding to x position on map
    float y;			// Corresponding to y position on map
    float radius = 0;
    float velocity = 0; 
    float angle = 0;
};

PathPoint::PathPoint() {}
PathPoint::PathPoint(float X, float Y)
	: x(X), y(Y) {}



#endif // SRC_PATH_POINT_H
