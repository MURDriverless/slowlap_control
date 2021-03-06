#ifndef SRC_PATH_POINT_H
#define SRC_PATH_POINT_H

struct PathPoint
{
    PathPoint();
    PathPoint(float, float); 
    float x;			// x coordinate on map
    float y;			// y coordinate on map
    float radius = 0;
    float velocity = 0; 
    float angle = 0;
    bool passedBy = false;
    void updatePoint(float, float);
    void updatePoint(PathPoint);
};

PathPoint::PathPoint() {}
PathPoint::PathPoint(float X, float Y)
	: x(X), y(Y) {}
void PathPoint::updatePoint(float xx,float yy)
{
    this->x = xx;
    this->y = yy;
}
void PathPoint::updatePoint(PathPoint p)
{
    this->x = p.x;
    this->y = p.y;
}



#endif // SRC_PATH_POINT_H
