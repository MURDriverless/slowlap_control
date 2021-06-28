#ifndef SRC_CONE_H
#define SRC_CONE_H

#include "path_point.h"

struct Cone 
{
    Cone(float, float, char);
    PathPoint position;
    char colour;			// Colour of Cone 
    float x_pos_avg;			// Average X position
    float y_pos_avg;			// Average Y position
    float track_width = 0.228;		// Default width of track (meters)
    unsigned char times_seen = 1;	// Times seen and scanned by SLAM
    unsigned int times_checked = 0;
    bool mapped = false;		// Mapped geometrically by PathPlanner
    bool seen = false;			// Seen and recorded at least once by SLAM
    float dist;				// Distance to car
};

#endif // SRC_CONE_H
