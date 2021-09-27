#ifndef SRC_CONE_H
#define SRC_CONE_H

#include "path_point.h"


struct Cone 
{
    Cone(float, float, char, int);
    PathPoint position;
    PathPoint uncertainPos;
    char colour;			        // Colour of Cone 
    float x_pos_avg;			    // Average X position
    float y_pos_avg;			    // Average Y position
    float track_width = 0.228;		// Default width of track (meters)
    unsigned char times_seen = 1;	// Times seen and scanned by SLAM
    unsigned int times_checked = 0;
    int mapped = 0;		            // number of times mapped
    bool passedBy = false;			// if passed by car
    float dist;				        // Distance to car
    int id;                         // ID number
    float cost;                     // total cost using cost function
};



Cone::Cone(float X, float Y, char col, int ID)
	: position(PathPoint(X, Y)), colour(col), id(ID) {}

#endif // SRC_CONE_H
