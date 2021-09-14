/**
 * This is the Cone class
 * all the necessary information about each cone is here
 * this is usually used in a vector, std::vector<Cone>
 * 
 **/

 /**
  * To do:
  * check the not used members and see if OK to erase
  * */


#ifndef SRC_CONE_H
#define SRC_CONE_H

#include "path_point.h"


struct Cone 
{
    Cone(float, float, char, int);  // constructor
    PathPoint position;             // cone pos
    char colour;			        // Colour of Cone 
    float x_pos_avg;			    // ..not used Average X position
    float y_pos_avg;			    // ..not used Average Y position
    float track_width = 0.228;		// ..not used Default width of track (meters)
    unsigned char times_seen = 1;	// ..not used Times seen and scanned by SLAM
    unsigned int times_checked = 0; // ..not used
    int mapped = 0;		            // number of times mapped
    bool passedBy = false;			// if passed by car
    int paired = 0;                 // count how many time paired to opp cone
    float dist;				        // Distance to car
    int id;                         // ID number
    float cost;                     // total cost using cost function
    void updateConePos(PathPoint);  // used to update the cone position if new pos is given by SLAM
};



Cone::Cone(float X, float Y, char col, int ID)
	: position(PathPoint(X, Y)), colour(col), id(ID) {}

void Cone::updateConePos(PathPoint newPos)
{
    
    this->position = newPos;
}
#endif // SRC_CONE_H
