#include "cone.h"
#include "path_point.h"

Cone::Cone(float X, float Y, char col, int ID)
	: position(PathPoint(X, Y)), colour(col), id(ID) {}