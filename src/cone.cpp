#include "cone.h"
#include "path_point.h"

Cone::Cone(float X, float Y, char col)
	: position(PathPoint(X, Y)), colour(col) {}
