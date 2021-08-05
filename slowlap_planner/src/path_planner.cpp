#include "path_planner.h"

/**
 * This handles the path planning algorithm
 * After receiving cone information (pos, colour, id) from SLAM
 * - sort cones by colour, and by correct order in race track (not necessarily by distance)
 * - generate path points by taking the mid point between 2 opposite cones 
 * 
 * for future improvement: generate velocity reference as well
 **/

PathPlanner::PathPlanner(float car_x, float car_y, const std::vector<Cone> &cones, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : const_velocity(const_velocity), v_max(v_max), v_const(v_const), f_gain(max_f_gain), car_pos(PathPoint(car_x,car_y))
{
	raw_cones.reserve(300);
	l_cones_to_add.reserve(150);
	r_cones_to_add.reserve(150);
	left_cones.reserve(150);
	right_cones.reserve(150);
	addedIDLeft.reserve(150);
	addedIDRed.reserve(150);
	centre_points.reserve(250);
	thisSide_cone.reserve(150);
	oppSide_cone.reserve(150);

	addCones(cones);								// add new cones to raw cones
    centre_points.emplace_back(car_x,car_y);		// add the car position to  centre points 
	std::cout << "path points size (initial): " << centre_points.size() <<std::endl;
	centralizeTimingCones();						// add orange cones midpoint to centre points

	if (DEBUG) std::cout<< "New cones added. Left: "<<l_cones_to_add.size()<<" Right: "<<r_cones_to_add.size()<<std::endl;
	sortConesByDist(car_pos);
	left_cones.push_back(l_cones_to_add.front());
	right_cones.push_back(r_cones_to_add.front());
	addedIDLeft.push_back(left_cones.back()->id);
	addedIDRight.push_back(right_cones.back()->id);
	resetTempConeVectors();							// Clear pointers and reset l/r_cones_to_add
    addFirstCentrePoints();
	// addCentrePoints();
}

void PathPlanner::update(const std::vector<Cone> &new_cones, const float car_x, const float car_y,
						 std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V,
						 std::vector<Cone> &Left,std::vector<Cone> &Right)
{
	if (complete)
		returnResult(X, Y, V,Left,Right);
	else
	{

		this->car_pos = PathPoint(car_x,car_y);
		if (left_start_zone)
		{
			// join track if feasible
			if (joinFeasible(car_x, car_y))
			{
				std::cout<<"race track almost complete"<<std::endl;
				centre_points.push_back(centre_points.front());
				reached_end_zone = true;
				complete = true;
			}
		}
		else
		{
			if (calcDist(centre_points.front(), car_pos) > 5)
				left_start_zone = true;
		}
		

		if (!reached_end_zone)
		{
			updateConePos(new_cones);
			addCones(new_cones);
			if (!timingCalc) // if failed the first time
			{
				centralizeTimingCones();
			}

			if (newConesFound)
			{
				if (DEBUG) std::cout<< "\nNew cones added. Left: "<<l_cones_to_add.size()<<" Right: "<<r_cones_to_add.size()<<std::endl;
				sortAndPushCone(l_cones_to_add);
				sortAndPushCone(r_cones_to_add);
				resetTempConeVectors();
				addCentrePoints();
			}
		}

		returnResult(X, Y, V, Left, Right);
	}
}

// checks whether track is (almost) finished
bool PathPlanner::joinFeasible(const float &car_x, const float &car_y)
{
	if (calcDist(centre_points.back(), centre_points.front()) < 2) //if less than 2 meters 
	{
		float angle = calcAngle(*(centre_points.end() - 2), centre_points.back(), centre_points.front());
		std::cout << angle << std::endl;

		if (abs(angle) < MIN_ANGLE)
			return true;
	}
	else
		return false;
}

void PathPlanner::returnResult(std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V,
									std::vector<Cone>&Left, std::vector<Cone>&Right)
{
    for (auto &e: centre_points)
	{
		X.push_back(e.x); 
		Y.push_back(e.y); 
		V.push_back(e.velocity);
	}

	for (auto &lc:left_cones)
	{
		Left.push_back(*lc);
	}

	for (auto &rc:right_cones)
	{
		Right.push_back(*rc);
	}
}



void PathPlanner::shutdown()
{
    left_cones.clear();
    right_cones.clear();
    timing_cones.clear();
    l_cones_to_add.clear();
    r_cones_to_add.clear();
}

// used for cone sorting
float PathPlanner::calcAngle(const PathPoint &A, const PathPoint &B, const PathPoint &C)
{
	float cb_x = C.x - B.x;
	float cb_y = C.y - B.y;
	float ca_x = B.x - A.x;
	float ca_y = B.y - A.y;

	float angle = (atan2(cb_y, cb_x) - atan2(ca_y, ca_x)) * (180 / M_PI);

	return angle;
}

float PathPlanner::calcRelativeAngle(const PathPoint &p1, const PathPoint &p2)
{
	const float angle = (atan2(p2.y - p1.y, p2.x - p1.x))* 180 / M_PI;
	return angle;
}



PathPoint PathPlanner::generateCentrePoint(const Cone* cone_one, const Cone* cone_two, bool& feasible)
{
	PathPoint midpoint(
		(cone_one->position.x + cone_two->position.x) / 2,
		(cone_one->position.y + cone_two->position.y) / 2
	);

	float dist_back = calcDist(centre_points.back(), midpoint);
	// float angle = abs(calcAngle(*(centre_points.end() - 2), centre_points.back(), midpoint));
	float angle1 = calcRelativeAngle(centre_points.back(),midpoint);
	float angle2 = calcRelativeAngle(*(centre_points.end()-2),centre_points.back());
	float angle = (calcAngle(*(centre_points.end()-2), centre_points.back(), midpoint));
	float angle12 = angle1- angle2;
	
	if ((abs(angle)<MAX_PATH_ANGLE) && (dist_back>MIN_POINT_DIST) && (dist_back<MAX_POINT_DIST))
	{
		std::cout << "Accepted path point: (" << midpoint.x << ", " << midpoint.y << ")"<< std::endl;
		if (DEBUG) std::cout << "dist and angle: " << dist_back << " " << angle << " a2: "<<angle12<< std::endl;
		feasible = true;
	}
	else
	{
		std::cout << "Rejected point: (" << midpoint.x << ", " << midpoint.y << ")"<< std::endl;
		if (DEBUG) std::cout << "dist and angle: " << dist_back << " " << angle << " a2: "<<angle12<<std::endl;
		feasible = false;
	}

	return midpoint;
}

void PathPlanner::addCentrePoints()
{
	bool feasible;
	PathPoint cp;
	int opp_idx;
	thisSide_cone.clear();
	oppSide_cone.clear();

	//we will look at the side with fewer cones discovered
	if (right_cones.size() < left_cones.size())
	{
		thisSide_cone.assign(right_cones.begin(),right_cones.end());
		oppSide_cone.assign(left_cones.begin(),left_cones.end());
	}
	else if (left_cones.size() <= right_cones.size())
	{
		thisSide_cone.assign(left_cones.begin(),left_cones.end());
		oppSide_cone.assign(right_cones.begin(),right_cones.end());
	}
		
	
	for (int i = 0; i < thisSide_cone.size(); i++)
	{
		if (i+1 == thisSide_cone.size() || thisSide_cone[i+1]->mapped == 0 ) // if latest cone or if next cone is not yet mapped
		{
		if (thisSide_cone[i]->mapped < 3  && thisSide_cone[i]->times_checked < 25) //magic numbers, sorry
		{			
			opp_idx = findOppositeClosest(*thisSide_cone[i], oppSide_cone);
			if (opp_idx != -1)
			{
				thisSide_cone[i]->times_checked++;
				oppSide_cone[opp_idx]->times_checked++;
				feasible = false;
				cp = generateCentrePoint(thisSide_cone[i], oppSide_cone[opp_idx], feasible);
				if (feasible)
				{
					centre_points.push_back(cp);	
					thisSide_cone[i]->mapped++;
					oppSide_cone[opp_idx]->mapped++;
					std::cout << "path points size (add): " << centre_points.size() <<std::endl;
				}
			}	 
		}
		}
	}
}

//add new cones to the local vector of cones and sort by colour
void PathPlanner::addCones(const std::vector<Cone> &new_cones)
{
	bool added = false;

	for (auto &cone: new_cones)
	{
		if (cone.colour == 'b')
		{
			added = false;
			for (auto n: addedIDLeft) 		// check using id if cone has been added before
			{
				if (n == cone.id)
				{
					added = true;
					break;
				}
			}
			if (!added)
			{
				raw_cones.push_back(cone);
				l_cones_to_add.push_back(&raw_cones.back());
				l_cones_sorted = false;
				newConesFound = true;
			}
		}

		else if (cone.colour == 'y')
		{
			added = false;
			for (auto m: addedIDRight) 	// check using id if cone has been added before
			{
				if (m == cone.id)
				{
					added = true;
					break;
				}
			}
			if (!added)
			{
				raw_cones.push_back(cone);
				r_cones_to_add.push_back(&raw_cones.back());
				r_cones_sorted = false;
				newConesFound = true;
			}
		}
		else
		{
			added = false;
			for (auto o: addedIDRed)
			{
				if (o==cone.id)
				{
					added = true;
					break;
				}
			}
			if (!added)
			{
				raw_cones.push_back(cone);
				timing_cones.push_back(&raw_cones.back());
				addedIDRed.push_back(timing_cones.back()->id);
				if (DEBUG) std::cout<<"Timing cones found: "<< timing_cones.size() <<std::endl;
			}		
		}
	}
		
}

// update the position of the raw cones using new cone pos
void PathPlanner::updateConePos(const std::vector<Cone> &new_cones)
{
	
	int i;
	for (auto &con:raw_cones)
	{
		i = con.id;
		con.position.x = new_cones[i].position.x;
		con.position.y = new_cones[i].position.y;
	}
	// future change: instead of replacing with new pos, get average posS
}
     

void PathPlanner::addFirstCentrePoints()
{
    size_t n = std::min(left_cones.size(), right_cones.size());		
    int closest_opp_idx;
    float centre_x, centre_y;

    for (size_t i = 0; i < n; i++)
    {
	    closest_opp_idx = findOppositeClosest(*left_cones[i], right_cones);
		if (closest_opp_idx != -1)
		{
		    centre_x = (right_cones[closest_opp_idx]->position.x + left_cones[i]->position.x) / 2;
		    centre_y = (right_cones[closest_opp_idx]->position.y + left_cones[i]->position.y) / 2;
		    centre_points.push_back(PathPoint(centre_x, centre_y));
		    left_cones[i]->mapped++;
		    right_cones[closest_opp_idx]->mapped++;
			std::cout << "path points size (firstCP): " << centre_points.size() <<std::endl;
	    }
    }
}

// get midpoint of orange cones
void PathPlanner::centralizeTimingCones()
{
	/*taking the average ditance of the timing cones is same as getting their midpoint */
	
	PathPoint avg_point(0, 0);
	
	for (int i = 0; i < timing_cones.size(); i++)
	{
		avg_point.x += timing_cones[i]->position.x; // summation of x positions
		avg_point.y += timing_cones[i]->position.y; // summation of y positions
	} 
	avg_point.x = avg_point.x / timing_cones.size(); // avg x dist
	avg_point.y = avg_point.y / timing_cones.size(); // avg y dist

	// Calc distance to timing cone
	PathPoint coneTemp(timing_cones.front()->position.x,timing_cones.front()->position.y);
	float dist = calcDist(coneTemp, avg_point);
	float angle = calcRelativeAngle(centre_points.back(), avg_point);

	if (dist > 0.4*TRACKWIDTH && dist < TRACKWIDTH && abs(angle)<MAX_PATH_ANGLE)
	{
		centre_points.push_back(avg_point);
		if (DEBUG) std::cout << "Average timing cones position calculated" <<std::endl;
		std::cout << "Accepted point: (" << avg_point.x << ", " << avg_point.y<<")"<<std::endl;
		timingCalc = true;
		std::cout << "path points size (timing): " << centre_points.size() <<std::endl;
	}
	else
	{
		if (DEBUG) std::cout << "[XX] Average timing cones position NOT calculated" <<std::endl;
		std::cout << "Rejected point: (" << avg_point.x << ", " << avg_point.y<<")"<<std::endl;
		timingCalc = false;
	}

		
	
}

// find the closest cone on the opposite side
int PathPlanner::findOppositeClosest(const Cone &cone, const std::vector<Cone*> &cones)
{
	uint8_t min_dist = OPP_CP_DIST;
	uint8_t dist;
	int index = cones.size()-1;
	
	for (int i = cones.size()-1; i>=0; i--)
	{
		dist = calcDist(cone.position, cones[i]->position);
		 if (dist < min_dist)
		 {
			 min_dist = dist;
			 index = i;
		 }
	}


	return index;
}



void PathPlanner::removeFirstPtr(std::vector<Cone*>& cone_vec)
{
    if (cone_vec.size() > 0 && cone_vec.front() != NULL)
    {
		cone_vec.erase(cone_vec.begin());
    }
}

void PathPlanner::sortConesByDist(const PathPoint &pos)
{
	// Assign distance Cone objects on left
    for (auto &cone: l_cones_to_add)
    {cone->dist = calcDist(pos, cone->position);}

	// Assign distance to Cone objects on right
    for (auto &cone: r_cones_to_add) 
    {cone->dist = calcDist(pos, cone->position);}

	// nlogn sort both cones_to_add vectors
    sort(l_cones_to_add.begin(), l_cones_to_add.end(), compareConeDist);
    sort(r_cones_to_add.begin(), r_cones_to_add.end(), compareConeDist);

    l_cones_sorted = true;
    r_cones_sorted = true;
}
bool PathPlanner::compareConeDist(Cone* const &cone_one, Cone* const &cone_two)
{
    return cone_one->dist < cone_two->dist;
}

/* Calculate the distance between 2 points */
float PathPlanner::calcDist(const PathPoint &p1, const PathPoint &p2)
{
    float x_dist = pow(p2.x - p1.x, 2);
    float y_dist = pow(p2.y - p1.y, 2);

    return sqrt(x_dist + y_dist);
}

void PathPlanner::resetTempConeVectors()
{
	l_cones_to_add.clear();
	r_cones_to_add.clear();
	oppSide_cone.clear();
	thisSide_cone.clear();
	l_cones_sorted = false;
	r_cones_sorted = false;
	newConesFound = false;
}



// cost 1: distance between cones of same colour
float PathPlanner::computeCost1(Cone* &cn1, Cone* &cn2)
{
	return calcDist(cn1->position,cn2->position);
}

// cost 2: distance between cone from opposite side
float PathPlanner::computeCost2(Cone* &cn1, Cone* &cn2)
{
	return calcDist(cn1->position,cn2->position);
}

// cost 3: change in track curvature cn2 is sorted cone
float PathPlanner::computeCost3(Cone* &cn1, std::vector<Cone*> &cn2)
{
	int i = cn2.size()-1;
	float theta1 =  atan2((cn2[i]->position.y - cn2[i-1]->position.y),(cn2[i]->position.x - cn2[i-1]->position.x));
	float theta2 =  atan2((cn2[i]->position.y - cn1->position.y),(cn2[i]->position.x - cn1->position.x));
	return theta1 - theta2;
}

//lr-cones_to_add are the newly seen cones
//leftright cones are the sorted
//from the beginning
void PathPlanner::sortAndPushCone(std::vector<Cone*> &cn)
{
	//if empty, return
	if (cn.size() == 0)
		return;


	float least_cost = 99999.22; //random large number
	float cost, cost1, cost2, cost3;
	int index = 0;
	char colour;
	thisSide_cone.clear();
	oppSide_cone.clear();

	if (cn.front()->colour == 'b') //if cone is blue(left)
	{
		colour = 'b';
		thisSide_cone.assign(left_cones.begin(),left_cones.end());
		oppSide_cone.assign(right_cones.begin(),right_cones.end());
	}
	else if (cn.front()->colour == 'y') //if cone is yellow(right)
	{
		colour = 'y';
		thisSide_cone.assign(right_cones.begin(),right_cones.end());
		oppSide_cone.assign(left_cones.begin(),left_cones.end());
	}

	if (cn.size()<2) //if only 1 cone is seen, compute cost 2 (dist to opposite side) and compare to track width
	{
		float indx = findOppositeClosest(*(cn.back()),oppSide_cone);
		float cost2 = computeCost2(cn.back(),oppSide_cone[indx]);
		std::cout<<"cone cost: "<<cost2<<std::endl;
		if (cost2 < TRACKWIDTH*1.25)
		{
			if (colour == 'b')
			{
				left_cones.push_back(cn[index]);
				addedIDLeft.push_back(left_cones.back()->id);
			}
			else if (colour == 'y')
			{
				right_cones.push_back(cn[index]);
				addedIDRight.push_back(right_cones.back()->id);
			}
			return;
		}
		else
			return;

	}
		

	else if (cn.size() > 1)
	{
		int indx;
		for (int i=0;i<cn.size();i++)
		{		
			cost1 = computeCost1(cn[i],thisSide_cone.back());
			indx = findOppositeClosest(*(cn[i]),oppSide_cone);
			cost2 = computeCost2(cn[i],oppSide_cone[indx]);
			cost3 = computeCost3(cn[i],thisSide_cone);
			cost = cost1 + cost2 + cost3;
			if (least_cost > cost)
			{
				index = i;
				least_cost = cost;
			}
		}
		if (colour == 'b')
		{
			left_cones.push_back(cn[index]);
			addedIDLeft.push_back(left_cones.back()->id);
		}
		else if (colour == 'y')
		{
			right_cones.push_back(cn[index]);
			addedIDRight.push_back(right_cones.back()->id);
		}
	}

	

}


