#include "path_planner.h"
#include "path_point.h"
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <utility>
#include <string>

PathPlanner::PathPlanner(float car_x, float car_y, const std::vector<Cone> &cones, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : const_velocity(const_velocity), v_max(v_max), v_const(v_const), f_gain(max_f_gain), car_pos(PathPoint(car_x,car_y))
{
	raw_cones.reserve(250);
	l_cones_to_add.reserve(100);
	r_cones_to_add.reserve(100);

    // Add to list of raw_cones so that references can be made to be amended
    // Need to change to take in points not cones, these cones will then be converted to cones 
	//updateConePos(cones);
	addCones(cones);
	
    // Add the car position to the centre points 
    centre_points.emplace_back(car_x,car_y);
	if (timing_cones.size()==4) //there should be 4 timing cones (orange)
	{
		centralizeTimingCones();
		timingEmpty = false;
	}
    // Sort by distance to car 
    //sortConesByDist(centre_points.back(), centre_points.back());
	
    // Add CLOSEST cones to vector of known cones
    // left_cones.push_back(l_cones_to_add.front());
    // right_cones.push_back(r_cones_to_add.front());
	std::cout<<"left cones to add size: " << l_cones_to_add.size()<<std::endl;
	sortAndPushCone(l_cones_to_add);
	std::cout<<"sorted left cones size: " << left_cones.size()<<std::endl;
	std::cout<<"right cones to add size: " << r_cones_to_add.size()<<std::endl;
	sortAndPushCone(r_cones_to_add);
	std::cout<<"sorted right cones size: " << right_cones.size()<<std::endl;
	resetTempConeVectors();
    // Clear pointers and reset l/r_cones_to_add
	
    l_cones_sorted = false;
    r_cones_sorted = false;
    
    // Re-sort by distance to closest cone
    //sortConesByDist(left_cones.front()->position, right_cones.front()->position);
    //popConesToAdd();
    addFirstCentrePoints();
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
			if ((timing_cones.size() == 4) && timingEmpty) //there should be 4 timing cones
			{
				centralizeTimingCones();
				timingEmpty = false;
			}
			std::cout<<"left cones to add size: " << l_cones_to_add.size()<<std::endl;
			sortAndPushCone(l_cones_to_add);
			std::cout<<"sorted left cones size: " << left_cones.size()<<std::endl;
			std::cout<<"right cones to add size: " << r_cones_to_add.size()<<std::endl;
			sortAndPushCone(r_cones_to_add);
			std::cout<<"sorted right cones size: " << right_cones.size()<<std::endl;
			resetTempConeVectors();
			// sortConesByDist(left_cones.back()->position, right_cones.back()->position);
			// popConesToAdd();
			addCentrePoints(car_x, car_y);
			//addVelocityPoints();
		}

		returnResult(X, Y, V, Left, Right);
	}
}

bool PathPlanner::joinFeasible(const float &car_x, const float &car_y)
{
	if (calcDist(centre_points.back(), centre_points.front()) < 2)
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

float PathPlanner::calcAngle(const PathPoint &A, const PathPoint &B, const PathPoint &C)
{
	float cb_x = C.x - B.x;
	float cb_y = C.y - B.y;
	float ca_x = B.x - A.x;
	float ca_y = B.y - A.y;

	float angle = (atan2(cb_y, cb_x) - atan2(ca_y, ca_x)) * 180 / PI;

	return angle;
}

float PathPlanner::calcRelativeAngle(const PathPoint &p1, const PathPoint &p2)
{
	const float angle = atan2(p2.y - p1.y, p2.x - p1.x);
	return angle;
}



PathPoint PathPlanner::generateCentrePoint(const Cone* cone_one, const Cone* cone_two, bool& feasible)
{
	PathPoint midpoint(
		(cone_one->position.x + cone_two->position.x) / 2,
		(cone_one->position.y + cone_two->position.y) / 2
	);

	float dist_back = calcDist(centre_points.back(), midpoint);
	float angle = abs(calcAngle(*(centre_points.end() - 2), centre_points.back(), midpoint));
	//float angle = abs(calcAngle(this->car_pos, centre_points.back(), midpoint));
	//this checks if the generated centre point is within the 275-95 degrees
	if ((angle) < MIN_ANGLE && dist_back > 0.5 && dist_back < CP_DIST)//AZ:magic number dunno
	{
		std::cout << "Accepted point: " << midpoint.x << ',' << midpoint.y << std::endl;
		std::cout << "dist,  angle: " << dist_back << ' ' << angle << std::endl;
		feasible = true;
	}
	else
	{
		std::cout << "Rejected point: " << midpoint.x << ',' << midpoint.y << std::endl;
		std::cout << "dist,  angle: " << dist_back << ' ' << angle << std::endl;
		feasible = false;
	}

	return midpoint;
}

void PathPlanner::addCentrePoints(const float &car_x, const float &car_y) //dunno why car_x and car_y 
{
	bool feasible;
	PathPoint cp;
	int opp_idx;
	if (centre_points.size() > 1)
	{
		for (int i = 0; i < left_cones.size(); i++)
		{
			if (!left_cones[i]->mapped && left_cones[i]->times_checked < 25)
			{
				opp_idx = findOppositeClosest(*left_cones[i], right_cones);
				if (opp_idx != -1)
				{
					left_cones[i]->times_checked++;
					right_cones[opp_idx]->times_checked++;
					feasible = false;
					cp = generateCentrePoint(left_cones[i], right_cones[opp_idx], feasible);

					if (feasible)
					{
						centre_points.push_back(cp);	
						left_cones[i]->mapped = true;
					}
				}
			}
		}

		for (int i = 0; i < right_cones.size(); i++)
		{
			if (!right_cones[i]->mapped && right_cones[i]->times_checked < 25)	
			{
				opp_idx = findOppositeClosest(*right_cones[i], left_cones);
				if (opp_idx != -1)
				{
					feasible = false;
					cp = generateCentrePoint(right_cones[i], left_cones[opp_idx], feasible);
					left_cones[opp_idx]->times_checked++;
					right_cones[i]->times_checked++;

					if (feasible)
					{
						centre_points.push_back(cp);
						right_cones[i]->mapped = true;
					}
				}
			}
		}
	}
}

void PathPlanner::addCones(const std::vector<Cone> &new_cones)
{
	bool mapped = false;

	for (auto &cone: new_cones)
	{
		if (cone.colour == 'b')
		{
			mapped = false;
			for (auto n: mappedIDLeft)
			{
				if (n == cone.id)
				{
					mapped = true;
					break;
				}
			}
			if (!mapped)
			{
				raw_cones.push_back(cone);
				l_cones_to_add.push_back(&raw_cones.back());
				l_cones_sorted = false;
			}
		}
		else if (cone.colour == 'y')
		{
			mapped = false;
			for (auto m: mappedIDRight)
			{
				if (m == cone.id)
				{
					mapped = true;
					break;
				}
			}
			if (!mapped)
			{
				raw_cones.push_back(cone);
				r_cones_to_add.push_back(&raw_cones.back());
				r_cones_sorted = false;
			}
		}
		else
		{
			mapped = false;
			for (auto o: mappedIDRed)
			{
				if (o==cone.id)
				{
					mapped = true;
					break;
				}
			}
			if (!mapped)
			{
				raw_cones.push_back(cone);
				timing_cones.push_back(&raw_cones.back());
				mappedIDRed.push_back(timing_cones.back()->id);
				std::cout<<"timing cones found: "<< timing_cones.size() <<std::endl;
			}		
		}
	}
		
}
void PathPlanner::updateConePos(const std::vector<Cone> &new_cones)
{
	int i;
	for (auto &con:raw_cones)
	{
		i = con.id;
		con.position.x = new_cones[i].position.x;
		con.position.y = new_cones[i].position.y;
	}
}
     

void PathPlanner::addFirstCentrePoints()
{
    size_t n = std::min(left_cones.size(), right_cones.size());		
    int closest_opp_idx;
    float centre_x, centre_y;

    for (size_t i = 0; i < n; i++)
    {
	    if (!left_cones[i]->mapped)
	    {
		    closest_opp_idx = findOppositeClosest(*left_cones[i], right_cones);
		    if (closest_opp_idx != -1)
		    {
			    centre_x = (right_cones[closest_opp_idx]->position.x + left_cones[i]->position.x) / 2;
			    centre_y = (right_cones[closest_opp_idx]->position.y + left_cones[i]->position.y) / 2;
			    centre_points.push_back(PathPoint(centre_x, centre_y));
			    left_cones[i]->mapped = true;
			    right_cones[closest_opp_idx]->mapped = true;
		    }
	    }
    }
}

void PathPlanner::centralizeTimingCones()
{
	
		// Get average x and y position of all timing cones
		PathPoint avg_point(0, 0);
		float dist_back;
		float angle;

		for (int i = 0; i < timing_cones.size(); i++)
		{
			avg_point.x += timing_cones[i]->position.x; // summation of x positions
			avg_point.y += timing_cones[i]->position.y; // summation of y positions
		} 

		avg_point.x = avg_point.x / timing_cones.size(); // avg x dist
		avg_point.y = avg_point.y / timing_cones.size(); // avg y dist

		// Calc distance to previous centrepoint
		dist_back = calcDist(centre_points.back(), avg_point);
		angle = abs(calcAngle(*(centre_points.end()-2), centre_points.back(), avg_point));


		if (angle > MIN_ANGLE && angle < MAX_ANGLE && dist_back > 0.5 && dist_back < CP_DIST)//AZ:magic number dunno
		{
			std::cout << "Accepted point: " << dist_back << ' ' << angle << std::endl;
			centre_points.push_back(avg_point);
		}
		else
			std::cout<<"timing cones midpoint not added"<<std::endl;
	
}

int PathPlanner::findOppositeClosest(const Cone &cone, const std::vector<Cone*> &cones)
{
	uint8_t min_dist = OPP_CP_DIST;
	uint8_t dist;
	int index = -1;
	int i = 0;

	for (auto it = cones.begin(); it != cones.end(); it++)
	{
	    dist = calcDist(cone.position, (*it)->position);

	    if (dist < min_dist)
	    {
			min_dist = dist;
			index = i;
	    }
	    i++;
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
	l_cones_sorted = false;
	r_cones_sorted = false;
}

bool PathPlanner::compareConeDist(Cone* const &cone_one, Cone* const &cone_two)
{
    return cone_one->dist < cone_two->dist;
}

float PathPlanner::computeCost1(Cone* &cn1, Cone* &cn2)//distance between cones of same colour
{
	return calcDist(cn1->position,cn2->position);
}

float PathPlanner::computeCost2(Cone* &cn1, Cone* &cn2)//distance between cone from opposite side
//use findOppositeClosest first
{
	return calcDist(cn1->position,cn2->position);
}

float PathPlanner::computeCost3(Cone* &cn1, std::vector<Cone*> &cn2)//change in track curvature cn2: sorted cone
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
	//empty at first, so sort the cones by distance
	if (left_cones.empty() || right_cones.empty())
	{	
		sortConesByDist(car_pos);
		// while (!cn.empty())
		// {
			if (cn.front()->colour == 'b')
			{
				left_cones.push_back(cn.front());
				mappedIDLeft.push_back(left_cones.back()->id);
				//removeFirstPtr(cn);

			}
			else if (cn.front()->colour == 'y')
			{
				right_cones.push_back(cn.front());
				mappedIDRight.push_back(right_cones.back()->id);
				//removeFirstPtr(cn);
			}
		// }
		
	}
	else
	{
		float least_cost = 99999.22; //random large number
		float cost, cost1, cost2, cost3;
		int index = 0;
	
		if (cn.front()->colour == 'b') //if cone is blue(left)
		{
			std::cout<<"new blue cones found:  "<< cn.size() <<std::endl;	
			if (cn.size() > 1)
			{
				for (int i=0;i<cn.size();i++)
				{
					std::cout<<"blue for i"<<i <<cn[i]<<"  back"<<right_cones.back()<<std::endl;		
					cost1 = computeCost1(cn[i],left_cones.back());
					cost2 = computeCost2(cn[i],right_cones.back());
					cost3 = computeCost3(cn[i],left_cones);
					cost = cost1 + cost2 + cost3;
					if (least_cost > cost)
					{
						index = i;
						least_cost = cost;
					}
				}
				left_cones.push_back(cn[index]);
				mappedIDLeft.push_back(left_cones.back()->id);

			}
			else if (cn.size()==1)
			{
				left_cones.push_back(cn.front());
				mappedIDLeft.push_back(left_cones.back()->id);
			}

		}
		else if (cn.front()->colour == 'y') //if cone is yellow(right)
		{
			std::cout<<"new yellow cones found: "<< cn.size() <<std::endl;
			if (cn.size()>1)
			{	
				for (int i = 0;i<cn.size();i++)
				{
					cost1 = computeCost1(cn[i],right_cones.back());
					cost2 = computeCost2(cn[i],left_cones.back());
					cost3 = computeCost3(cn[i],right_cones);
					cost = cost1+cost2+cost3;
					if (least_cost > cost)
					{
						index = i;
						least_cost = cost;
					}
				}
				right_cones.push_back(cn[index]);
				mappedIDRight.push_back(right_cones.back()->id);
			}
			else if (cn.size()==1)
			{
				right_cones.push_back(cn.front());
				mappedIDRight.push_back(right_cones.back()->id);
			}
				
		}
	}

}


