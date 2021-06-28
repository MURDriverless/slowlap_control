
#include <algorithm>
#include <math.h>
#include <utility>
#include <string>
 
#include "path_follower.h"

/*PathFollower::PathFollower()
	: rear_X(rear_X), rear_Y(rear_Y),acceleration(acceleration),acc_threshold(acc_threshold),steering(steering)
	{}
*/

float PathFollower::CalcRear2Target(float x, float y, float yaw, float targetX, float targetY)
{
	//convert current position of car from central vehicle frame to rear vehicle frame
	float rear_X = x - ((vLength / 2) * cos(yaw));
	float rear_Y = y - ((vLength / 2) * sin(yaw));

	float dX = rear_X - targetX;
	float dY = rear_Y - targetY;
	float a = sqrt(dX * dX + dY * dY);
	return a;
}

float PathFollower::SearchTargetIndex(float carX, float carY, float carV, float carYaw, float X, float Y,float V)
//float SearchTargetIndex(float carX, float carY, float carV, float carYaw, const std::vector<float*>&X,const std::vector<float*>&Y,const std::vector<float*>&V)
{
	float Lf = (LFV * carV) + LFC;
	//might need to change X and Y to vectors
	/*int i = 0;
	float dis = CalcRear2Target(carX, carY, carYaw,X,Y);
	while(dis<Lf)
	{
		if (i+1 >= X.size())
			break;
		i++;
		dis = CalcRear2Target(carX, carY, carYaw, X,Y);
	}
	*/
	return Lf;

}
void PathFollower::AccelerationControl(float targetSpeed, float currentSpeed)
{
	 float acc = KP * (targetSpeed - currentSpeed);

	 //constrain
	 if (acc >= MAX_ACC)
		acc = MAX_ACC;
	else if (acc <= MAX_DECEL)
		acc =  MAX_DECEL;


	//convert to threshold
	if (acc > 0){
		acceleration = acc/MAX_ACC;
	}
	else{
		acceleration = acc/MAX_DECEL;
	}

}

void PathFollower::SteeringControl(float x, float y, float yaw, float targetX, float targetY, float Lf)
{
	//convert position from COM frame to rear wheel frame
	float rearX = x - ((vLength / 2) * cos(yaw));
	float rearY = y - ((vLength / 2) * sin(yaw));

	float alpha = atan2((targetY - rearY),(targetX - rearX)) - yaw;
	float steer = atan2((2 * vLength * sin(alpha)),Lf);
	//return steerAngle; 

//constrain
	if (steer >= MAX_STEER)
		steering =  MAX_STEER - 0.001; //copied from sanitise output
	else if (steer <= -MAX_STEER)
		steering =  -(MAX_STEER - 0.001);
	else
		steering = steer;
  
}



/*  //combined with AccelerationControl
float PathFollower::ConstrainOutput(float acc) //anti windup? 
{
	if (acc >= MAX_ACC)
		return MAX_ACC;
	else if (acc <= MAX_DECEL)
		return MAX_DECEL;
	else
		return acc;
}
float PathFollower::ConstrainOutputSteer(float steer) //
{
	if (steer >= MAX_STEER)
		return MAX_STEER - 0.001; //copied from sanitise output
	else if (steer <= -MAX_STEER)
		return -(MAX_STEER - 0.001);
	else
		return steer;
}

void PathFollower::Convert2Threshold(float acc) //from sanitise output
{
	if (acc > 0){
		acc_threshold = acc/MAX_ACC;
	}
	else{
		acc_threshold = acc/MAX_DECEL;
	}
}*/



//void PathFollower::Control(float carX,float carY,float carV, float carYaw, const std::vector<float> &X,const std::vector<float> &Y,const std::vector<float> &V)
void PathFollower::Control(float carX,float carY,float carV, float carYaw,float X, float Y, float V)
{
	float Lf = SearchTargetIndex(carX,carY,carV,carYaw,X,Y,V);
	SteeringControl(carX,carY,carYaw,X,Y,Lf);
	AccelerationControl(V, carV);
	//Convert2Threshold(acceleration);

}




