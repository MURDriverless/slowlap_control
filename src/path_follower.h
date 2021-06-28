#ifndef SRC_PATH_FOLLOWER_H
#define SRC_PATH_FOLLOWER_H

#include <vector>
#include <cstdint>
#include <memory>
#include "path_point.h"

//#include <eigen3/unsupported/Eigen/Splines>

#define PI 3.14159265359
#define LENGTH 2.95 //length from fron to rear wheels
#define G  9.81
#define MAX_ACC 11.772//1.2*G
#define MAX_DECEL -17.658//-1.8*Gg
#define MAX_STEER 0.8 //

	//PID gains:
#define KP 1
#define KI 1
#define KD  1

	//pure pursuit gains
#define LFV  0.1
#define LFC  2


class PathFollower
{
public:
	float vLength = LENGTH;

	//positions
	float rear_X;
	float rear_Y;

	//index

	//output
	float acceleration;
	float steering;
	//PathFollower(); //constructor
	float CalcRear2Target(float x, float y, float yaw, float targetX, float targetY);
	float SearchTargetIndex(float carX, float carY, float carV, float carYaw, float X, float Y,float V);
	void AccelerationControl(float targetSpeed, float currentSpeed);
	void SteeringControl(float x, float y, float yaw, float targetX, float targetY, float Lf);
		//float SearchTargetIndex(float carX, float carY, float carV, float carYaw, const std::vector<float*>&,const std::vector<float*>&,const std::vector<float*>&);
	
	float ConstrainOutputSteer(float steer);
	float Convert2Threshold(float acc);
	void Control(float carX,float carY,float carV, float carYaw, float X, float Y, float V);

	std::vector<PathPoint> path;

	//PID gains:
//	float Kp = 1;
//	float Ki = 1;
//	float Kd = 1;

	//pure pursuit gains
//	float Lfv = 0.1;
//	float Lfc = 2;




};
#endif // SRC_PATH_FOLLOWER_H