#include <ros/ros.h>
#include "path_planner.h"
#include <stdio.h>
#include "node.h"

bool constant_v = false;
float v_max = 5.0;
float v_const = 3.0;
float max_f_gain = 3.0;

void checkParams(ros::NodeHandle &n)
{
    if (!n.hasParam("constant_v"))
    {
	ROS_INFO_STREAM("Planner: No 'const_v' param found. Defaulting");
    }
    if (!n.hasParam("v_max"))
    {
	ROS_INFO_STREAM("Planner: No 'v_max' param found. Defaulting");
    }
    if (!n.hasParam("v_const"))
    {
	ROS_INFO_STREAM("Planner: No 'v_const' param found. Defaulting");
    }
    if (!n.hasParam("max_f_gain"))
    {
	ROS_INFO_STREAM("Planner: No 'max_f_gain' param found. Defaulting");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slowLapNode"); 
    ros::NodeHandle n;

    if (ros::ok())
    {
	checkParams(n);
	n.getParam("constant_v", constant_v);
	n.getParam("v_max", v_max);
	n.getParam("v_const", v_const);
	n.getParam("max_f_gain", max_f_gain);

	PlannerNode planner(n, constant_v, v_max, v_const, max_f_gain);

	while (ros::ok())
	{
	    planner.spinThread();
        if (planner.planner->complete)
            break;
	}
    }
}
