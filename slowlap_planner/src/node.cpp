#include "path_planner.h"
#include "node.h"
#include <iostream>
#include <vector>
#include <assert.h>
#include <limits>
#include <numeric>

PlannerNode::PlannerNode(ros::NodeHandle n, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : nh(n), const_velocity(const_velocity), v_max(v_max), v_const(v_const), max_f_gain(max_f_gain)
{
    times.reserve(std::numeric_limits<uint16_t>::max());
    rtimes.reserve(std::numeric_limits<uint16_t>::max());
    launchSubscribers();
    launchPublishers();
    
    waitForMsgs();

    try
    {
	this->planner = std::unique_ptr<PathPlanner>(new PathPlanner(car_x, car_y, cones, const_velocity, v_max, v_const, max_f_gain));
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);	
    }

    ROS_INFO_STREAM("Planner: Planner initialized");

    now = ros::Time::now();

    cone_msg_received = false;
    odom_msg_received = false;
}

void PlannerNode::waitForMsgs()
{
    while (!cone_msg_received || !odom_msg_received && ros::ok()) 
    {
	ros::spinOnce();
	ros::Duration(0.005).sleep();
    }
}

int PlannerNode::launchSubscribers()
{
    try
    {
	sub_odom = nh.subscribe(ODOM_TOPIC, 1, &PlannerNode::odomCallback, this);
	sub_cones = nh.subscribe(CONE_TOPIC, 1, &PlannerNode::coneCallback, this);
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);
	return 0;
    }
    ROS_INFO_STREAM("Planner: Odometry and cone subscribers connect");
    return 1;
}

int PlannerNode::launchPublishers()
{
    try
    {
        //pub_path = nh.advertise<mur_common::path_msg>(PATH_TOPIC, 1);
        pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
        pub_health = nh.advertise<mur_common::diagnostic_msg>(HEALTH_TOPIC, 1);
        pub_control = nh.advertise<mur_common::actuation_msg>(CONTROL_TOPIC, 1000);//new
        pub_lcones = nh.advertise<mur_common::cone_msg>(SORTED_LCONES_TOPIC, 1);
        pub_rcones = nh.advertise<mur_common::cone_msg>(SORTED_RCONES_TOPIC, 1);

    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
        return 0;
    }

    ROS_INFO_STREAM("Planner: Path, visualisation, diagnostic publishers connected");
    return 1;
}

void PlannerNode::printVectors()
{
    for (int i = 0; i < X.size(); i++)
    {
	std::cout << X[i] << ' ' << Y[i] << ' ' << V[i] << std::endl;
    }
}

void PlannerNode::spinThread()
{
    auto rstart = Clock::now();
    clearTempVectors();
    waitForMsgs();
    auto start = Clock::now();
    planner->update(cones, car_x, car_y, X, Y, V, Left, Right);
    auto end = Clock::now();
    pushPath();
    pushPathViz();
    pushSortedCones();
    cone_msg_received = false;
    odom_msg_received = false;
    auto rend = Clock::now();
    pushHealth(start, end, rstart, rend);
////
    AccelerationControl();
    int siz = X.size();
    SteeringControl(X.back(),Y.back());
    pushControl();
    //pushDesiredCtrl();
    //pushDesiredAccel();
}

void PlannerNode::pushHealth(ClockTP& s, ClockTP& e, ClockTP& rs, ClockTP& re)
{
    mur_common::diagnostic_msg h;
    times.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(e - s).count());
    rtimes.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(re - rs).count());
    float mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    float rmean = std::accumulate(rtimes.begin(), rtimes.end(), 0.0) / rtimes.size();
    h.compute_times = times;
    h.full_compute_times = rtimes;
    h.avg_times = mean;
    h.full_avg_times = rmean;
    pub_health.publish(h);
}

void PlannerNode::clearTempVectors()
{
    X.clear();
    Y.clear();
    V.clear();
    Left.clear();
    Right.clear();
    cones.clear();
}

void PlannerNode::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(X.size());

    for (int p = 0; p < X.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = "map";
        item.header.seq = p;
        item.pose.position.x = X[p];
        item.pose.position.y = Y[p];
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
}
void PlannerNode::pushSortedCones()// const
{
    ros::Time current_time = ros::Time::now();

    mur_common::cone_msg cl; //left cones
    cl.header.frame_id = "map";
    cl.header.stamp = current_time;

    mur_common::cone_msg cr; //right cones
    cr.header.frame_id = "map";
    cr.header.stamp = current_time;

    std::vector<float> coneX, coneY;
    std::vector<std::string> col;
    coneX.reserve(Left.size());
    coneY.reserve(Left.size());
    col.reserve(Left.size());

    for (int i = 0;i<Left.size();i++)
    {
        coneX.push_back(Left[i].position.x);
        coneY.push_back(Left[i].position.y);
        col.push_back("BLUE");
    }
    cl.x = coneX;
    cl.y = coneY;
    cl.colour = col;
    
    coneX.reserve(Right.size());
    coneY.reserve(Right.size());
    col.reserve(Right.size());
    for (int i = 0;i<Right.size();i++)
    {
        coneX.push_back(Right[i].position.x);
        coneY.push_back(Right[i].position.y);
        col.push_back("YELLOW");
    }
    cr.x = coneX;
    cr.y = coneY;
    cr.colour = col;

    pub_lcones.publish(cl);
    pub_rcones.publish(cr);
}

// no need to publish path
// didnt change function name
void PlannerNode::pushPath()
{
    
    //path_point.x = X.back();
    //path_point.y = Y.back();
    //path_point.velocity = V.back();

}

void PlannerNode::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_v = msg.twist.twist.linear.x; //velocity
    float q_x = msg.pose.pose.orientation.x;
    float q_y = msg.pose.pose.orientation.y;
    float q_z = msg.pose.pose.orientation.z;
    float q_w = msg.pose.pose.orientation.w;
    yaw = Quart2EulerYaw(q_x, q_y, q_z, q_w);
    odom_msg_received = true;
}

void PlannerNode::coneCallback(const mur_common::cone_msg &msg)
{
    for (int i = 0; i < msg.x.size(); i++)
    {
	    if (msg.colour[i] == "BLUE")
	    {
	      cones.push_back(Cone(msg.x[i], msg.y[i], 'b', i));
          cones.back().id = i;
	    }
	    else if (msg.colour[i] == "YELLOW")
	    {
	        cones.push_back(Cone(msg.x[i], msg.y[i], 'y', i)); 
            cones.back().id = i;
	    }
	    else if (msg.colour[i] == "na")
	    {
	        std::cout << "'na' cone colour passed, skipping" << std::endl;
            //cones.back().id = i;
	    }
	    else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
	    {
	        cones.push_back(Cone(msg.x[i], msg.y[i], 'r', i));
            cones.back().id = i;
	    }
    }
    cone_msg_received = true;
}


//publish actuation control
void PlannerNode::pushControl()//new
{
    mur_common::actuation_msg ctrl_msg;

    //ctrl_msg.header.seq = 1;
   // ctrl_msg.header.frame_id = "map";
    ctrl_msg.acceleration_threshold = acceleration;
    ctrl_msg.steering = steering;

    pub_control.publish(ctrl_msg);
}
//publish actuation control
void PlannerNode::pushDesiredCtrl()//new
{
    geometry_msgs::Twist ctrl_desired; 
    float next_v = car_v + acceleration*DT;
    ctrl_desired.linear.x = next_v + cos(yaw);
    ctrl_desired.linear.y = next_v + sin(yaw);
    ctrl_desired.angular.z = steering;
    pub_control.publish(ctrl_desired);
}
void PlannerNode::pushDesiredAccel()//new
{
    geometry_msgs::Accel accel_desired;
    accel_desired.linear.x = acceleration * cos(yaw);
    accel_desired.linear.y = acceleration * sin(yaw);
    pub_control.publish(accel_desired);
}

void PlannerNode::AccelerationControl()
{
    //this is just a P controller for now, since velocity is kept constant
    //can make this into a PID if we have varying velocity
    //in the future, targetSpeed can be changed
    float targetSpeed = v_const;
    float acc = KP * (targetSpeed - car_v);

	 //constrain
	if (acc >= MAX_ACC)
		acc = MAX_ACC;
	else if (acc <= MAX_DECEL)
		acc =  MAX_DECEL;
	
	acceleration = acc;

	/*convert to threshold
	if (acc > 0){
		acceleration =  acc/MAX_ACC; //acceleration
	}
	else{
		acceleration =  acc/MAX_DECEL; //acceleration
	}*/
}
void PlannerNode::SteeringControl(float targetX, float targetY)
{
	//convert position from COM frame to rear wheel frame
	float rearX = car_x - ((LENGTH / 2) * cos(yaw));
	float rearY = car_y - ((LENGTH / 2) * sin(yaw));
    float dX = rearX - targetX;
	float dY = rearY - targetY;
    float Lf = sqrt(dX * dX + dY * dY);

	float alpha = atan2((targetY - rearY),(targetX - rearX)) - yaw;
	float steer = atan2((2 * LENGTH * sin(alpha)),Lf);

    //constrain
	if (steer >= MAX_STEER)
		steering =   (MAX_STEER - 0.001); //copied from sanitise output
	else if (steer <= -MAX_STEER)
		steering =  -(MAX_STEER - 0.001);
	else
		steering =  steer;
  
}

float PlannerNode::Quart2EulerYaw(float q_x, float q_y, float q_z, float q_w)
    {
        float siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
        float cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
        return std::atan2(siny_cosp, cosy_cosp);
    }



