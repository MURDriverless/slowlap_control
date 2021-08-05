#include "follower.h"
#include <iostream>


/**
 * Slow Lap Path Follower 
 * How this works:
 * -receive path points from path planner
 * -spline points to have smoother curves
 * -use proportional control to give acceleration commands
 * -use Pure Pursuit Control to give steering commands
 * 
 * author: Aldrei Recamadas (MURauto2021)
 * ***/

//constructor
PathFollower::PathFollower(ros::NodeHandle n):nh(n)
{
    path_x.reserve(500);
    path_y.reserve(500);
    centre_points.reserve(500);
    centre_splined.reserve(2000);
    centre_endOfLap.reserve(100);
    xp.reserve(100);
    yp.reserve(100);
    T.reserve(100);
    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }

    waitForMsgs();

    //set current position as first goal point
    currentGoalPoint = PathPoint(car_x,car_y); 
    ROS_INFO_STREAM("FOLLOWER: follower initialized, publishers and subscribers launched!");
}

void PathFollower::waitForMsgs()
{
    while (!odom_msg_received || !path_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

// this is like void loop() in Arduino
void PathFollower::spin()
{
    waitForMsgs();
    updateRearPos();
    if (centre_points.size()>1)
    {
        steeringControl();
        accelerationControl();
    }
    pushcontrol();
    pushDesiredAccel();
    pushDesiredCtrl();
    // pushTarget();
    //pushPathViz();  
    clearVars();
}

//to clear temporary vectors
void PathFollower::clearVars()
{
    path_x.clear();
    path_y.clear();
    xp.clear();
    yp.clear();
    T.clear();
    odom_msg_received = false;
    path_msg_received = false;
    new_centre_points = false;

}

// standard ROS function
int PathFollower::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &PathFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &PathFollower::pathCallback, this);
    
}

// standard ROS function
int PathFollower::launchPublishers()
{
    pub_control = nh.advertise<mur_common::actuation_msg>(CONTROL_TOPIC, 10);
    pub_accel = nh.advertise<geometry_msgs::Accel>(ACCEL_TOPIC, 10);
    pub_steer = nh.advertise<geometry_msgs::Twist>(STEER_TOPIC, 10);
    pub_target =  nh.advertise<geometry_msgs::PoseStamped>("TargetNode", 10);
    pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
}

// get odometry message from SLAM
void PathFollower::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_v = msg.twist.twist.linear.x; //velocity
    
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;
    //convert quaternion to Euler
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    car_yaw = yaw;

    odom_msg_received = true;
}

// get path message from path planner
void PathFollower::pathCallback(const mur_common::path_msg &msg)
{
    for (int i=0; i < msg.x.size(); i++)
    {
        path_x.push_back(msg.x[i]);
        path_y.push_back(msg.y[i]);
    }
    path_msg_received = true;
    
    if (centre_points.size() != path_x.size()) //add end of lap
    {
        new_centre_points = true;
        for(int i=centre_points.size();i<path_x.size();i++)
        {
            centre_points.emplace_back(path_x[i],path_y[i]);
            // if (DEBUG) std::cout<<"\n FOLLOWER new points received: "<< centre_points[i].x<<", "<<centre_points[i].y<<std::endl;
            
        }
        // std::cout << "path points size: "<<centre_points.size()<<std::endl;
        if (centre_points.size()>1) generateSplines();
        
    }
    
}
void PathFollower::pushcontrol()
{
    mur_common::actuation_msg ctrl_msg;

    ctrl_msg.acceleration_threshold = acceleration;
    ctrl_msg.steering = steering;

    pub_control.publish(ctrl_msg);
    // std::cout<<"ctr_msg: "<<ctrl_msg<<std::endl;
}
void PathFollower::pushDesiredCtrl()
{
    geometry_msgs::Twist ctrl_desired; 
    float next_v = car_v + acceleration*DT;
    ctrl_desired.linear.x = next_v + cos(car_yaw);
    ctrl_desired.linear.y = next_v + sin(car_yaw);
    ctrl_desired.angular.z = steering;
    pub_steer.publish(ctrl_desired);
}
void PathFollower::pushDesiredAccel()
{
    geometry_msgs::Accel accel_desired;
    accel_desired.linear.x = acceleration * cos(car_yaw);
    accel_desired.linear.y = acceleration * sin(car_yaw);
    pub_accel.publish(accel_desired);
}
void PathFollower::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(centre_points.size());

    for (int p = 0; p < centre_points.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = "map";
        item.header.seq = p;
        item.pose.position.x = centre_points[p].x;
        item.pose.position.y = centre_points[p].y;
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);
}

void PathFollower::generateSplines()
{
    if (endOfLap)
    {
        if (stopSpline)
            return;
        
        xp.push_back(car_x);
        yp.push_back(car_y);
        T.push_back(0);
        for (int i=0; i<STOP_INDEX+2; i++)
        {
            xp.push_back(centre_points[i].x);
            yp.push_back(centre_points[i].y);
            T.push_back(i+1);
        }
        tk::spline sx, sy;
        sx.set_points(T, xp);
        sy.set_points(T, yp);
        for (float i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_endOfLap.emplace_back(sx(i),sy(i));
        }
        if (DEBUG) std::cout<<"[splines] end of lap splined. splined path size: "<<centre_endOfLap.size()<<std::endl;
        stopSpline = true;
    }
    
    //there must be at least 3 points for cubic spline to work
    else if (centre_points.size() == 2) //if only 2, make a line
    {
        float tempX, tempY;
        for (auto p:centre_points)
        {
            xp.push_back(p.x);
            yp.push_back(p.y);
        }

        float slopeY = (yp.back() - yp.front()) / STEPSIZE;
        float slopeX = (xp.back() - xp.front()) / STEPSIZE;
        for (float i = 0; i<=xp.size(); i+= STEPSIZE)
        {
            tempY = slopeY * i + yp.front();
            tempX = slopeX * i + xp.front();
            centre_splined.emplace_back(tempX,tempY);
        }
        if (DEBUG) std::cout<<" first centre_splined xp, yp : "<<xp.front()<<yp.front()<<std::endl;
        
    }

    else if (centre_points.size()>SPLINE_N) //we will only spline the last N points as it is computationally expensive
    {
        //separate x and y values
        int t = 0;
        for (int i = centre_points.size()- SPLINE_N; i < centre_points.size(); i++)
            {
                xp.push_back(centre_points[i].x);
                yp.push_back(centre_points[i].y);
                T.push_back(t);
                t++;
            }

        // Generate Spline Objects
        // spline and x and y separately
        // (see how tk::spline works)
        tk::spline sx, sy;
        sx.set_points(T, xp);
        sy.set_points(T, yp);
        
        int temp = (centre_points.size() - SPLINE_N )/ STEPSIZE;
        centre_splined.assign(centre_splined.begin(),centre_splined.begin()+ temp);  //erase the last N points, then replace with new points
        for (float i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
        // if (DEBUG) std::cout<<"[splines] new centre_splined size is: "<<centre_splined.size()<<std::endl;
    }

    else //for 2 < centre points size < 10 
    {
        xp.clear();
        yp.clear();
        T.clear();

        //separate x and y values
        int t=0;
        for (auto p:centre_points)
        {
            xp.push_back(p.x);
            yp.push_back(p.y);
            T.push_back(t);
            t++;
        }

        // Generate Spline Objects
        // spline and x and y separately
        // (see how tk::spline works)
        tk::spline sx, sy;
        sx.set_points(T, xp);
        sy.set_points(T, yp);

        centre_splined.clear(); //erase centre_splined and replace with new points
        for (float i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
        // if (DEBUG) std::cout<<"[splines] new centre_splined x, y : "<<xp.back()<<yp.back()<<std::endl;
    }
       
}

void PathFollower::accelerationControl()
{
    //this is just a P controller for now, since velocity is kept constant
    //can make this into a PID if we have varying velocity
    //in the future, targetSpeed can be changed
    float targetSpeed = V_CONST;
    if (slowDown || centre_points.size() < 3)
        targetSpeed = 0.6 * V_CONST;
    if (slowLapFinish)
        targetSpeed = 0.0;
    
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

void PathFollower::steeringControl()
{
    
    float dist = getGoalPoint();
    if (endOfLap)
    {
        if (dist < 0.5)
        {  
            stopCar = true;
            ROS_INFO_STREAM("SLOW LAP FINISHED!");
            slowLapFinish = true;
        }     
    }

    if (DEBUG) std::cout << "FOLLOwER: Current car pose: ("<<car_x<<", "<<car_y
        <<"). Goal point: ("<<currentGoalPoint.x<<", "<<currentGoalPoint.y<<"), "<<dist<<"m away"<<std::endl;

   
    if (endOfPath)
    {
        if (DEBUG) std::cout<<"[steeringControl] end of path triggered!"<<std::endl;
        slowDown = true;
        
        if (getDistFromCar(centre_splined.front())< Lf)//if 1 look ahead distance away from finish/start line
        {
            ROS_INFO_STREAM("End of lap near");
            endOfLap = true;
            generateSplines(); //trigger splining of centre_endOfPath
        }
    }
    float alpha = atan2((currentGoalPoint.y - rearY),(currentGoalPoint.x - rearX)) - car_yaw;
	float steer = atan2((2 * LENGTH * sin(alpha))/Lf,1);

    // //constrain
	// if (abs(steer) >= MAX_STEER)
	// 	steering =   (MAX_STEER - 0.001); //copied from sanitise output
	// // else if (steer <= -MAX_STEER)
	// // 	steering =  -(MAX_STEER - 0.001);
	// else
	// 	steering =  steer;
     //constrain
	if (steer >= MAX_STEER)
		steering =   (MAX_STEER - 0.001); //copied from sanitise output
	else if (steer <= -MAX_STEER)
		steering =  -(MAX_STEER - 0.001);
	else
		steering =  steer;
}

void PathFollower::updateRearPos()
{
    rearX = car_x - ((LENGTH / 2) * cos(car_yaw));
	rearY = car_y - ((LENGTH / 2) * sin(car_yaw));
}

float PathFollower::getDistFromCar(PathPoint &pnt) //from rear wheels
{
    //updateRearPos();
    float dX = rearX - pnt.x;
	float dY = rearY - pnt.y;
    return sqrt(dX * dX + dY * dY);
}


/*************
* This function searches for the goal point from the splined path points 
*  (searches for the index of the goal point from centre_splined vector)
* The concept of look ahead distance of the pure puruit controller is used here
* it returns the distance to the current goal point
**/
float PathFollower::getGoalPoint()
{
    float dist = getDistFromCar(currentGoalPoint);
    //look ahead distance
    Lf = LFC;
    //if velocity is not constant, we can adjust lookahead dist using the formula:
    // Lf = LFV * car_lin_v + LFC;

    if (Lf <= dist && !endOfLap) //dont update goal point yet
        {
            // if (DEBUG) std::cout<<"FOLLOWER: Current goal point is: ("<<currentGoalPoint.x<<", "<<currentGoalPoint.y<<"), "<<dist<<"m away"<<std::endl;
            return dist;
        }
    
    
    if (endOfLap) //if end of lap, change reference path to centre_endOfLap
    {
        currentGoalPoint.updatePoint(centre_endOfLap[index_endOfLap]); 
        if ((Lf) > getDistFromCar(currentGoalPoint))
        {
            index_endOfLap ++;
            currentGoalPoint.updatePoint(centre_endOfLap[index_endOfLap]); 

            if (index_endOfLap >= centre_endOfLap.size()-(STOP_INDEX/STEPSIZE)) //last point/ stopping point
                index_endOfLap = centre_endOfLap.size()-(STOP_INDEX/STEPSIZE);
        }

        if (DEBUG) std::cout << "end of lap goal near" << std::endl;   
        return dist;
    }

    float temp; //temporary var
    
    //search for index with distance to car that is closest to look ahead distance
    while (true)
    {
        if (index+1 >= centre_splined.size())
            break;
        dist = getDistFromCar(centre_splined[index]);
        if (dist >= Lf)
            break;
        else
            index++;
    }
    // if (DEBUG) std::cout<<"[getGoalPoint] new goal point set" <<std::endl;
    
    if (index == centre_splined.size()-1) //if at last index of centre_splined path
    {
        endOfPath = true;
        currentGoalPoint.updatePoint(centre_splined.back());
        return getDistFromCar(currentGoalPoint);
        // if (DEBUG) std::cout<<"[getGoalPoint] car near end of path" <<std::endl;
    }
    else
    {
        endOfPath = false;
        currentGoalPoint.updatePoint(centre_splined[index]); //return value
        return getDistFromCar(currentGoalPoint);
    }
        
}


float PathFollower::Quart2EulerYaw(float q_x, float q_y, float q_z, float q_w)
    {
        float siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
        float cosy_cosp = 1.0 - (2.0 * (q_y * q_y + q_z * q_z));
        return std::atan2(siny_cosp, cosy_cosp);
    }

float PathFollower::getSign(float &num)
{
    if (num < 0)
        return -1;
    else 
        return 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathFollower"); 
    ros::NodeHandle n;
       
        PathFollower follower(n);
        //ros::Rate freq(20);
	
	    while (ros::ok())
	    {
	        follower.spin();
          //  freq.sleep();
	    }
    
}