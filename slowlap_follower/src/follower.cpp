/**
 * This is the Path follower for the husky
 * it receives path information from path planner
 * then passes actuation commands to the Husky
 * 
 * uses pure pursuit controller, velocity is constant for now
 * 
 * see header file for descriptions of member variables
 * author: Aldrei (MURauto21)
*/

#include "follower.h"
#include <iostream>

// constructor
PathFollower::PathFollower(ros::NodeHandle n, double max_v, double max_w)
                :nh(n), max_v(max_v),max_w(max_w)
{
    //set capacity of vectors
    centre_points.reserve(500);
    centre_splined.reserve(2000);
    xp.reserve(200);
    yp.reserve(200);
    T.reserve(200);

    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    waitForMsgs();

    ROS_INFO_STREAM("[FOLLOWER] follower initialized, publisher and subscriber launched!");
}

// spinonce when msgs are received
void PathFollower::waitForMsgs()
{
    if (!path_msg_received || !odom_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

// void loop()
void PathFollower::spin()
{
    waitForMsgs();
    DrivingControl();
    publishCtrl();
    pushPathViz();
    pushDesiredAccel();
    pushDesiredCtrl();
    clearVars();
    
    if (fastLapReady)
        shut_down();

    ros::Rate(HZ).sleep();
}

// clear temporary vectors and flags
void PathFollower::clearVars()
{
    odom_msg_received = false;
    path_msg_received = false;
    new_centre_points = false;
    cenPoints_updated = 0;
    newGP = false;
    xp.clear();
    yp.clear();
    T.clear();
}

//standard ROS func
int PathFollower::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &PathFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &PathFollower::pathCallback, this);
    sub_transition = nh.subscribe(FASTLAP_READY_TOPIC, 1, &PathFollower::transitionCallback, this);
}

//standard ROS func
int PathFollower::launchPublishers()
{
    pub_control = nh.advertise<mur_common::actuation_msg>(CONTROL_TOPIC, 10);
    pub_accel = nh.advertise<geometry_msgs::Accel>(ACCEL_TOPIC, 10);
    pub_steer = nh.advertise<geometry_msgs::Twist>(STEER_TOPIC, 10);
    pub_target =  nh.advertise<geometry_msgs::PoseStamped>("TargetNode", 10);
    pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
    pub_goalPt = nh.advertise<visualization_msgs::Marker>(GOALPT_VIZ_TOPIC, 1);
}

//standard ROS func. gets transition msg from fast lap
void PathFollower::transitionCallback(const mur_common::transition_msg &msg)
{
    fastLapReady = msg.fastlapready;
}

// get odometry messages
void PathFollower::odomCallback(const nav_msgs::Odometry &msg)
{
    
    if (!initialised)
    {
       initX = msg.pose.pose.position.x;
       initY = msg.pose.pose.position.y;
       initYaw = car_yaw;
       initialised = true;
       
    }
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    updateRearPos();
    
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;

    car_v = msg.twist.twist.linear.x;

    // convert quaternions to euler
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    car_yaw = yaw;

    // this formula is from Dennis, it works somehow
    car_yaw2 = 2 *asin(abs(q_z)) * getSign(q_z) * getSign(q_w);
    car_yaw2 = yaw;
    odom_msg_received = true;
}

//get path msgs from path planner
void PathFollower::pathCallback(const mur_common::path_msg &msg)
{   
    // if the last 5 path points have changed, copy new path points
    int j =0;
    for (int i=centre_points.size()-1; i>=0 ;i--)
    {
        if (calcDist(centre_points[i],PathPoint(msg.x.back(),msg.y.back()))>0.01)
        {
            new_centre_points = true;
            break;
        }
        if (j>5) break;
        j++;
    }
    
    //copy path points msg
    if (centre_points.empty() || new_centre_points)
    {
        centre_points.clear();
        for (int i=0; i < msg.x.size(); i++)
        {
            centre_points.emplace_back(msg.x[i],msg.y[i]);
        }
        generateSplines();
    }

    //check if lap is complete
    if (calcDist(PathPoint(initX,initY),centre_splined.back())<0.02)
        plannerComplete = true;

    path_msg_received = true;
    if (DEBUG)
    {
        std::cout<<"[FOLLOWER] path points received: "<<centre_points.size()<<std::endl;
        for (auto &p:centre_points)
        {
            std::cout<<"("<<p.x<<", "<<p.y<<") ";
        }
        std::cout<<"\n";
    }
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

// publish splined path to RVIZ
void PathFollower::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = FRAME;

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(centre_splined.size());

    for (int p = 0; p < centre_splined.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = FRAME;
        item.header.seq = p;
        item.pose.position.x = centre_splined[p].x;
        item.pose.position.y = centre_splined[p].y;
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);

    // visualise goal point
    visualization_msgs::Marker marker;
    marker.header.frame_id = FRAME;
    marker.header.stamp = ros::Time();
    marker.header.seq = index;
    marker.ns = "my_namespace";
    marker.id = index;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1/HZ);
    marker.pose.position.x = currentGoalPoint.x;
    marker.pose.position.y = currentGoalPoint.y;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    
    // alpha and RGB settings
    // color.a is opacity, 0=invisible
    marker.color.a = 1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    pub_goalPt.publish(marker);
}

//publish actuation control commands
void PathFollower::publishCtrl()
{
    mur_common::actuation_msg ctrl_msg;

    ctrl_msg.acceleration_threshold = acceleration;
    ctrl_msg.steering = steering;

    pub_control.publish(ctrl_msg);
    // std::cout<<"ctr_msg: "<<ctrl_msg<<std::endl;
}
void PathFollower::shut_down()
{
    ROS_INFO_STREAM("[FOLLOWER] shutting down...");
    clearVars();
    centre_points.clear();
    centre_splined.clear();
}


// compute linear and angular velocity commands
// can be confusing, dont mind end of lap codes at first
void PathFollower::DrivingControl()
{
	if (centre_points.size()<4)
        currentGoalPoint.updatePoint(centre_points.front());
    if (centre_points.size() <= 1) //no path points yet
        return; //to ignore rest of function
    
    double targetSpeed = V_CONST;
    double dist = getDistFromCar(currentGoalPoint);

    if (endOfLap)
    {
        ROS_INFO_STREAM("[FOLLOWER] SLOW LAP FINISHED! waiting for fast lap ready...");
        slowLapFinish = true;
    }

    // check if need to change goal pt 
    if (Lf > dist) 
        getGoalPoint();

    if (endOfPath)
    {
        if (DEBUG) std::cout<<"[FOLLOWER] end of path triggered!"<<std::endl;        
        if (plannerComplete)//
        {
            endOfLap = true;
            index = -1;
            getGoalPoint();
        }
    }
    else
        targetSpeed = V_CONST; //constant velocity for now
    if (endOfLap)
    {
        if (DEBUG) std::cout<<"[FOLLOWER] Distance to finish line: "<<getDistFromCar(centre_points.front())<<std::endl;
    }
        
    // Acceleration Control
    //this is just a P controller for now, since velocity is kept constant
    //can make this into a PID if we have varying velocity
    //in the future, targetSpeed can be changed
    double acc = KP * (targetSpeed - car_v);
	//constrain
	if (acc >= MAX_ACC)
		acc = MAX_ACC;
	else if (acc <= MAX_DECEL)
		acc =  MAX_DECEL;
    
    acceleration = acc;

    // steering control
    double alpha = getAngleFromCar(currentGoalPoint);
    double steer = atan2((2 * LENGTH * sin(alpha)),Lf);
    double targetSteer;
    if (steer >= MAX_STEER)
		targetSteer =   (MAX_STEER - 0.001); //copied from sanitise output
	else if (steer <= -(MAX_STEER))
		targetSteer =  -(MAX_STEER - 0.001);
	else
		targetSteer =  steer;

    // so there is no abrupt changes in steering
    if ((steering - targetSteer)<0)
        steering += DELTA_STEER;
    else if ((steering-targetSteer)>0)
        steering -= DELTA_STEER;
    else
        steering = targetSteer;

    // std::cout<<"acceleration: "<<acceleration<<" steering: "<<steering<<std::endl;
}

void PathFollower::updateRearPos()
{
    rearX = car_x - ((LENGTH / 2) * cos(car_yaw2));
	rearY = car_y - ((LENGTH / 2) * sin(car_yaw2));
}

// calculate distance between 2 points
double PathFollower::calcDist(const PathPoint &p1, const PathPoint &p2)
{
    double x_dist = pow(p2.x - p1.x, 2);
    double y_dist = pow(p2.y - p1.y, 2);

    return sqrt(x_dist + y_dist);
}

//calculate distance of a point to the car
double PathFollower::getDistFromCar(PathPoint& pnt) 
{
    double dX = car_x - pnt.x;
	double dY = car_y - pnt.y;
    return sqrt((dX*dX) + (dY*dY));
}

// calculate the angle of a point wrt car
double PathFollower::getAngleFromCar(PathPoint& pnt)
{
    double dX = pnt.x - rearX;
	double dY = pnt.y - rearY;
    double ang  = atan2(dY,dX) - car_yaw2;
    // double ang  = atan2(dY,dX) - car_yaw;
    if (ang > M_PI)
        ang -= 2*M_PI;
    else if (ang < -M_PI)
        ang += 2*M_PI;
    
    return ang;
}

/**********
* This Function uses tk::spline library (see spline.h)
* Path points from path planner have metres of interval, they are splined to have a smoother path
* Splining is computationally expensive, so we will not spline all the path points
* variables:
* centre_points: path points from path planner
* centre_splined: splined path points
* xp, yp, T: temporary variables for generatting splines using tk::spline
***********/
void PathFollower::generateSplines()
{
    if (endOfLap)
    return;
  
    
    //there must be at least 3 points for cubic spline to work
    if (centre_points.size() <= 2) //if less than = 2, make a line
    {
        centre_splined.clear();
        double tempX, tempY, slopeY,slopeX,stepX,stepY;
        for (auto &p:centre_points)
        {
            xp.push_back(p.x);
            yp.push_back(p.y);
        }
       
        stepY = (yp.back() - yp.front()) * STEPSIZE;
        stepX = (xp.back() - xp.front()) * STEPSIZE;       
        for (double i = 0; i<10; i++)
        {
            tempY = (i*stepY) + yp.front();
            tempX = (i*stepX) + xp.front();
            centre_splined.emplace_back(tempX,tempY);
        }
    }

    else if (centre_points.size()>SPLINE_N) //we will only spline the last N points as it is computationally expensive
    {      
        
        //separate x and y values
        int t = 0;
        for (int i = centre_points.size()-SPLINE_N; i < centre_points.size(); i++)
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
        for (double i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
        if (endOfPath && plannerComplete) std::cout<<"[FOLLOWER] Splined last sections of the track!"<< std::endl;
    }

    else //for 2 < centre points size < N 
    {
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
        for (double i = 0; i < T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
    }
       
}


/*************
* This function searches for the goal point from the splined path points 
*  (searches for the index of the goal point from centre_splined vector)
* The concept of look ahead distance of the pure puruit controller is used here
*
**/
void PathFollower::getGoalPoint()
{
    double temp; //temporary var
    double dist = 99999.1; //random large number

    //step 1: look for the point nearest to the car
    if (index == -1 || oldIndex == -1)
    {
        for (int i = 0; i < centre_splined.size(); i++)
        {
            temp = getDistFromCar(centre_splined[i]);
            if(dist < temp)
            {
                break;
            }
            else
            {
                dist = temp;
                index = i;
            }   
        }
        oldIndex = index;
    }

    else //
    {
        index = oldIndex;
        dist = getDistFromCar(centre_splined[index]); //get dist of old index

        //search for new index with least dist to car
        for(int j = index+1; j < centre_splined.size(); j++)
        {
            temp = getDistFromCar(centre_splined[j]);
            if (dist < temp)
            {
                index = j;
                break;
            }
            dist = temp;
        }
        oldIndex = index;
    }

    //look ahead distance
    Lf = LFC;
    //if velocity is not constant, we can adjust lookahead dist using the formula:
    // Lf = LFV * car_lin_v + LFC;

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
    
    if (index == centre_splined.size()-1) //if at last index of centre_splined path
    {
        if (centre_splined.size()>(5/STEPSIZE))
            endOfPath = true;
        currentGoalPoint.updatePoint(centre_splined.back());
        if (DEBUG) std::cout<<"[FOLLOWER] car near end of path" <<std::endl;
    }
    else
    {
        endOfPath = false;
        currentGoalPoint.updatePoint(centre_splined[index]); //return value
    }
    // if (DEBUG) std::cout<<"[FOLLOWER] new goal point set (" <<currentGoalPoint.x<<", "<<currentGoalPoint.y<<")"<<std::endl;
      
}

// 
double PathFollower::getSign(double &num)
{
    if (num < 0)
        return -1.0;
    else 
        return 1.0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PathFollower"); 
    ros::NodeHandle n;
       
         // Get parameters from CLI
    double max_v = atof(argv[1]);
    double max_w = atof(argv[2]);
    
    //Initialize Husky Object
    
    PathFollower follower(n,max_v, max_w);
        //ros::Rate freq(20);
	
	while (ros::ok())
    {
	    follower.spin();
        if (follower.fastLapReady)
            break;
	}
    return 0;
    
}