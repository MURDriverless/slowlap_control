#include "follower.h"
#include <iostream>

PathFollower::PathFollower(ros::NodeHandle n):nh(n)
{
    path_x.reserve(500);
    path_y.reserve(500);
    centre_points.reserve(500);
    centre_splined.reserve(2000);
    xp.reserve(20);
    yp.reserve(20);
    T.reserve(20);
    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    waitForMsgs();

    ROS_INFO_STREAM("FOLLOWER: follower initialized, pub and sub launched!");
    
}
void PathFollower::waitForMsgs()
{
    while (!odom_msg_received ||!path_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

void PathFollower::spin()
{
    waitForMsgs();
    updateRearPos();
    if (new_centre_points)
    {   generateSplines(); }
    accelerationControl();
    steeringControl();
    pushcontrol();
    pushDesiredAccel();
    pushDesiredCtrl();
    pushTarget();
    clearVars();
    
}
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

int PathFollower::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &PathFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &PathFollower::pathCallback, this);
    
}

int PathFollower::launchPublishers()
{
    pub_control = nh.advertise<mur_common::actuation_msg>(CONTROL_TOPIC, 10);
    pub_accel = nh.advertise<geometry_msgs::Accel>(ACCEL_TOPIC, 10);
    pub_steer = nh.advertise<geometry_msgs::Twist>(STEER_TOPIC, 10);
    pub_target =  nh.advertise<geometry_msgs::PoseStamped>("TargetNode", 10);
}

void PathFollower::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_v = msg.twist.twist.linear.x; //velocity
    // float q_x = msg.pose.pose.orientation.x;
    // float q_y = msg.pose.pose.orientation.y;
    float q_z = msg.pose.pose.orientation.z;
    float q_w = msg.pose.pose.orientation.w;
    yaw = 2 * asin(abs(q_z)) * getSign(q_z) * getSign(q_w);
    odom_msg_received = true;
}

void PathFollower::pathCallback(const mur_common::path_msg &msg)
{
    for (int i=0; i < msg.x.size(); i++)
    {
        path_x.push_back(msg.x[i]);
        path_y.push_back(msg.y[i]);
    }
    path_msg_received = true;
    if (centre_points.size() != path_x.size())
    {
        new_centre_points = true;
        for(int i=centre_points.size();i<path_x.size();i++)
        {
            centre_points.emplace_back(path_x[i],path_y[i]);
        }
        
    }
    
}
void PathFollower::pushcontrol()
{
    mur_common::actuation_msg ctrl_msg;

    //ctrl_msg.header.seq = 1;
   // ctrl_msg.header.frame_id = "map";
    ctrl_msg.acceleration_threshold = acceleration;
    ctrl_msg.steering = steering;

    pub_control.publish(ctrl_msg);
    std::cout<<"ctr_msg: "<<ctrl_msg<<std::endl;
}
void PathFollower::pushDesiredCtrl()
{
    geometry_msgs::Twist ctrl_desired; 
    float next_v = car_v + acceleration*DT;
    ctrl_desired.linear.x = next_v + cos(yaw);
    ctrl_desired.linear.y = next_v + sin(yaw);
    ctrl_desired.angular.z = steering;
    pub_steer.publish(ctrl_desired);
}
void PathFollower::pushDesiredAccel()
{
    geometry_msgs::Accel accel_desired;
    accel_desired.linear.x = acceleration * cos(yaw);
    accel_desired.linear.y = acceleration * sin(yaw);
    pub_accel.publish(accel_desired);
}

void PathFollower::pushTarget()
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.seq = 1;
    ps.header.stamp = current_time;
    if (centre_splined.size() == 0)
    {
        ps.pose.position.x = centre_splined.back().x;
        ps.pose.position.y = centre_splined.back().y;
    }
    else
    {
        ps.pose.position.x = centre_splined[index].x;
        ps.pose.position.y = centre_splined[index].y;
    }
    
}

void PathFollower::generateSplines()
{
    
    std::cout<<"before: points,splined = "<<centre_points.size()<<", "<<centre_splined.size()<<std::endl;
    //there must be at least 3 points for spline to work
    if (centre_points.size() == 2) //if only 2, get midpoint to make 3 pts
    {
        return;    
            // float temp1 = (centre_points.front().x + centre_points.back().x)/2;
            // float temp2 = (centre_points.front().y + centre_points.back().y)/2; 
            // PathPoint midpoint(temp1,temp2);
            // centre_splined.push_back(centre_points.front());
            // centre_splined.push_back(midpoint);
            // centre_splined.push_back(centre_points.back());
    }
    else if (centre_points.size()>10) //we will only spline 10 points as it is computationally expensive
    {
        for (int i = centre_points.size()-10; i <= 10; i++)
            {
                xp.push_back(centre_points[i].x);
                yp.push_back(centre_points[i].y);
            }
        for (float i = 0.0; i < xp.size(); i = i+1)
            {T.push_back(i);}
        std::cout<<"xp,yp size1 "<<xp.size()<<", "<<yp.size()<<std::endl;

        // Generate Spline Objects
        tk::spline sx, sy, sv;
        sx.set_points(T, xp);
        sy.set_points(T, yp);
        
        float temp = (centre_points.size() / STEPSIZE) - centre_splined.size();
        centre_splined.erase(centre_splined.begin()+temp,centre_splined.end());
        for (float i = 0; i <= T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }


    }
    else
    {
        for (int i = 0; i <centre_points.size(); i++)
            {
                xp.push_back(centre_points[i].x);
                yp.push_back(centre_points[i].y);
            }

        for (float i = 0.0; i < xp.size(); i = i+1)
            {T.push_back(i);}

        std::cout<<"xp,yp size2 "<<xp.size()<<", "<<yp.size()<<std::endl;

        // Generate Spline Objects
        tk::spline sx, sy, sv;
        sx.set_points(T, xp);
        sy.set_points(T, yp);
        centre_splined.clear();
        for (float i = 0; i <= T.size(); i += STEPSIZE)
        {
            centre_splined.emplace_back(sx(i),sy(i));
        }
    }
    std::cout<<"after: points,splined = "<<centre_points.size()<<", "<<centre_splined.size()<<std::endl;
    
       
}

void PathFollower::accelerationControl()
{
    //this is just a P controller for now, since velocity is kept constant
    //can make this into a PID if we have varying velocity
    //in the future, targetSpeed can be changed
    float targetSpeed = V_CONST;
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
    //convert position from COM frame to rear wheel frame
	PathPoint goalPt = getGoalPoint();
    std::cout<<"goal point: "<<goalPt.x<<", "<<goalPt.y<<std::endl;
    float Lfc = LFC;
    //adjust look ahead distance (use when speed is not constant)
    //Lfc = K * car_v + LFC;
    // updateRearPos();
	float alpha = atan2((goalPt.y - rearY),(goalPt.x - rearX)) - yaw;
	float steer = atan2((2 * LENGTH * sin(alpha))/Lfc,1);

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
    rearX = car_x - ((LENGTH / 2) * cos(yaw));
	rearY = car_y - ((LENGTH / 2) * sin(yaw));
}

float PathFollower::getDistFromCar(PathPoint pnt) //from rear wheels
{
    //updateRearPos();
    float dX = rearX - pnt.x;
	float dY = rearY - pnt.y;
    return sqrt(dX * dX + dY * dY);
}


//this is search target index from Dennis' code
//search for the pathpoint that is closest to the car
PathPoint PathFollower::getGoalPoint() 
{
    if (centre_splined.size() == 0)
    {
        return PathPoint(car_x,car_y);
    }

    if (index == -1 || oldIndex == -1)
    {
        float dist = 99999.1; //random large number
        float temp;
        int skip = 0;
        for (int i = centre_points.size()-1; i>0; i--)
        {
            temp = getDistFromCar(centre_points[i]);
            // std::cout<< "dist, temp = "<<dist<<", "<<temp<<std::endl;
            if(dist > temp)
            {
               dist = temp;
               skip = 0;
               index = i;
            }
            // else
            //     skip++;

            // if (skip>5)
            //     break;
        }
        oldIndex = index;
        std::cout<<"index: "<<index<<std::endl;
    }
    else
    {
        index = oldIndex;
        float dis1 = getDistFromCar(centre_points[index]);
        for(int j = index; j < centre_points.size(); j++)
        {
            if (dis1 > getDistFromCar(centre_points[j]))
            {
                index = j;
                break;
            }
        }
        oldIndex = index;
        std::cout<<"index: "<<index<<std::endl;
    }
    return PathPoint(centre_points[index]);
    

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