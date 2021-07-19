#include "follower.h"


PathFollower::PathFollower(ros::NodeHandle n):nh(n)
{
    centre_points.reserve(500);
    centre_splined.reserve(1000);
    xp.reserve(20);
    yp.reserve(20);
    T.reserve(20);

    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    
}

void PathFollower::spin()
{
    ros::spinOnce();
    accelerationControl();
    steeringControl();

}

int PathFollower::launchSubscribers()
{
    try
    {
	sub_odom = nh.subscribe(ODOM_TOPIC, 1, &PathFollower::odomCallback, this);
	sub_path = nh.subscribe(PATH_TOPIC, 1, &PathFollower::pathCallback, this);
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);
	return 0;
    }
    ROS_INFO_STREAM("FOLLOWER: Odometry and Path subscribers connect");
    return 1;
}

int PathFollower::launchPublishers()
{
    try
    {
        pub_control = nh.advertise<mur_common::actuation_msg>(CONTROL_TOPIC, 10);//new
    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
        return 0;
    }

    ROS_INFO_STREAM("FOLLOWER: actuation control publisher connected");
    return 1;
}

void PathFollower::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_v = msg.twist.twist.linear.x; //velocity
    odom_msg_received = true;
}

void PathFollower::pathCallback(const mur_common::path_msg &msg)
{
    path_x = msg.x;
    path_y = msg.y;
    path_msg_received = true;
    if (centre_points.size() != path_x.size())
    {
        new_centre_points = true;
        for(int i=centre_points.size();i<path_x.size();i++)
        {
            centre_points.emplace_back(path_x[i],path_y[i]);
        }
        generateSplines();
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
}
void PathFollower::generateSplines()
{
    
    //there must be at least 3 points for spline to work
    if (centre_points.size() == 2) //if only 2, get midpoint to make 3 pts
    {
        float temp1 = (centre_points[0].x + centre_points[1].x)/2;
        float temp2 = (centre_points[0].y + centre_points[1].y)/2; 
        PathPoint midpoint(temp1,temp2);
        centre_points.insert(centre_points.begin()+1,midpoint);
    }

    if (centre_points.size()>10) //we will only spline 10 points as it is computationally expensive
    {
        for (int i = centre_points.size()-10; i <= 10; i++)
            {
                xp.push_back(centre_points[i].x);
                yp.push_back(centre_points[i].y);
            }
        for (int i = 0; i < xp.size(); i++)
            T[i] = i;

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
        xp.assign(centre_points.begin()->x,centre_points.end()->x);
        yp.assign(centre_points.begin()->y,centre_points.end()->y);
        T.assign(xp.size(), 0);

        for (int i = 0; i < xp.size(); i++)
            T[i] = i;

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
float PathFollower::getDistFromCar(PathPoint &pnt) //from rear wheels
{
    float rearX = car_x - ((LENGTH / 2) * cos(yaw));
	float rearY = car_y - ((LENGTH / 2) * sin(yaw));
    float dX = rearX - pnt.x;
	float dY = rearY - pnt.y;
    return sqrt(dX * dX + dY * dY);
}

//this search target index from Dennis' code
PathPoint PathFollower::getGoalPoint()
{
    if (index == -1)
    {
        float dist = 99999.1; //random large number
        float temp;
        int skip = 0;
        for (int i = centre_splined.size()-1; i<0; i--)
        {
            temp = getDistFromCar(centre_splined[i]);
            if(dist<temp)
            {
               dist = temp;
            }
            else
                skip++;

            if (skip>5)
                break;
        }
        index = i+1;
        oldIndex = index;
    }
    else
    {
        index = oldIndex;
        float dis1 = getDistFromCar(centre_splined[index]);

        for(int j = index; j < centre_splined.size(); j++)
        {
            if (dis1 > getDistFromCar(centre_splined[j]))
            {
                break;
            }
        }
        index = j-1;
        oldIndex = index;
    }
    return PathPoint(centre_splined[index]);
    

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathFollower"); 
    ros::NodeHandle n;

    if (ros::ok())
    {
	    PathFollower follower(n);
	    while (ros::ok())
	    {
	        follower.spin();
	    }
    }
}