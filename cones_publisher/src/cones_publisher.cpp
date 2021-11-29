/**
 * This publishes cone positions
 * by Aldrei Recamadas (MURauto21)
 *
 * **/


#include "cones_publisher.h"

ConesPublisher::ConesPublisher(ros::NodeHandle n) :nh(n)
{
    //specify minimum size of vectors
    true_cones.reserve(300);
    seen_cones.reserve(300);
    cones_list.reserve(300);

    if (ros::ok())
    {
        launchSubscribers();
        launchPublishers();
    }
    
    waitForMsgs();
    ROS_INFO_STREAM("CONES PUBLISHER: initialized!");
}

void ConesPublisher::waitForMsgs()
{
    if (!trueCones_msg_received || !odom_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

void ConesPublisher::spin()
{
    waitForMsgs();
    detectCones();
    makeUncertain(); //uncomment this to simulate uncertainty
    publishCones();
    clearTemps();
    ros::Rate(HZ).sleep();
}

void ConesPublisher::clearTemps()
{
    trueCones_msg_received = false;
    odom_msg_received = false;
}


int ConesPublisher::launchSubscribers()
{
    sub_odom = nh.subscribe(ODOM_TOPIC, 1, &ConesPublisher::odomCallback, this);
	sub_cones = nh.subscribe(TRUE_CONES, 1, &ConesPublisher::trueConesCallback, this);
    
}

int ConesPublisher::launchPublishers()
{
    pub_cones = nh.advertise<mur_common::cone_msg>(CONE_TOPIC, 1);
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>(RVIZ_CONES, 1);
}

// get odometry messages
// convert Quat to Euler yaw 
void ConesPublisher::odomCallback(const nav_msgs::Odometry &msg)
{
    car_pose.x = msg.pose.pose.position.x;
    car_pose.y = msg.pose.pose.position.y;
    
    double q_x = msg.pose.pose.orientation.x;
    double q_y = msg.pose.pose.orientation.y;
    double q_z = msg.pose.pose.orientation.z;
    double q_w = msg.pose.pose.orientation.w;

   
    tf::Quaternion q(q_x, q_y, q_z, q_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    car_yaw = yaw;
    odom_msg_received = true;
}

// get path messages
void ConesPublisher::trueConesCallback(const mur_common::cone_msg &msg)
{
    
    if (msg.x.size() != 0);
    { 
        true_cones.clear();
        for (int i = 0; i < msg.x.size(); i++)
        {
            if (msg.colour[i] == "BLUE")
            {
            true_cones.push_back(Cone(msg.x[i], msg.y[i], 'b', i));
            }
            else if (msg.colour[i] == "YELLOW")
            {
                true_cones.push_back(Cone(msg.x[i], msg.y[i], 'y', i)); 
            }
            else if (msg.colour[i] == "na")
            {
                std::cout << "PLANNER: 'na' cone colour passed, skipping" << std::endl;
            }
            else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
            {
                true_cones.push_back(Cone(msg.x[i], msg.y[i], 'r', i));
            }

            //off set pos for EUFS track
            if (EUFS)
            {
                true_cones.back().position.x -= -13.0;
                true_cones.back().position.y -= 10.3;
            }
        }
        
    trueCones_msg_received = true;
    }

}


void ConesPublisher::publishCones()
{
    ros::Time current_time = ros::Time::now();
    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_msg.markers.resize(seen_cones.size()); ///
    mur_common::cone_msg cones;
    cones.header.frame_id = FRAME;
    cones.header.stamp = current_time;
    int i = 0;
    if (DEBUG) std::cout<<"seen cones: ";
    for (auto &cn:seen_cones) ////
    {
        cones.x.push_back(cn.uncertainPos.x);
        cones.y.push_back(cn.uncertainPos.y);
        if (cn.colour == 'b')
            cones.colour.push_back("BLUE");
        else if(cn.colour == 'y')
            cones.colour.push_back("YELLOW");
        else if(cn.colour == 'r')
            cones.colour.push_back("ORANGE");
        
        // for RVIZ markers
        setMarkerProperties(&marker_array_msg.markers[i],cn.uncertainPos,i,cones.colour.back(),FRAME);
        i++;
        if(DEBUG) std::cout<<"("<<cn.position.x<<", "<<cn.position.y<<") ";
    }
    if (DEBUG) std::cout<<"\n"<<std::endl;
    
    pub_cones.publish(cones);
    markers_pub.publish(marker_array_msg);
}

// for RVIZ markers
// fills in the marker property for each element in Marker array
void ConesPublisher::setMarkerProperties(visualization_msgs::Marker *marker,PathPoint pos,
                                            int n,std::string colour,std::string frame_id)
{
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
    marker->ns = "my_namespace";
    marker->id = n;
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;

    marker->pose.position.x = pos.x;
    marker->pose.position.y = pos.y;
    marker->pose.position.z = pos.z;

    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    marker->scale.x = 0.3;
    marker->scale.y = 0.3;
    marker->scale.z = 0.7;

    // alpha and RGB settings
    marker->color.a = 1;

    if (colour == "BLUE")
    {
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 1.0;
    }
    else if (colour == "ORANGE")
    {
        marker->color.r = 1.0;
        marker->color.g = 0.4;
        marker->color.b = 0.0;
    }
    else if (colour == "YELLOW")
    {
        marker->color.r = 1.0;
        marker->color.g = 1.0;
        marker->color.b = 0.0;
    }
    else // UNKNOWN_STR
    {
        marker->color.r = 1.0;
        marker->color.g = 1.0;
        marker->color.b = 1.0;
    }

    marker->lifetime = ros::Duration(0);
}

double ConesPublisher::getDistFromCar(PathPoint& pnt) 
{
    double dX = car_pose.x - pnt.x;
	double dY = car_pose.y - pnt.y;
    return sqrt((dX*dX) + (dY*dY));
}
double ConesPublisher::getAngleFromCar(PathPoint& pnt)
{
    double dX = pnt.x - car_pose.x;
	double dY = pnt.y - car_pose.y;
    double ang  = atan2(dY,dX) - car_yaw;
    if (ang > M_PI)
        ang -= 2*M_PI;
    else if (ang < -M_PI)
        ang += 2*M_PI;
    
    return ang*180/M_PI;
}

//to detect cones within sensor range
void ConesPublisher::detectCones()
{
    double dist,angle;

    for (int i= 0; i<true_cones.size(); i++)
    {
        dist = getDistFromCar(true_cones[i].position);
        if (dist <= SENSOR_RANGE)
        {
            angle = getAngleFromCar(true_cones[i].position);
            if (abs(angle)<90)//(0-180)
            {
                bool added = false;
                for (auto id:cones_list)
                {
                    if (id == i)
                        added=true;
                }
                if (!added)
                {
                    seen_cones.push_back(true_cones[i]);
                    seen_cones.back().uncertainPos = seen_cones.back().position;
                    cones_list.push_back(i);
                }
            }
        }
    }

    
}
void ConesPublisher::makeUncertain()
{
        // varying seen cone pos to simulate uncertainty
    float randNum;
    float dist;
    for (auto &con:seen_cones)
    {
        dist = getDistFromCar(con.position);
        if (dist>CERTAIN_RANGE)
        {
            if(!con.passedBy)
            {
                randNum = ((float)rand()/RAND_MAX)-0.5; //this will give rand num from -0.5 to 0.5
                con.uncertainPos.x = con.position.x+(randNum*(dist/SENSOR_RANGE));
                
                randNum = ((float)rand()/RAND_MAX)-0.5; //this will give rand num from -0.5 to 0.5
                con.uncertainPos.y = con.position.y+(randNum*(dist/SENSOR_RANGE));
            }
        }
        else
            con.passedBy = true;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ConesPublisher"); 
    ros::NodeHandle n;
       
    //Initialize Husky Object
    
    ConesPublisher conesPub(n);
        //ros::Rate freq(20);
	
	while (ros::ok())
    {
	    conesPub.spin();
        if (conesPub.last_cone)
            break;
	}
    return 0;
    
}
