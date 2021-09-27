/**
 * This handles all the ROS stuff like subscribing and publishing msgs
 * some of these are copied from Joseph (MURauto20) and modified a bit 
 * see header file for description of member variables
 * 
 * author: Aldrei (MURauto21)
**/


#include "node.h"

// constructor
PlannerNode::PlannerNode(ros::NodeHandle n, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : nh(n), const_velocity(const_velocity), v_max(v_max), v_const(v_const), max_f_gain(max_f_gain)
{
    Path.reserve(300);
    Left.reserve(200);
    Right.reserve(200);
    cones.reserve(500);
    Markers.reserve(1000); 
    // sortMarks.reserve(100);
    
    times.reserve(std::numeric_limits<uint16_t>::max());    // diagnostic stuff (MURauto20)
    rtimes.reserve(std::numeric_limits<uint16_t>::max());   // diagnostic stuff (MURauto20)
    
    launchSubscribers();
    launchPublishers();
    waitForMsgs();
    initialisePlanner();
    now = ros::Time::now();
}

// initialises planner (path_planner.cpp), waits for cones to be received (especially orange cones)
void PlannerNode::initialisePlanner()
{
    if (cone_msg_received)
    {
        int countRed = 0;
        for (auto &cn: cones)
        {
            if (cn.colour == 'r')
                countRed++;
        }
        if (countRed>1)
        {
            this->planner = std::unique_ptr<PathPlanner>(new PathPlanner(car_x, car_y, cones, const_velocity, v_max, v_const, max_f_gain, Markers));
            ROS_INFO_STREAM("[PLANNER] Planner initialized");
            plannerInitialised = true;
        }
        else
            std::cout<<"Timing cones (orange) not yet found"<<std::endl;

    }
    else
        std::cout<<"No cones received yet"<<std::endl;

}

// spinOnce when msgs are received
void PlannerNode::waitForMsgs()
{
    while (!cone_msg_received || !odom_msg_received && ros::ok()) 
    {
	ros::spinOnce();
    }
}

// standard ROS function. launch subscribers
int PlannerNode::launchSubscribers()
{
    try
    {
	sub_odom = nh.subscribe(MUR_ODOM_TOPIC, 1, &PlannerNode::odomCallback, this);
	sub_cones = nh.subscribe(CONE_TOPIC, 1, &PlannerNode::coneCallback, this);
    sub_transition = nh.subscribe(FASTLAP_READY_TOPIC, 1, &PlannerNode::transitionCallback, this);
    }
    catch (const char *msg)
    {
	ROS_ERROR_STREAM(msg);
	return 0;
    }
    ROS_INFO_STREAM("[PLANNER] Odometry and cone subscribers connect");
    return 1;
}

// standard ROS function. launch publishers
int PlannerNode::launchPublishers()
{
    try
    {
        pub_path = nh.advertise<mur_common::path_msg>(PATH_TOPIC, 1);
        pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
        pub_health = nh.advertise<mur_common::diagnostic_msg>(HEALTH_TOPIC, 1);
        pub_lcones = nh.advertise<mur_common::cone_msg>(SORTED_LCONES_TOPIC, 1);
        pub_rcones = nh.advertise<mur_common::cone_msg>(SORTED_RCONES_TOPIC, 1);
        pub_pathCones = nh.advertise<visualization_msgs::MarkerArray>(PATH_CONES_TOPIC,1);
        pub_map = nh.advertise<mur_common::map_msg>(FINISHED_MAP_TOPIC,1);
        // pub_sorting_markers = nh.advertise<visualization_msgs::MarkerArray>(SORTING_MARKER,1);
        // pub_path_marks =  nh.advertise<visualization_msgs::MarkerArray>(PATH_MARKER,1);

    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
        return 0;
    }

    ROS_INFO_STREAM("[PLANNER] Path, visualisation, diagnostic publishers connected");
    return 1;
}

// these are the last steps when slow lap is finished mapping
// transition to fast lap
void PlannerNode::SlowLapFinished()
{
    ROS_INFO_STREAM("[PLANNER] SLOW LAP FINISHED!! [PLANNER] Publishing path points and cone positions...");
    
    // publish complete map:
    mur_common::map_msg map;
    std::vector<float> ConeX,ConeY;
    //copy left cones
    ConeX.reserve(Left.size());
    ConeY.reserve(Left.size());
    for (auto &cn:Left)
    {
        ConeX.push_back(cn.position.x);
        ConeY.push_back(cn.position.y);
    }
    map.x_o = ConeX;
    map.y_o = ConeY;
    ConeX.clear();
    ConeY.clear();
    //copy right cones
    ConeX.reserve(Right.size());
    ConeY.reserve(Right.size());
    for (auto &cn:Right)
    {
        ConeX.push_back(cn.position.x);
        ConeY.push_back(cn.position.y);
    }
    map.x_i = ConeX;
    map.y_i = ConeY;

    //copy path points
    for (auto &p:Path)
    {
        map.x.push_back(p.x);
        map.y.push_back(p.y);
    }

    map.mapready = true;
    map.frame_id = FRAME;

    pub_map.publish(map);
}

// shut down
void PlannerNode::shut_down()
{
    ROS_INFO_STREAM("[PLANNER] shutting down...");
    clearTempVectors();
    pushPath();
    // pushPathViz();
    pushMarkers();     
}

// void loop()
void PlannerNode::spinThread()
{
    clearTempVectors();
    waitForMsgs();
    if (plannerInitialised)
    {
        planner->update(cones, car_x, car_y, Path, Left, Right,Markers,plannerComplete);
        if (plannerComplete)
            SlowLapFinished();
        
        if(fastLapReady)
            shut_down();
        else
        {
            pushPath();
            pushPathViz();
            pushMarkers();
        }        
    }
    else
        initialisePlanner();
        
   
    ros::Rate(HZ).sleep();
}

// diagnostic stuff (MURauto20)
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

//clear temporary vectors, and reset some flags
void PlannerNode::clearTempVectors()
{
    Path.clear();
    Markers.clear();
    // sortMarks.clear();
    Left.clear();
    Right.clear();
    cones.clear();
    cone_msg_received = false;
    odom_msg_received = false;
}

// publish path for rviz
void PlannerNode::pushPathViz()
{
    nav_msgs::Path path_viz_msg;
    path_viz_msg.header.frame_id = FRAME; //"map"

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(Path.size());

    for (int p = 0; p < Path.size(); p++)
    {
        geometry_msgs::PoseStamped item; 
        item.header.frame_id = FRAME;
        item.header.seq = p;
        item.pose.position.x = Path[p].x;
        item.pose.position.y = Path[p].y;
        item.pose.position.z = 0.0;

        poses.emplace_back(item);
    }

    path_viz_msg.poses = poses;
    pub_path_viz.publish(path_viz_msg);

    // visualization_msgs::MarkerArray pathMarks;
    // pathMarks.markers.resize(Path.size());
    // for (int i=0;i<Path.size();i++)
    // {
    //     setMarkerProperties2(&pathMarks.markers[i],Path[i],i,i,'p');
    // }
    // pub_path_marks.publish(pathMarks);
}

// publish path points for path follower
void PlannerNode::pushPath()
{
    mur_common::path_msg msg;
    msg.header.frame_id = FRAME;
    for (auto &p:Path)
    {
        msg.x.push_back(p.x);
        msg.y.push_back(p.y);
        msg.v.push_back(p.velocity);
    }
    pub_path.publish(msg);
}

// get transition flag from fast lap
void PlannerNode::transitionCallback(const mur_common::transition_msg &msg)
{
    fastLapReady = msg.fastlapready;
}

// get odometry messages (from SLAM)
void PlannerNode::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_v = msg.twist.twist.linear.x; 
    odom_msg_received = true;
}

// get cone positions (from SLAM)
void PlannerNode::coneCallback(const mur_common::cone_msg &msg)
{
    if (msg.x.size() == 0)
        cone_msg_received = false;
    else
    {
        for (int i = 0; i < msg.x.size(); i++)
        {
            if (msg.colour[i] == "BLUE")
            {
            cones.push_back(Cone(msg.x[i], msg.y[i], 'b', i));
            }
            else if (msg.colour[i] == "YELLOW")
            {
                cones.push_back(Cone(msg.x[i], msg.y[i], 'y', i)); 
            }
            else if (msg.colour[i] == "na")
            {
                std::cout << "[PLANNER] 'na' cone colour passed, skipping" << std::endl;
            }
            else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
            {
                cones.push_back(Cone(msg.x[i], msg.y[i], 'r', i));
            }
        }
        cone_msg_received = true;
    }
}

// pablish markers to rviz
void PlannerNode::pushMarkers()
{
    visualization_msgs::MarkerArray marks;
    marks.markers.resize(Markers.size()/2);
    int j,k;
    for (int i=0; i<marks.markers.size(); i++)
    {
        j=2*i;
        setMarkerProperties(&marks.markers[i],Markers[j],Markers[j+1],i,Markers[i].accepted);
    }
    pub_pathCones.publish(marks);


    // j=0;
    // for (auto &c:Left)
    // {
    //     if(!c.passedBy)
    //     {
    //         sortMarks.push_back(c.position);
    //         j++;
    //     }
    // }
    // for (auto &c:Right)
    // {
    //     if(!c.passedBy)
    //     {
    //         sortMarks.push_back(c.position);
    //     }
    // }
    // visualization_msgs::MarkerArray sortingMarks;
    // sortingMarks.markers.resize(sortMarks.size());
    // k=1;
    // char col='b';
    // for(int i=0;i<sortMarks.size();i++)
    // {
    //     if (i==j)
    //     {
    //         k = 1;
    //         col = 'y';
    //     }
            
    //     setMarkerProperties2(&sortingMarks.markers[i],sortMarks[i],i,k,col);
    //     k++;      
    // }
    // pub_sorting_markers.publish(sortingMarks);
}

// marker properties
// google Visualisation::Markers message for more details
void PlannerNode::setMarkerProperties(visualization_msgs::Marker *marker,PathPoint cone1,PathPoint cone2,int n,bool accepted)
{
    marker->header.frame_id = FRAME;
    marker->header.stamp = ros::Time();
    marker->header.seq = n;
    marker->ns = "my_namespace";
    marker->id = n;
    marker->type = visualization_msgs::Marker::LINE_LIST;
    marker->action = visualization_msgs::Marker::ADD;
    marker->lifetime = ros::Duration(1);
    geometry_msgs::Point p1, p2;


    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = 0.1;
    p1.x = cone1.x;
    p1.y = cone1.y;
    p1.z = 0;
    p2.x = cone2.x;
    p2.y = cone2.y;
    p2.z = 0;
    marker->points.push_back(p1);
    marker->points.push_back(p2);

    marker->scale.x = 0.3;
    marker->scale.y = 0.3;
    marker->scale.z = 0.7;

    // alpha and RGB settings
    // color.a is opacity, 0: invisible
    marker->color.a = 0.5;

    if (accepted)
    {
        marker->color.r = 1.0;
        marker->color.g = 0.0;
        marker->color.b = 0.0;
    }
    else
    {
        marker->color.a = 1.0;
        marker->color.r = 0.50;
        marker->color.g = 0.1;
        marker->color.b = 1.0;
        marker->lifetime = ros::Duration(0.1);
    }
    if (plannerComplete)
    marker->color.a = 0.0;
        
}

