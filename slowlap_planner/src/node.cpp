
#include "node.h"


/**
 * This handles all the ROS stuff like subscribing and publishing msgs
 * most of these are copied from (Joseph 2020) and modified a bit 
 * 
**/

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

    ROS_INFO_STREAM("PLANNER: Planner initialized");

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
    ROS_INFO_STREAM("PLANNER: Odometry and cone subscribers connect");
    return 1;
}

int PlannerNode::launchPublishers()
{
    try
    {
        pub_path = nh.advertise<mur_common::path_msg>(PATH_TOPIC, 1);
        pub_path_viz = nh.advertise<nav_msgs::Path>(PATH_VIZ_TOPIC, 1);
        pub_health = nh.advertise<mur_common::diagnostic_msg>(HEALTH_TOPIC, 1);
        pub_lcones = nh.advertise<mur_common::cone_msg>(SORTED_LCONES_TOPIC, 1);
        pub_rcones = nh.advertise<mur_common::cone_msg>(SORTED_RCONES_TOPIC, 1);

    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
        return 0;
    }

    ROS_INFO_STREAM("PLANNER: Path, visualisation, diagnostic publishers connected");
    return 1;
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
    auto rend = Clock::now();
    pushHealth(start, end, rstart, rend);
    int siz = X.size();
  
}

// diagnostic stuff (2020)
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
    cone_msg_received = false;
    odom_msg_received = false;
}

// publish path or rviz
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

//publish sorted cones
void PlannerNode::pushSortedCones()
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

// publish path points
void PlannerNode::pushPath()
{
    mur_common::path_msg msg;
    msg.header.frame_id = "map";
    msg.x = X;
    msg.y = Y;
    msg.v = V;
    pub_path.publish(msg);
}

// get odometry messages
void PlannerNode::odomCallback(const nav_msgs::Odometry &msg)
{
    car_x = msg.pose.pose.position.x;
    car_y = msg.pose.pose.position.y;
    car_v = msg.twist.twist.linear.x; 
    odom_msg_received = true;
}

// get cone positions
void PlannerNode::coneCallback(const mur_common::cone_msg &msg)
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
	        std::cout << "PLANNER: 'na' cone colour passed, skipping" << std::endl;
	    }
	    else if (msg.colour[i] == "BIG" || msg.colour[i] == "ORANGE")
	    {
	        cones.push_back(Cone(msg.x[i], msg.y[i], 'r', i));
	    }
    }
    cone_msg_received = true;
}

