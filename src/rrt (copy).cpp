// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

#include <nav_msgs/Path.h>
std::vector<float> distant;
//std::vector<Node> tree;
double angle;
//double yaw;
int ctn = 0;

bool updated = true;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
	delete[] &submap;
	//delete[] &tree;
    /*for(int i=0; i<submap.size(); i++)
    {
	delete submap;
    }*/
	
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TO_DO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    

    std::string pose_topic, scan_topic;
    nh_.getParam("/rrt_node/pose_topic", pose_topic);
    nh_.getParam("/rrt_node/scan_topic", scan_topic);
    std::string drive_topic;
    nh_.getParam("/rrt_node/drive_topic", drive_topic);

    nh_.getParam("/rrt_node/threshold", threshold_);
    nh_.getParam("/rrt_node/near_threshold_", near_threshold_);
    nh_.getParam("/rrt_node/max_iteration", interation_);
    nh_.getParam("/rrt_node/max_expansion_dist", max_expansion_dist_);
    nh_.getParam("/rrt_node/LOOK_AHEAD", LOOK_AHEAD);
    nh_.getParam("/rrt_node/VELOCITY", velocity);
    
    nh_.getParam("/rrt_node/map_sizex_", map_sizex_);
    nh_.getParam("/rrt_node/map_sizey_", map_sizey_);
    nh_.getParam("/rrt_node/map_resolution_", map_resolution_);
    nh_.getParam("/rrt_node/map_origin_x_", map_origin_x_);
    nh_.getParam("/rrt_node/map_origin_y_", map_origin_y_);

    // ROS publishers
    // TO_DO: create publishers for the the drive topic, and other topics you might need
    tree_pub_ = nh_.advertise<sensor_msgs::PointCloud>("rrt_tree", 1);
    plan_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path", 1);
    nav_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);

    chosen_point_pub = nh_.advertise<visualization_msgs::Marker>("/chosen_point", 0);
    tree_node_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_node_point", 0);
    free_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/free_point", 0);

    // ROS subscribers
    // TO_DO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    //map_sub_ = nh.subscribe("map", 1, boost::bind(&RRT::makemap, this, 1));
    tf::TransformListener listener(ros::Duration(10));

    //publish visualization

	chosen_point.header.frame_id = "map";
	chosen_point.ns = "waypoint_vis";
	chosen_point.type = visualization_msgs::Marker::SPHERE;
	chosen_point.action = visualization_msgs::Marker::ADD;
	chosen_point.pose.position.z = 0;
	chosen_point.pose.orientation.x = 0.0;	
	chosen_point.pose.orientation.y = 0.0;	
	chosen_point.pose.orientation.z = 0.0;	
	chosen_point.pose.orientation.w = 1.0;	
	chosen_point.scale.x = 0.2;	
	chosen_point.scale.y = 0.2;	
	chosen_point.scale.z = 0.2;	
	chosen_point.color.a = 1.0;	
	chosen_point.color.r = 1.0;	
	chosen_point.color.g = 0.0;
	chosen_point.color.b = 0.0;

     //visualization sampled points
	tree_node_point.header.frame_id = "map";
	tree_node_point.ns = "tree_node_point_vis";
	tree_node_point.type = visualization_msgs::Marker::POINTS;//SPHERE;
	tree_node_point.action = visualization_msgs::Marker::ADD;
	//tree_node_point.pose.position.z = 0;
	//tree_node_point.pose.orientation.x = 0.0;	
	//tree_node_point.pose.orientation.y = 0.0;	
	//tree_node_point.pose.orientation.z = 0.0;	
	tree_node_point.pose.orientation.w = 1.0;	
	tree_node_point.scale.x = 0.1;	
	tree_node_point.scale.y = 0.1;	
	tree_node_point.scale.z = 0.1;	
	tree_node_point.color.a = 1.0;	
	tree_node_point.color.r = 0.0;	
	tree_node_point.color.g = 0.0;
	tree_node_point.color.b = 1.0;

     //visualization sampled points
	free_point.header.frame_id = "map";
	free_point.ns = "free_point_vis";
	free_point.type = visualization_msgs::Marker::POINTS;//SPHERE;
	free_point.action = visualization_msgs::Marker::ADD;
	//tree_node_point.pose.position.z = 0;
	//tree_node_point.pose.orientation.x = 0.0;	
	//tree_node_point.pose.orientation.y = 0.0;	
	//tree_node_point.pose.orientation.z = 0.0;	
	free_point.pose.orientation.w = 1.0;	
	free_point.scale.x = 0.1;	
	free_point.scale.y = 0.1;	
	free_point.scale.z = 0.1;	
	free_point.color.a = 1.0;	
	free_point.color.r = 1.0;	
	free_point.color.g = 0.0;
	free_point.color.b = 0.0;
    
    freespace_.clear();
    root_.parent = 0;
    root_.children.clear();
    //root_ = new Node;

    // TO_DO: create a occupancy grid
    //read form map, global for now
    std::ifstream fin("/home/zan/zhenzan_ws/src/rrt/src/map.csv");
    std::string line;
    while(getline(fin, line)){
	std::istringstream sin(line);
	std::vector<float> row;
	std::string wp;
	while(getline(sin, wp, ',')){
	    if(atof(wp.c_str()) <= 0.86){//if it is wall
		row.push_back(0.0);
	    }else{
		row.push_back(1.0);
	    }
	}
	map_global_.push_back(row);
	
    }

    //read waypoint
    std::ifstream fin2("/home/zan/zhenzan_ws/src/rrt/src/test_1.csv");
    std::string line2;
    while(getline(fin2, line2)){
	std::istringstream sin(line2);
	std::vector<float> row2;
	std::string wp2;
	while(getline(sin, wp2, ',')){
	    row2.push_back(atof(wp2.c_str()));
	}
	waypoint.push_back(row2);
	
    }
    
    //get
    ROS_INFO("the size of map is %lu", map_global_.size());

    ROS_INFO("Created new RRT Object.");
}



void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    // TO_DO: update your occupancy grid
    double min, max;//, angle
    int sample_time;
    min = scan_msg -> angle_min;
    max = scan_msg -> angle_max;
    angle = scan_msg -> angle_increment;
    sample_time = int((max-min)/angle);
    //std::vector<float> distant;
    distant.clear();
    distant = scan_msg -> ranges;

/*
    std::vector<tf::Vector3> finally;
    tf::Vector3 temp;
    for(int i=0; i<distant.size(); i++){
	//
	//geometry_msgs::Vector3Stamped scan_distant;
	//scan_distant.vector.x = cos(i*angle) * distant[i];//+-?
	//scan_distant.vector.y = sin(i*angle) * distant[i];
	//scan_distant.vector.z = 0;
	//scan_distant.header.stamp = ros::Time();
	//scan_distant.header.frame_id = "laser";
	//tf::Stamped<tf::Vector3> global_distant;
	//geometry_msgs::Vector3Stamped global_distant;
	tf::Stamped<tf::Vector3> scan_distant(tf::Vector3(cos(i*angle) * distant[i], sin(i*angle) * distant[i], 0.0), ros::Time(), "laser");

	float x,y,z;
	const std::string base_link = "base_link";
	const std::string laser = "laser";

	tf::Stamped<tf::Vector3> base_link_distant;
	tf::Stamped<tf::Vector3> map_distant;

	tf::Transformer::transformPoint(base_link, ros::Time::now(), scan_distant, laser, base_link_distant); 
	tf::Transformer::transformPoint("map", ros::Time::now(), base_link_distant, base_link, map_distant); 
        //temp.x = x;
	//temp.y = y;
	//temp.z = z;
	finally.push_back(tf::Vector3(x,y,z));
    }

    //update occupancy grid?
    ROS_INFO("Donot update for now.");
    //tf::Stamped<tf::Vector3> global_distant;
    //tf::Transformer::transformPoint("map", ros::Time::now(), scan_distant, "laser", global_distant); 

*/

    ROS_INFO("yaw %f", yaw);
    //transform it to the origin submap coordinate
    Eigen::Matrix2f R;
    Eigen::Vector2f scan_;
    Eigen::Vector2f sub_co;

    R << cos(-yaw), -sin(-yaw), sin(-yaw), cos(-yaw);
    
    std::vector<Eigen::Vector2f> new_dist;
    for(int i=0; i<distant.size(); i++){
	scan_ << cos(i*angle) * (distant[i]+0.3), sin(i*angle) * (distant[i]+0.3);
        sub_co = R.inverse()*scan_;

	new_dist.push_back(sub_co);
    }

    ROS_INFO("%f,%f", cos(0) * distant[0], sin(0) * distant[0]);
    ROS_INFO("%f,%f", new_dist[0](0),new_dist[0](1));

int xx =0;
int yy = 0;

    float map_x, map_y;
    map_x = (abs(map_origin_x_) + curr_.x)/map_resolution_;
    map_y = (abs(map_origin_y_) - curr_.y)/map_resolution_;
    submap.clear();
    for(int i=0; i<120; i++){
        std::vector<float> row3;
	for(int j=0; j<120; j++){
	    row3.push_back(map_global_[map_y-60+j][map_x-60+i]);
		//if 0,1
	    //ROS_INFO("point %d, %d is %f", i, j, map_global_[map_y-60+j][map_x-60+i]);
	}
	submap.push_back(row3);
    }



    for(int j=0; j< new_dist.size(); j++){
	int row = 60-new_dist[j](1)/map_resolution_;
	int col = 60+new_dist[j](0)/map_resolution_;
	
	//update submap
	//mn = 8, 4-115  -4
	//mn = 4, 2-118  -2
	//
	if((row <= 115 && row >= 4) && (col >= 4 && col <= 115)){
		for(int m=0;m<4;m++){
			for(int n=0;n<8;n++){
				//ROS_INFO("row col %d,%d,%d,%d", row,119-(row-2+n), col, 119-(col-2+m));
				if((119-(col-10+m) <= 119 && 119-(col-10+m) >= 0) && (119-(row-10+n) >= 0 && 119-(row-10+n) <= 119)){
				submap[119-(col-2+m)][119-(row-2+n)] = 0.0;
				//submap[row-2+m][col-2+n] = 0.0;
				xx = row;
				yy = col;
				//ROS_INFO("xx, yy %d, %d", xx, yy);
				}
			}
		}
		
	}
    }
	ROS_INFO("updated the laser scan");
	updated = true;
	
}
/*
void transformPoint(const tf::TransformListener& listener){
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "laser";
    laser_point.header.stamp = ros::Time();

    for(int i=0; i<distant.size(); i++){
	laser_point.point.x = cos(i*angle) * distant[i];
	laser_point.point.y = sin(i*angle) * distant[i];
	laser_point.point.z = 0.0;
	geometry_msgs::PointStamped odom_point;
	listener.transformPoint("odom", laser_point, odom_point);
	ROS_INFO("the transformed point %f, %f, %f", odom_point.point.x, odom_point.point.y, odom_point.point.z);
    }
}
*/
//void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;

    // TO_DO: fill in the RRT main loop
    curr_.x = pose_msg->pose.pose.position.x;
    curr_.y = pose_msg->pose.pose.position.y;
    root_.x = curr_.x;
    root_.y = curr_.y;
    //rrt_nodes_.push_back(root_);


///////////////////////////////////////////////////////////////////////////////////////

    //find the goal point in wawpoint
    //std::vector<float> best_point = getBestPoint(curr_.x, curr_.y, LOOK_AHEAD, waypoint, chosen_point, chosen_point_pub);
    //std::vector<float> best_point;
    float curr_x = curr_.x;
    float curr_y = curr_.y;
    

    int min_index;
	float min_dist = 10000000;

	for(int i=0; i<waypoint.size();i++){
		int dist = pow((curr_x-waypoint[i][0]), 2) + pow((curr_y-waypoint[i][1]), 2);
			if(dist < min_dist){
				min_index = i;
				min_dist = dist;
			}
		}

	// ROS_INFO("the closest point is: %i, with x:%f, y:%f", min_index,waypoint[min_index][0],waypoint[min_index][1]);

	//find the most look ahead point
	float x, y;
	int search = 0;
	bool cross_zero = false;
	for(int i=0;i<waypoint.size();i++){		   
		if(pow(waypoint[search][0]-curr_x,2)+pow(waypoint[search][1]-curr_y,2) == pow(LOOK_AHEAD,2)){
			//index = min_index + i;
			x = waypoint[search][0];
			y = waypoint[search][1];
			
			break;
		}
		    //if the best is some point between two points
		if((pow(waypoint[search][0]-curr_x,2)+pow(waypoint[search][1]-curr_y,2) <= pow(LOOK_AHEAD,2)) && (pow(waypoint[search+1][0]-curr_x,2)+pow(waypoint[search+1][1]-curr_y,2) >= pow(LOOK_AHEAD,2))){
			//index = min_index + i;
			x = (waypoint[search][0] + waypoint[search+1][0])/2;
			y = (waypoint[search][1] + waypoint[search+1][1])/2;
				
			break;

		}
		  
		search = min_index+i;
	}
		// ROS_INFO("the look ahead point is x:%f, y:%f", x, y);
	std::vector<float> best_point;
	best_point.push_back(x);
	best_point.push_back(y);
	chosen_point.id = 1;
	chosen_point.header.stamp = ros::Time::now();
	chosen_point.pose.position.x = x;
	chosen_point.pose.position.y = y;
	chosen_point_pub.publish(chosen_point);

//////////////////////////////////////////////////////////////////////////////////////////       


    //update goal point
    goal_node_.x = best_point[0];
    goal_node_.y = best_point[1];

    float map_x, map_y;
    map_x = (abs(map_origin_x_) + curr_.x)/map_resolution_;
    map_y = (abs(map_origin_y_) - curr_.y)/map_resolution_;

    //update submap

if(ctn == 0){
    submap.clear();
    
    for(int i=0; i<120; i++){
        std::vector<float> row3;
	for(int j=0; j<120; j++){
	    row3.push_back(map_global_[map_y-60+j][map_x-60+i]);
		//if 0,1
	    //ROS_INFO("point %d, %d is %f", i, j, map_global_[map_y-60+j][map_x-60+i]);
	}
	submap.push_back(row3);
    }
	ctn ++;
}
    
        free_point.points.clear();
	for(int m=0; m<120; m++){
			for(int n=0; n<120; n++){
				
				//ROS_INFO("submap %d, %d: %f ", m, n, submap[m][n]);
				//ROS_INFO("5");
//ROS_INFO("1111");
			    if(submap[m][n]>0.96){
				free_point.id = 4;
				free_point.header.stamp = ros::Time::now();
				geometry_msgs::Point p;
				p.x = curr_.x + (m - 60)*map_resolution_;
				p.y = curr_.y - (n - 60)*map_resolution_;
				p.z = 0;
	//ROS_INFO("2222");

				free_point.points.push_back(p);
			    }
			}
	}
	free_point_pub_.publish(free_point);


    //goal_node_ in global map
    int goal_map_x = (abs(map_origin_x_) + goal_node_.x)/map_resolution_;
    int goal_map_y = (abs(map_origin_y_) - goal_node_.y)/map_resolution_;
    //goal_node_ in submap
    int goal_submap_x = 60 + goal_map_x - map_x;
    int goal_submap_y = 60 + goal_map_y - map_y;

    ROS_INFO("map_origin_x_ %f, map_origin_y %f", map_origin_x_, map_origin_y_);
    //ROS_INFO("LOOK_AHEAD %f", LOOK_AHEAD);
    ROS_INFO("map_x %f, map_y %f", map_x, map_y);
    ROS_INFO("goal_node_.x %f, goal_node_.y %f", goal_node_.x, goal_node_.y);
    ROS_INFO("goal_map_x %d, goal_map_y %d", goal_map_x, goal_map_y);
    ROS_INFO("goal_submap_x %d, goal_submap_y %d", goal_submap_x, goal_submap_y);

/////////////////////////////////////////////////////////////////////////////////////////

    //now we have the submap and the goal point in submap

    std::vector<double> sampled_point;
    int nearest_node = 0;
    Node new_node;
    
    root_.x = 60;/// + curr_.xmap_resolution_
    root_.y = 60;
    //ROS_INFO("curr.x x %f, y %f", curr_.x, curr_.y);
    //ROS_INFO("root_.x x %f, y %f", root_.x, root_.y);
    root_.is_root = true;
    tree.push_back(root_);//submap??

    //////////////////////////////////////////////////////////
    //main loop
    for(int k=0; k< interation_; k++){
	ROS_INFO("now we sample new point");
	sampled_point = sample(goal_submap_x, goal_submap_y);
    	ROS_INFO("sample.x %f, sample.y %f", sampled_point[0], sampled_point[1]);
	nearest_node = nearest(tree, sampled_point);
	ROS_INFO("nearest node index %d", nearest_node); 
	ROS_INFO("the nearest x %f, y %f", tree[nearest_node].x, tree[nearest_node].y);
	new_node = steer(tree[nearest_node], sampled_point);
	ROS_INFO("the steered x %f, y %f", new_node.x, new_node.y);

	if(!check_collision(tree[nearest_node], new_node)){
	    new_node.parent = nearest_node;
	    tree.push_back(new_node);
	}else{
	    continue;
	}
	if(is_goal(new_node, goal_submap_x, goal_submap_y)){
	    ROS_INFO("found %f, %f is the current goal", new_node.x, new_node.y);
	    break;
	}
    }
    
/////////////////////////////////////////////////////////////////////////////////////////////
/*
if(updated){
    submap.clear();
    for(int i=0; i<120; i++){
        std::vector<float> row3;
	for(int j=0; j<120; j++){
	    row3.push_back(map_global_[map_y-60+j][map_x-60+i]);
		//if 0,1
	    //ROS_INFO("point %d, %d is %f", i, j, map_global_[map_y-60+j][map_x-60+i]);
	}
	submap.push_back(row3);
    }
    updated = false;
}*/
/////////////////////////////////////////////////////////////

    //publish the path message
    // path found as Path message
    std::vector<Node> found_path;
    found_path = find_path(tree, new_node);
    
	std::vector<geometry_msgs::PoseStamped> temp_plan;
	geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "map";
	pose.pose.position.x = curr_.x + (goal_submap_x - 60)*map_resolution_;
	pose.pose.position.y = curr_.y - (goal_submap_y - 60)*map_resolution_;
	pose.pose.orientation.w = 1;
	temp_plan.push_back(pose);

	pose.pose.position.x = curr_.x + (new_node.x - 60)*map_resolution_;
	pose.pose.position.y = curr_.y - (new_node.y - 60)*map_resolution_;
	pose.pose.orientation.w = 1;
	temp_plan.push_back(pose);

    int curr_parent = new_node.parent;
    while(!tree[curr_parent].is_root){

	pose.header.frame_id = "map";
	pose.pose.position.x = curr_.x + (tree[curr_parent].x - 60)*map_resolution_;
	pose.pose.position.y = curr_.y - (tree[curr_parent].y - 60)*map_resolution_;
	pose.pose.orientation.w = 1;
	temp_plan.push_back(pose);
	curr_parent = tree[curr_parent].parent;	

    }
    pose.pose.position.x = curr_.x;
    pose.pose.position.y = curr_.y;
    pose.pose.orientation.w = 1;
    temp_plan.push_back(pose);

    plan_.clear();
    for(int i = temp_plan.size()-1; i>=0; i--){
	plan_.push_back(temp_plan[i]);
    }
    //plan_pub_.publish(plan_, 0, 1, 0, 0);
    nav_msgs::Path gui_path;
    gui_path.poses.resize(plan_.size());
    if(!plan_.empty()){
	gui_path.header.frame_id = plan_[0].header.frame_id;
	gui_path.header.stamp = plan_[0].header.stamp;
    }
    for(unsigned int i=0; i<plan_.size(); i++){
	gui_path.poses[i] = plan_[i];
    }
    plan_pub_.publish(gui_path);

/////////////////////////////////////////////////////////////////////////////////////////////
    //we publish the nodes in the tree
	tree_node_point.points.clear();

    sensor_msgs::PointCloud tree_to_publish;
    tree_to_publish.header.frame_id = "map";
    for(int i=0; i< tree.size(); i++){
	geometry_msgs::Point32 point;//32
	//ROS_INFO("the previous to publish %f, %f to the map", tree[i].x, tree[i].y);
	point.x = curr_.x + (tree[i].x - 60)*map_resolution_;
	point.y = curr_.y - (tree[i].y - 60)*map_resolution_;//??????????????
	//ROS_INFO("publish %f, %f to the map", point.x, point.y);
	tree_to_publish.points.push_back(point);

	tree_node_point.id = 4;
	tree_node_point.header.stamp = ros::Time::now();
	geometry_msgs::Point p;
	p.x = point.x;
	p.y = point.y;
	p.z = 0;
	//tree_node_point.pose.position.x = point.x;
	//tree_node_point.pose.position.y = point.y;
	tree_node_point.points.push_back(p);

    }
    tree_node_point_pub_.publish(tree_node_point);
    tree_pub_.publish(tree_to_publish);

//////////////////////////////////////////////////////////////////////////////////////////////

    //now publish the drive topic 
    double angle, curvature;//, velocity

    //velocity = 0.2;

    double quatx= pose_msg->pose.pose.orientation.x;
    double quaty= pose_msg->pose.pose.orientation.y;
    double quatz= pose_msg->pose.pose.orientation.z;
    double quatw= pose_msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch;//, yaw
    m.getRPY(roll, pitch, yaw);

    //ROS_INFO("rpy              %f, %f, %f", roll, pitch, yaw);

    float vehicle_x, vehicle_y;

    Eigen::Matrix2f R;
    Eigen::Vector2f x_c;
    Eigen::Vector2f x_v;
    Eigen::Vector2f x_m;

    R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

    if(found_path.size()>7){
    x_m << (found_path[found_path.size()-7].x-60)/20 + curr_x, -(found_path[found_path.size()-7].y-60)/20 + curr_y;
    }else{
	x_m << (found_path[found_path.size()-2].x-60)/20 + curr_x, -(found_path[found_path.size()-2].y-60)/20 + curr_y;
    }
    x_c << curr_x, curr_y;
    //x_m << x, y;
    ROS_INFO("(found_path[found_path.size()-4].x-60)/20  x %f, y %f", (found_path[found_path.size()-4].x-60)/20 , (found_path[found_path.size()-5].y-60)/20);
    ROS_INFO("curr_x, curr_y  x %f, y %f", curr_x, curr_y);
    ROS_INFO("x, y  x %f, y %f", x, y);
    //x_m << (found_path[0].x-60)/20, (found_path[0].y-60)/20;
    x_v = R.inverse()*(x_m - x_c);//(x_m - x_c)

    vehicle_x = x_v(0);//x_v(0)
    vehicle_y = x_v(1);//x_v(1)
	//vehicle_x = found_path[0].x;//tree[tree.size()-1].x
	//vehicle_y = found_path[0].y;
	ROS_INFO("xxxxxxxxxxxxx x %f, y %f", x, y);

ROS_INFO("found_path[found_path.size()-4].x x %f, y %f", found_path[found_path.size()-4].x, found_path[found_path.size()-4].y);
//ROS_INFO("x_m - x_c x %f, y %f", (x_m - x_c)(0), (x_m - x_c)(1));

ROS_INFO("vehicle_x x %f, y %f", vehicle_x, vehicle_y);


    curvature = 2 * vehicle_y / (pow(vehicle_y, 2) + pow(vehicle_x, 2));//abs(vehicle_y)?
    angle = asin(0.325*curvature);

    ackermann_msgs::AckermannDriveStamped drv;
    drv.header.frame_id = "pure";
    drv.drive.steering_angle = angle;
    drv.drive.speed = velocity;
    ROS_INFO("the speed is x:%f, angle:%f", velocity, angle);
    nav_pub_.publish(drv);

}




std::vector<double> RRT::sample(int goal_submap_x, int goal_submap_y) {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TO_DO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    

    int gen_x, gen_y;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(goal_submap_x - 30, goal_submap_x + 30);
    std::uniform_real_distribution<double> y_dist(goal_submap_y - 30, goal_submap_y + 30);
    
    gen_x = x_dist(gen);
    gen_y = y_dist(gen);
    int cnt = 0;

    while(gen_x >= 120 || gen_y >= 120 || gen_x < 0 || gen_y < 0){
	gen_x = x_dist(gen);
    	gen_y = y_dist(gen);
    }
 //ROS_INFO("3, %d, %d", gen_x, gen_y);

    //if it not in free space, then resample it
    while(submap[gen_x][gen_y] < 1.0){
	gen_x = x_dist(gen);
    	gen_y = y_dist(gen);
	while(gen_x >= 120 || gen_y >= 120 || gen_x < 0 || gen_y < 0){
	    gen_x = x_dist(gen);
    	    gen_y = y_dist(gen);
        }
	cnt += 1;
	if(cnt > 100){
	    ROS_INFO("can not find point in freespace");
	    ROS_INFO("the target point is %d, %d, and the value %f", gen_x, gen_y, submap[gen_x][gen_y]);
	}
    }
 //ROS_INFO("4, %d, %d", gen_x, gen_y);
    sampled_point.push_back(gen_x);  
    sampled_point.push_back(gen_y);  

    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TO_DO: fill in this method
    double min_dist_ = 10000;
    for(int i=0; i< tree.size(); i++){
	double dist = sqrt(pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2));
	if(dist < min_dist_){
	    nearest_node = i;
	    min_dist_ = dist;
	}
    }
    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TO_DO: fill in this method
    double node_dist = sqrt(pow(sampled_point[0] - nearest_node.x, 2) + pow(sampled_point[1] - nearest_node.y, 2));
    new_node.x = sampled_point[0];
    new_node.y = sampled_point[1];
    ROS_INFO("node dist %f", node_dist);
    while(node_dist > max_expansion_dist_){
	//not found?
        new_node.x -= (sampled_point[0] - nearest_node.x) * 0.03;
	new_node.y -= (sampled_point[1] - nearest_node.y) * 0.03;
	node_dist = sqrt(pow(new_node.x - nearest_node.x, 2) + pow(new_node.y - nearest_node.y, 2));
	//ROS_INFO("node dist %f, max dist: %f", node_dist, max_expansion_dist_);
    }

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TO_DO: fill in this method

    float x1, y1, x2, y2, k, b;
    x1 = nearest_node.x;
    y1 = nearest_node.y;
    x2 = new_node.x;
    y2 = new_node.y;
    k = (y2 - y1)/(x2 - x1);
    b = y1 - k * x1;
    if(isinf(k)){k = 1000;}
    if(isinf(b)){b = 1000;}
 
	for(int i=std::min(x1, x2); i<= x1 + x2 - std::min(x1, x2); i++){
	    int y_map = k*i+b;
	    //ROS_INFO("k %f, b %f", k, b);
		//ROS_INFO("x1 %f, y1 %f", x1, y1);
		//ROS_INFO("x2 %f, y2 %f", x2, y2);
	    if(y_map>=120){y_map = 119;}
	    if(y_map<0){y_map = 0;}
	    if(submap[i][y_map] < 0.96){
		collision = true;
		
		//ROS_INFO("submap  %f", submap[60][60]);
		//ROS_INFO("submap2  %f", submap[y_map][i]);
		ROS_INFO("collision x %d, y %d", i, y_map);
		break;
	    }
	}

    //collision = false;///////////////////////////
    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TO_DO: fill in this method

    if(sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2)) < threshold_){// * map_resolution_
	ROS_INFO("we found the goal!!");
	ROS_INFO("x %f, y %f ", latest_added_node.x - goal_x, latest_added_node.y - goal_y);
	close_enough = true;
    }
    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    // TO_DO: fill in this method

    found_path.push_back(latest_added_node);

    int curr_parent = latest_added_node.parent;
    while(!tree[curr_parent].is_root){
	found_path.push_back(tree[curr_parent]);
	curr_parent = tree[curr_parent].parent;
    }
    found_path.push_back(tree[0]);

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TO_DO: fill in this method
	
	index = node.parent;//parent node index
	cost += node.cost;
	
	while(!tree[index].is_root){
		cost += tree[index].cost;
		index = tree[index].parent;
	}

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TO_DO: fill in this method
    cost = sqrt(pow(n1.x-n2.x, 2)+pow(n1.y-n2.y, 2));

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TO_DO:: fill in this method

	double distant;
	for(int i=0; i<tree.size(); i++){
		distant = sqrt(pow(tree[i].x-node.x, 2)+pow(tree[i].y-node.y, 2));
		if(distant < near_threshold_){
			neighbourhood.push_back(i);
		}
	}

    return neighborhood;
}
