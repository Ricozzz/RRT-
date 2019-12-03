// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

#include <nav_msgs/Path.h>


// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
    for(int i=0; i<rrt_nodes_.size(); i++)
    {
	//delete rrt_nodes_[i];
    }
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
    nh_.getParam("/rrt_node/max_iteration", interation_);
    nh_.getParam("/rrt_node/max_expansion_dist", max_expansion_dist_);
    nh_.getParam("/rrt_node/LOOK_AHEAD", LOOK_AHEAD);
    
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

    // ROS subscribers
    // TO_DO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    //map_sub_ = nh.subscribe("map", 1, boost::bind(&RRT::makemap, this, 1));

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
    std::ifstream fin2("/home/zan/zhenzan_ws/src/rrt/src/wp.csv");
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
    // TODO: update your occupancy grid
    double min, max, angle;
    int sample_time;
    min = scan_msg -> angle_min;
    max = scan_msg -> angle_max;
    angle = scan_msg -> angle_increment;
    sample_time = int((max-min)/angle);
    std::vector<float> distant;
    distant = scan_msg -> ranges;

    //update occupancy grid?
    //ROS_INFO("Donot update for now.");





}

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

    // TODO: fill in the RRT main loop
    curr_.x = pose_msg->pose.pose.position.x;
    curr_.y = pose_msg->pose.pose.position.y;
    root_.x = curr_.x;
    root_.y = curr_.y;
    rrt_nodes_.push_back(root_);

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

    //create submap
    std::vector<std::vector<int>> submap;
    float map_x, map_y;
    map_x = (abs(map_origin_x_) + curr_.x)/map_resolution_;
    map_y = (abs(map_origin_y_) - curr_.y)/map_resolution_;
    for(int i=0; i<120; i++){
        std::vector<int> row3;
	for(int j=0; j<120; j++){
	    row3.push_back(map_global_[map_x-60+j][map_y-60+i]);
		//if 0,1
	}
	submap.push_back(row3);
    }
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
    
    root_.x = 60 + root_.x/map_resolution_;
    root_.y = 60 + root_.y/map_resolution_;
    ROS_INFO("root_.x x %f, y %f", root_.x, root_.x);
    root_.is_root = true;
    tree.push_back(root_);//submap??
 

    for(int i=0; i< interation_; i++){
	
	sampled_point = sample(goal_submap_x, goal_submap_y);
    	ROS_INFO("sample.x %f, sample.y %f", sampled_point[0], sampled_point[1]);
	nearest_node = nearest(tree, sampled_point);
	ROS_INFO("nearest node index %d", nearest_node); 
	ROS_INFO("the nearest x %f, y %f", tree[nearest_node].x, tree[nearest_node].y);
	new_node = steer(tree[nearest_node], sampled_point);
	ROS_INFO("the steered x %f, y %f", new_node.x, new_node.y);

	if(check_collision(tree[nearest_node], new_node)){
	    new_node.parent = nearest_node;
	    tree.push_back(new_node);
	}
	if(is_goal(new_node, goal_submap_x, goal_submap_y)){
	    ROS_INFO("found %f, %f is the current goal", new_node.x, new_node.y);
	    break;
	}
    }

/////////////////////////////////////////////////////////////////////////////////////////////

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

    //publish the whole tree node
/*    sensor_msgs::PointCloud tree_to_publish;
    tree_to_publish.header.frame_id = "map";
    for(int i=0; i< tree.size(); i++){
	geometry_msgs::Point32 point;
	//ROS_INFO("the previous to publish %f, %f to the map", tree[i].x, tree[i].y);
	point.x = curr_.x + (tree[i].x - 60)*map_resolution_;
	point.y = curr_.y - (tree[i].y - 60)*map_resolution_;//??????????????
	//ROS_INFO("publish %f, %f to the map", point.x, point.y);
	tree_to_publish.points.push_back(point);

	tree_node_point.id = 1;
	tree_node_point.header.stamp = ros::Time::now();
	tree_node_point.pose.position.x = point.x;
	tree_node_point.pose.position.y = point.y;
	tree_node_point_pub_.publish(tree_node_point);
    }
    tree_pub_.publish(tree_to_publish);*/

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
    
//free???????????????????????????????????????????????????????????????????
    double gen_x, gen_y;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(goal_submap_x - 10, goal_submap_x + 10);
    std::uniform_real_distribution<double> y_dist(goal_submap_y - 10, goal_submap_y + 10);
   
    gen_x = x_dist(gen);
    gen_y = y_dist(gen);

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
        new_node.x -= (sampled_point[0] - nearest_node.x) * 0.1;
	new_node.y -= (sampled_point[1] - nearest_node.y) * 0.1;
	node_dist = sqrt(pow(new_node.x - nearest_node.x, 2) + pow(new_node.y - nearest_node.y, 2));
	ROS_INFO("node dist %f, max dist: %f", node_dist, max_expansion_dist_);
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

    bool collision = true;
    // TODO: fill in this method

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
    // TODO: fill in this method

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
    // TODO: fill in this method

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
    // TODO:: fill in this method

    return neighborhood;
}
