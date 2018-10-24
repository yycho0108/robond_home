#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void set_goal(
		move_base_msgs::MoveBaseGoal& goal,
		float x, float y, float h
		){

	// fill in header information again, just in case
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	// now fill in data
	auto& p = goal.target_pose.pose.position;
	auto& q = goal.target_pose.pose.orientation;

	p.x = x;
	p.y = y;
	p.z = 0;

	q.x = 0;
	q.y = 0;
	q.z = sin(h/2);
	q.w = cos(h/2);
}

bool send_goal(
		MoveBaseClient& ac,
		move_base_msgs::MoveBaseGoal& goal
		){
	auto& p = goal.target_pose.pose.position;
	auto& q = goal.target_pose.pose.orientation;

	// Send the goal position and orientation for the robot to reach
	// + also add information about the goal
	ROS_INFO("Sending goal (x=%.2f, y=%.2f, qz=%.2f, qw=%.2f)", p.x, p.y, q.z, q.w);

	ac.sendGoal(goal);
	// Wait an infinite time for the results
	ac.waitForResult();
	// Check if the robot reached its goal
	return (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool wait(){
	// wait
	ROS_INFO("Waiting for 5 seconds for payload ...");
	ros::Rate rate(10.0); // check every 0.1 sec
	auto wait_start = ros::Time::now();

	while(true){
		if(!ros::ok()){
			std::cout << "ROS Broke; aborting" << std::endl;
			return false;
		}

		float dt = (ros::Time::now() - wait_start).toSec();

		if(dt > 5.0){
			// finished waiting for 5 sec.
			break;
		}
		ROS_INFO_THROTTLE(0.5, "%f seconds left to departure", (5.0 - dt) );
		rate.sleep();
	}

	return true;
}

int main(int argc, char** argv){
	// Initialize the simple_navigation_goals node
	ros::init(argc, argv, "pick_objects");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// Wait 5 sec for move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;
	bool suc;

	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	// pickup
	ROS_INFO("Leaving for pickup zone ...");
	set_goal(goal, 3.892, 6.799, 1.66);
	suc = send_goal(ac,goal);

	// display log
	if(suc){
		ROS_INFO("Hooray, the base successfully moved to the pickup zone!");
	}else{
		ROS_INFO("The base failed to move to the pickup zone for some reason");
		return 1;
	}

	// wait
	suc = wait();
	if(suc){
		ROS_INFO("Finished Waiting!");
	}else{
		ROS_INFO("Waiting failed for some reason, aborting.");
		return 2;
	}

	// dropoff
	ROS_INFO("Leaving for dropoff zone ...");
	set_goal(goal, -3.195, 5.968, 3.25);
	suc = send_goal(ac,goal);

	// display log
	if(suc){
		ROS_INFO("Hooray, the base successfully moved to the dropoff zone!");
	}else{
		ROS_INFO("The base failed to move to the dropoff for some reason");
		return 3;
	}

	return 0;
}
