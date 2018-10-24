#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// current position
float cx = 0.0, cy = 0.0;

void show(ros::Publisher& pub,
		visualization_msgs::Marker& msg,
		float x, float y){

	const float radius = 0.25;
	const float height = 0.5;

	msg.ns = "object";
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.id = 0;
	msg.type = msg.CYLINDER;
	msg.action = msg.ADD;

	// setup pose
	msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.position.z = height / 2.0;

	msg.pose.orientation.x = 0.0;
	msg.pose.orientation.y = 0.0;
	msg.pose.orientation.z = 0.0;
	msg.pose.orientation.w = 1.0;

	// setup shape
	msg.scale.x = radius;
	msg.scale.y = radius;
	msg.scale.z = height;

	// setup color
	msg.color.r = 0.7f;
	msg.color.g = 0.3f;
	msg.color.b = 0.4f;
	msg.color.a = 1.0f;

	msg.lifetime = ros::Duration(0); // "forever"

	pub.publish(msg);
}

void hide(ros::Publisher& pub, visualization_msgs::Marker& msg){
	msg.action = msg.DELETEALL;
	msg.header.stamp = ros::Time::now();
	pub.publish(msg);
}

bool wait(float x, float y, float gtol,
		ros::Publisher& pub,
		visualization_msgs::Marker& msg,
		bool show
		){
	ros::Time wait_start = ros::Time::now();

	ros::Rate rate(1.0);

	while(true){
		if(!ros::ok()){
			std::cerr << "ROS Broke for some reason; Aborting." << std::endl;
			return false;
		}

		if(show){
			msg.header.stamp = ros::Time::now();
			pub.publish(msg);
		}

		// feedback on how long the wait was
		if(wait_start.toSec() == 0){
			//get proper time
			wait_start = ros::Time::now();
		}else{
			float dt = (ros::Time::now() - wait_start).toSec();
			ROS_INFO_THROTTLE(1.0, "Been %.2f seconds since waiting", dt);
		}

		// check goal reached
		float d = sqrt(pow(cx - x, 2) + pow(cy - y, 2));
		ROS_INFO_THROTTLE(2.0, "Current Distance : %.2f", d);
		if(d < gtol) break;
		rate.sleep();
		ros::spinOnce();
	}
	return true;
}

void pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
	//latest_pose = *msg;
	cx = msg->pose.pose.position.x;
	cy = msg->pose.pose.position.y;
}

int main(int argc, char* argv[]){
	ros::init(argc,argv, "add_markers");

	// create ROS Handles
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("");
	ros::Publisher mk_pub = nh.advertise<visualization_msgs::Marker>("viz", 10);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10,
			pose_cb);

	// get goal information
	double pk_x, pk_y, dp_x, dp_y, gtol;
	bool suc;

	// pickup
	nh_priv.param<double>("pk_x", pk_x, 3.892);
	nh_priv.param<double>("pk_y", pk_y, 6.799);

	//dropoff
	nh_priv.param<double>("dp_x", dp_x, -3.195);
	nh_priv.param<double>("dp_x", dp_y, 5.968);

	//goal distance tolerance
	nh_priv.param<double>("gtol", gtol, 0.5);

	visualization_msgs::Marker msg;

	// show marker @ pickup
	show(mk_pub, msg, pk_x, pk_y);

	// wait while robot reaches pickup zone
	suc = wait(pk_x, pk_y, gtol,
			mk_pub, msg, true
			);
	if(!suc){
		ROS_INFO("Pickup Wait Unsuccessful");
		return 1;
	}
	hide(mk_pub, msg);

	ROS_INFO("Picked up the Object!");

	// wait while robot reaches dropoff zone
	suc = wait(dp_x, dp_y, gtol, mk_pub, msg, false);
	if(!suc){
		ROS_INFO("Dropoff Wait Unsuccessful");
		return 2;

	}
	show(mk_pub, msg, dp_x, dp_y);
	ROS_INFO("Mission Success!!");

	return 0;
}
