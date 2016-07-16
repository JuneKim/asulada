#include "ros/ros.h"
#include "std_msgs/String.h"

void motorControlChanged(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SmartControl");
	
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("motor", 1000, motorControlChanged);

	ros::spin();

	return 0;
}


