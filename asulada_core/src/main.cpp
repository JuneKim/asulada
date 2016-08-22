#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MotionCtrl.h"

void motorControlChanged(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AsuladaMotion");

	ros::NodeHandle *nh = new ros::NodeHandle();	
	asulada::MotionCtrl *ctrl = new asulada::MotionCtrl(nh);
	ctrl->start();

	while (ros::ok()) {
		ROS_INFO("test");

	}
	ctrl->stop();


	ros::spin();

	return 0;
}


