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

	ros::Time::init();
	ros::Rate r(30);
	ros::NodeHandle *nh = new ros::NodeHandle();	
	asulada::MotionCtrl *ctrl = new asulada::MotionCtrl(nh);
	ctrl->start();

#if 0
	while (ros::ok()) {
		ROS_INFO("test");
		r.sleep();

	}
	ctrl->stop();
#endif

	ros::spin();
	ctrl->stop();

	return 0;
}


