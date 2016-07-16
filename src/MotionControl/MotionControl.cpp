#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This is the core to control motor & camera
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotionControl");

	ros::NodeHandle nh;

	/**
	 * @brief motion - publishes motion's status such as moving, or idle. Core could call acitions to other modules
	 */
	ros::Publisher motorPublisher = nh.advertise<std_msgs::String>("motion", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {
		/**
         * This is a message object. You stuff it with data, and then publish it.
         */
		std_msgs::String msg;

		std::stringstream ss;
		ss << "Motor changed" << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		/**
		* publish message whose type is std_msgs::String, and it sends to SmartControl
		*/
	 	motorPublisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		count++;
	}
	return 0;
}
