#ifndef __VISION_H__
#define __VISION_H__

#include <ros/ros.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace asulada {

class IVision;

class Vision {
public:
	Vision();
	~Vision();
	static Vision *getInstance(ros::NodeHandle *nh);
	static void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
	static void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void doWork(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg);
	int start();
	void stop();
	void addListener(IVision *l);
	void removeListener(IVision *l);

private:
	void _subscribe();
	void _unsubscribe();
	void _notify(double x, double y, double dimension);

	static Vision *inst_;
	image_transport::Publisher img_pub_;
	image_transport::Subscriber img_sub_;
	image_transport::CameraSubscriber cam_sub_;

	cv::CascadeClassifier face_cascade_;
	cv::CascadeClassifier eyes_cascade_;
	ros::Publisher msg_pub_;

	boost::shared_ptr<image_transport::ImageTransport> it_;
	ros::NodeHandle *pnh_;
	//ros::NodeHandle *nh_;
	std::vector<IVision *>listeners_;

//	bool debug_view_;
//	ros::Time prev_stamp_;

};
}

#endif /* __VISION_H__ */
