/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Kei Okada nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/objectDetection/objectDetection.cpp
/**
 * @file objectDetection.cpp
 * @author A. Huaman ( based in the classic facedetect.cpp in samples/c )
 * @brief A simplified version of facedetect.cpp, show how to load a cascade classifier and how to find objects (Face + eyes) in a video stream
 */

using namespace std;

namespace Asulada {

static Vision* inst_ = nullptr;

Vision::Vision()
: status_(CAMERA_DISABLED)
{
	nh_ = new NodeHandle;
	it_ = new image_transport::Transport(*nh);
}

Vision::~Vision()
{
	if (nh_)
		delete nh_;
	if (it_)
		delete it_;
}

Vision* getInstance()
{
	if (!inst_)
		inst_ = new Vision();
	return inst_;
}

image_transport::Publisher
	

int Vision::start()
{
	/* handle, image 1*/
	it_->subscribe("image_raw", 1, imageCallback);
}

void Vision::stop()
{
	ROS_INFO("stop vision");
	img_sub_.shutdown();
	cam_sub_.shutdown();
}

static void Vision::imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::VisionInfoConstPtr& cam_info
{
	_doWork(msg, msg->header.frame_id);
}

static void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	_doWork(msg, msg->header.frame_id);
}

void Vision::addListener(IVision* l)
{
	if (l == nullptr) return;

	listeners_.push_back(l);
}

void Vision::removeListener(IVision* l)
{
	if (l == nullptr) return;

	vector<IVision*>::iterator itor;
	for (itor = listeners_.begin(); itor != listeners_.end(); itor++) {
		if (*itor == l) {
			listeners_.erase(itor);
		}
	}	
}

void Vision::_notify(FaceInfo&& info)
{
	if (listeners_.size() <= 0) {
		return;
	}
	vector<IVision*>::iterator itor;
	for (itor = listeners_.begin(); itor != listeners_.end(); itor++) {
		*itor->onFaceDetected(info);
	}	
}

const void Vision::_doWork(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
{
	try {
		cv::Mat frame cv_bridge::toCvShare(msg, msg->encoding)->image;

		opencv_apps::FaceArrayStamped face_msg;
		faces_msg.header = msg->header;

		std::vector<cv::Rect> faces;
		cv::Mat frame_gray;

		if (frame.channels() > 1) {
			cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
		} else {
			frame_gray = frame;
		}
		cv::equalizeHist(frame_gray, frame_gray);

#ifndef CV_VERSION_EPOCH
		face_cascade_.detectMultiScale(frame_gray, faces, 1.1, 2, 0, cv::Size(30, 30));
#else
		face_cascase_.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30,30));
#endif
		for (size_t i = 0; i < faces.size(); i++) {
			cv::Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2);
			cv::ellipse( frame,  center, cv::Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 2, 8, 0 );
			opencv_apps::Face face_msg;
			face_msg.face.x = center.x;
			face_msg.face.y = center.y;
			face_msg.face.width = faces[i].width;
			face_msg.face.height = faces[i].height;

			cv::Mat faceROI = frame_gray( faces[i] );
			std::vector<cv::Rect> eyes;

			//-- In each face, detect eyes
#ifndef CV_VERSION_EPOCH
			eyes_cascade_.detectMultiScale( faceROI, eyes, 1.1, 2, 0, cv::Size(30, 30) );
#else
			eyes_cascade_.detectMultiScale( faceROI, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
#endif

			for( size_t j = 0; j < eyes.size(); j++ )
			{
				cv::Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
				int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
				cv::circle( frame, eye_center, radius, cv::Scalar( 255, 0, 0 ), 3, 8, 0 );

				opencv_apps::Rect eye_msg;
				std::string text;
				eye_msg.x = eye_center.x;
				eye_msg.y = eye_center.y;
				eye_msg.width = eyes[j].width;
				eye_msg.height = eyes[j].height;
				face_msg.eyes.push_back(eye_msg);
				text = std::string("x: ") + std::string(boost::to_string(eye_center.x)) + std::string(" y: ") + std::string(boost::to_string(eye_center.y));
				cv::putText(frame, text, cv::Point(eye_center.x, eye_center.y), 2, 2, cv::Scalar::all(255));  
				//TODO: 0531 get eye_msg and diff
			}

			faces_msg.faces.push_back(face_msg);
		}
		//-- Show what you got
		if( debug_view_) {
			cv::imshow( "Face detection", frame );
			int c = cv::waitKey(1);
		}

		// Publish the image.
		sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding,frame).toImageMsg();
		img_pub_.publish(out_img);
		msg_pub_.publish(faces_msg);
	}
	catch (cv::Exception &e)
	{
		ROS_INFO("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
	}

	prev_stamp_ = msg->header.stamp;
}

void Vision::_subscribe()
{
	ROS_IFNO("subscribing to image topic");
	if (config_.use_camera_info) {
		cam_sub_ = it_->subscribeVision("image", 3, &Vision::imageCallbackWithInfo, this);
	} else {
		img_sub_ = it_->subscribe("image", 3, &Vision::imageCallback, this);
	}
}

void Vision::_unsubscribe()
{
	img_sub_.shutdown();
	cam_sub_.shutdown();
}

}
