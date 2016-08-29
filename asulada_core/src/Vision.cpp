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

//#include "opencv_apps/FaceDetectionConfig.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include "asulada_core/Face.h"
#include "asulada_core/FaceArray.h"
#include "asulada_core/FaceArrayStamped.h"

#include "Vision.h"
#include "IVision.h"

using namespace std;

namespace asulada {

bool debug_view_;
ros::Publisher msg_pub_;
ros::Time prev_stamp_;
bool always_subscribe_;
const char* VISION_RAW_IMAGE = "/usb_cam/image_raw";

cv::CascadeClassifier face_cascade_;
cv::CascadeClassifier eyes_cascade_;
image_transport::Publisher img_pub_;
image_transport::Subscriber img_sub_;
image_transport::CameraSubscriber cam_sub_;

Vision *Vision::inst_ = NULL;

Vision::Vision()
{
}

Vision::~Vision()
{
}

void Vision::addListener(asulada::IVision *l)
{
	listeners_.push_back(l);
}

void Vision::removeListener(asulada::IVision *l)
{
	vector<IVision*>::iterator itor;
	for (itor = listeners_.begin(); itor != listeners_.end(); itor++) {
		if (*itor == l) {
			listeners_.erase(itor);
			break;
		}
	}	
}

void Vision::_notify(double x, double y, double dimension)
{
	ROS_INFO("x[%f], y[%f], dimension[%f]", x, y, dimension);
	vector<IVision *>::iterator itor;
	if (listeners_.size() > 0) {
		for (itor = listeners_.begin(); itor != listeners_.end(); itor++) {
			((asulada::IVision *)*itor)->onFaceDetected(x, y, dimension);
		}
	}
}

void Vision::imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	//ROS_INFO("%s", __func__);
	doWork(msg, cam_info->header.frame_id);
}

void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO("%s", __func__);
    doWork(msg, msg->header.frame_id);
}

void Vision::doWork(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
{
	//ROS_INFO("doWork");
	double _x, _y, _dimension, _tmpDimension;
	_x = _y = _dimension = _tmpDimension = 0.0;

    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

      // Messages
      asulada_core::FaceArrayStamped faces_msg;
      faces_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat frame_gray;

      if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
      } else {
        frame_gray = frame;
      }
      cv::equalizeHist( frame_gray, frame_gray );
      //-- Detect faces
#ifndef CV_VERSION_EPOCH
      face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 2, 0, cv::Size(30, 30) );
#else
      face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
#endif

      for( size_t i = 0; i < faces.size(); i++ )
      {
        cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        cv::ellipse( frame,  center, cv::Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 2, 8, 0 );
        asulada_core::Face face_msg;
        face_msg.face.x = center.x;
        face_msg.face.y = center.y;
        face_msg.face.width = faces[i].width;
        face_msg.face.height = faces[i].height;

		_tmpDimension = faces[i].width * faces[i].height;
		if (_dimension < _tmpDimension) {
			_x = center.x;
			_y = center.y;
			_dimension = _tmpDimension;
		}

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

          asulada_core::Rect eye_msg;
          eye_msg.x = eye_center.x;
          eye_msg.y = eye_center.y;
          eye_msg.width = eyes[j].width;
          eye_msg.height = eyes[j].height;
          face_msg.eyes.push_back(eye_msg);
        }
		inst_->_notify(_x, _y, _dimension);
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
		ROS_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
}

void Vision::_subscribe()
{
	ROS_INFO("%s", __func__);
    //ROS_DEBUG("Subscribing to image topic.");
    cam_sub_ = it_->subscribeCamera("/usb_cam/image_raw", 1000, &imageCallbackWithInfo);
	ROS_INFO("get topic info[%s, %s]", cam_sub_.getTopic().c_str(), cam_sub_.getInfoTopic().c_str());
	//img_sub_ = it_->subscribe(VISION_RAW_IMAGE, 1, &imageCallback);
}

void Vision::_unsubscribe()
{
	ROS_INFO("unsubscribe");
    //ROS_DEBUG("Unsubscribing from image topic.");
	// http://docs.ros.org/kinetic/api/image_transport/html/classimage__transport_1_1Subscriber.html
    //img_sub_.shutdown();
    cam_sub_.shutdown();
}

int Vision::start(void)
  {
	ROS_INFO("%s", __func__);
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*pnh_));

    pnh_->param("debug_view", debug_view_, false);
    if (debug_view_ ) {
      always_subscribe_ = true;
    }
    prev_stamp_ = ros::Time(0, 0);

    img_pub_ = it_->advertise("image", 10);
    msg_pub_ = pnh_->advertise<asulada_core::FaceArrayStamped>("faces", 1);

    std::string face_cascade_name, eyes_cascade_name;
    pnh_->param("face_cascade_name", face_cascade_name, std::string("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"));
    pnh_->param("eyes_cascade_name", eyes_cascade_name, std::string("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml"));


    if( !face_cascade_.load( face_cascade_name ) )
	{
		ROS_ERROR("--Error loading %s", face_cascade_name.c_str());
	};
    if( !eyes_cascade_.load( eyes_cascade_name ) )
	{
		ROS_ERROR("--Error loading %s", eyes_cascade_name.c_str()); 
	};

	_subscribe();
	return 0;
}

void Vision::stop(void)
{
	_unsubscribe();
}

Vision *Vision::getInstance(ros::NodeHandle *nh)
{
	if (!inst_)
		inst_ = new Vision();

	inst_->pnh_ = nh;

	return inst_;
}
} // namespace asulada
