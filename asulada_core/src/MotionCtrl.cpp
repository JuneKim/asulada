#include <std_msgs/String.h>
#include "MotionCtrl.h"
#include "Motor.h"
#include "Vision.h"
#include "IMotor.h"
#include "IVision.h"

namespace asulada {

int MotionCtrl::INIT_ARC = 2000;
int MotionCtrl::gTarget_ = 0;

MotionCtrl::MotionCtrl(ros::NodeHandle *nh)
: curX_(0.0)
,  curY_(0.0)
, curDimension_(0.0)
, curFaceArea_(FACE_AREA_2)
, pnh_(nh)
{
	motor_ = Motor::getInstance(nh);
	vision_ = Vision::getInstance(nh);
}

MotionCtrl::~MotionCtrl()
{
}

int MotionCtrl::start()
{
	if (motor_) {
		motor_->start();
		motor_->addListener(this);
	}
	if (vision_) {
		vision_->start();
		vision_->addListener(this);
	}
	setMotorGoal(MotionCtrl::INIT_ARC);
	led_pub_ = pnh_->advertise<std_msgs::String>("ctrl" , 10);
}

void MotionCtrl::stop()
{
	if (vision_) {
		vision_->stop();
		vision_->removeListener(this);
	}

	if (motor_) {
		motor_->stop();
		motor_->removeListener(this);
	}
}

void MotionCtrl::setLed(int idx)
{
	ROS_INFO("set LED");
	std_msgs::String str;
	ros::Rate r(60);

	switch(idx) {
	case 1:
		str.data = "f";
		break;
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		str.data = "v";
		break;
	default:
		break;
	}
	led_pub_.publish(str);
}

void MotionCtrl::onFaceDetected(double x, double y, double dimension)
{
	static int tmp = 0;
	FaceArea_e area;
	int goal = 0;
	ROS_INFO("[%f:%f] %f", x, y, dimension);
	if (curDimension_ / 2 < dimension && curFaceArea_ != _getFaceArea(x, y)) {
		// TODO: set arc
		ROS_INFO("Moved.... and set Arc");
		area = _getFaceArea(x, y);
		if (area != curFaceArea_) {
			setLed(1);
			ROS_INFO("curMotorPos_ [%d]", curMotorPos_);
			gTarget_ = MotionCtrl::INIT_ARC + 110 * (curFaceArea_ - area);
			setMotorGoal(gTarget_); // tmp
			setLed(7);
			curFaceArea_ = area;
		}
	}
}

void MotionCtrl::onCurrentMotorStatus(int pos)
{
	ROS_INFO("current pos[%d]", pos);
	curMotorPos_ = pos;
	if (pos == gTarget_) {
		curFaceArea_ = FACE_AREA_2;
		setLed(7);
	}
}

FaceArea_e  MotionCtrl::_getFaceArea(double x, double y)
{
	FaceArea_e area = FACE_AREA_1;
	if (x > 200) {
		area = FACE_AREA_3;
	} else if (x > 100) {
		area = FACE_AREA_2;
	} else {
		area = FACE_AREA_1;
	}

	return area;
}

void MotionCtrl::_setCurrentArea(FaceArea_e area)
{
}

// convert arc from area
int MotionCtrl::_getCurrentMotorGoal(FaceArea_e area)
{
	int goal = 0;

	return goal;	
}

int MotionCtrl::_faceArea2Arc(FaceArea_e area)
{
	int target = 0;
	return target;
}

void MotionCtrl::setMotorGoal(int goal)
{
	ROS_INFO("goal [%d]", goal);
	if (motor_)
		motor_->setGoal(goal);
}

} // namespace asulada
