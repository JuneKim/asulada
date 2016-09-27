#include "MotionCtrl.h"
#include "Motor.h"
#include "Vision.h"
#include "IMotor.h"
#include "IVision.h"

namespace asulada {

int MotionCtrl::INIT_ARC = 2000;

MotionCtrl::MotionCtrl(ros::NodeHandle *nh)
: curX_(0.0)
,  curY_(0.0)
, curDimension_(0.0)
, curFaceArea_(FACE_AREA_3)
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
#if 0
			goal = _faceArea2Arc(area);
			setMotorGoal(goal);
#else
			ROS_INFO("curMotorPos_ [%d]", curMotorPos_);
			setMotorGoal(MotionCtrl::INIT_ARC + 400 * (curFaceArea_ - area)); // tmp
#endif
			curFaceArea_ = area;
		}
	}
}

void MotionCtrl::onCurrentMotorStatus(int pos)
{
	ROS_INFO("current pos[%d]", pos);
	curMotorPos_ = pos;
}

FaceArea_e  MotionCtrl::_getFaceArea(double x, double y)
{
	FaceArea_e area = FACE_AREA_1;
	if (x > 200) {
		area = FACE_AREA_5;
	} else if (x > 150) {
		area = FACE_AREA_4;
	} else if (x > 100) {
		area = FACE_AREA_3;
	} else if (x > 50) {
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
