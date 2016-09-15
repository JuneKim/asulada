#include "MotionCtrl.h"
#include "Motor.h"
#include "Vision.h"
#include "IMotor.h"
#include "IVision.h"

namespace asulada {

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
	FaceArea_e area;
	int goal = 0;
	ROS_ERROR("[%f:%f] %f", x, y, dimension);
	if (curDimension_ / 2 < dimension && curFaceArea_ != _getFaceArea(x, y)) {
		// TODO: set arc
		area = _getFaceArea(x, y);
		if (area != curFaceArea_) {
			goal = _faceArea2Arc(area);
			setMotorGoal(goal);
			curFaceArea_ = area;
		}
	}
}

void MotionCtrl::onCurrentMotorStatus(int pos)
{
	curMotorPos_ = pos;
}

FaceArea_e  MotionCtrl::_getFaceArea(double x, double y)
{
	
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
