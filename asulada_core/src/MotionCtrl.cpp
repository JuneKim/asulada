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
	if (motor_)
		motor_->start();

	if (vision_)
		vision_->start();
}

void MotionCtrl::stop()
{
	if (vision_)
		vision_->stop();

	if (motor_)
		motor_->stop();
}

void MotionCtrl::onFaceDetected(double x, double y, double dimension)
{
	ROS_ERROR("[%f:%f] %f", x, y, dimension);
	if (curDimension_ / 2 < dimension && curFaceArea_ != _getFaceArea(x, y)) {
		// TODO: set arc
	}
}

void MotionCtrl::onCurrentMotorStatus(int pos)
{
	curMotorPos_ = pos;
}

FaceArea_e  MotionCtrl::_getFaceArea(double x, double y)
{
	
}


} // namespace asulada
