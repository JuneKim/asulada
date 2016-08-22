#include "MotionCtrl.h"
#include "Motor.h"
#include "Vision.h"
#include "IMotor.h"
#include "IVision.h"

namespace asulada {

MotionCtrl::MotionCtrl(ros::NodeHandle *nh)
: curX_(0.0)
,  curY_(0.0)
, curZ_(0.0)
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
	ROS_ERROR("[%f:%f] %f", x, y, x);
}

void MotionCtrl::onCurrentMotorStatus()
{
}


} // namespace asulada
