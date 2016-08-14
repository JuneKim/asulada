#include "MotionCtrl.h"
#include "Motor.h"
#include "Vision.h"
#include "IMotor.h"
#include "IVision.h"

namespace asulada {

MotionCtrl::MotionCtrl()
: curX_(0.0)
,  curY_(0.0)
, curZ_(0.0)
{
	motor_ = Motor::getInstance();
	vision_ = Vision::getInstance();
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

void MotionCtrl::onFaceDetected(double x, double y, double z)
{
}

void MotionCtrl::onCurrentMotorStatus()
{
}


} // namespace asulada
