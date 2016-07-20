#include <string>

#include "Common.h"

using namespace std;

namespace Asulada
{
MotionControl::MotionControl()
: isStarted_(false), motor_(nullptr), camera_(nullptr)
{
	motor_ = Moter::getInstance();
	camera_ = Camera::getInstance();
}

MotionControl::~MotionControl()
{
	delete motor_;
	delete camera_;
}

Error_t int MotionControl::start()
{
	motor_->start();
	motor_->addListener(this);
	camera_->start();
	camera_->addListener(this);
	isStarted_ = true;


	return ERROR_NONE;
}

void MotionControl::stop()
{
	if (isStarted_ == false) {
		ROS_INFO("not started");
		return;
	}

	motor->removeListener(this);
	camera_->removeListener(this);

	motor_->stop();
	camera_->stop();
}

int MotionControl::getValue(double& x, double& y, double& z)
{
	if (isStarted != true)
		return ERROR_NOT_INITIALIZED;

	x = x_;
	y = y_;
	z = z_;

	return ERROR_NONE;
}

void MotionControl::setValue(double x, double y, double z)
{
	x_ = x;
	y_ = y;
	z_ = z;
}

void MotionControl::onPosUpdated(double x, double y, double z)
{
	setValue(x, y, z);
}

void MotionControl::onFaceMoved(double xdiff, double ydiff, double zdiff)
{
	// calculate movement
	double x, y, z;

	// move motor....
	motor_->move(x, y, z);
}

} // Asulada
