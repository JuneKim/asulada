#include <vector>
#include <iostream>

#include "Motor.h"
#include "IMotor.h"

using namespace std;

asulada::Motor *asulada::Motor::inst_ = NULL;
namespace asulada {

Motor::Motor()
: pnh_(NULL)
{
}

Motor::~Motor()
{
}

Motor *Motor::getInstance(ros::NodeHandle *nh)
{
	if (inst_)
		inst_ = new Motor();
	inst_->pnh_ = nh;

	return inst_;
}

int Motor::start()
{
	return 0;
}

void Motor::stop()
{
}

void Motor::addListener(IMotor *l)
{
	listeners_.push_back(l);
}

void Motor::removeListener(IMotor *l)
{
}

void Motor::_notify()
{
}

} // namespace asulada
