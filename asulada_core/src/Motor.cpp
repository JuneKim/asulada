#include <vector>
#include <iostream>
#include <ctime>

#include "Motor.h"
#include "IMotor.h"

using namespace std;

namespace asulada {

Motor *Motor::inst_ = NULL;

Motor::Motor()
: pnh_(NULL)
, curTime_(0.0)
{
}

Motor::~Motor()
{
}

void Motor::positionCallback(const std_msgs::Int32::ConstPtr& position)
{
	ROS_INFO("current_pos[%d]", position->data);
	inst_->currentPos_ = position->data;
	inst_->_notify(position->data);
}

Motor *Motor::getInstance(ros::NodeHandle *nh)
{
	if (!inst_)
		inst_ = new Motor();
	inst_->pnh_ = nh;
	inst_->goal_pub_ = inst_->pnh_->advertise<std_msgs::Int32>("/asulada_goal_position", 10);

	return inst_;
}

int Motor::start()
{
	if (!pos_sub_)
		pos_sub_ = pnh_->subscribe("/asulada_current_position", 1, &positionCallback);
	_updateTime();

	return 0;
}

void Motor::stop()
{
	//TODO : stop subscribe
}

int Motor::_isValidTime()
{
	double curTime = clock() / 1000;
	if (curTime - curTime_ < 3) {
		ROS_INFO("## wait for seconds...");
		return 0;
	}
	return 1;
}

int Motor::_updateTime()
{
	curTime_ = clock() / 1000;
}

void Motor::setGoal(int goal)
{
	ROS_INFO("set goal : %d", goal);
	if (_isValidTime() != 1) {
		return;
	}
	std_msgs::Int32 target;
	target.data = goal;
	if (goal_pub_) {
		goal_pub_.publish(target);
		_updateTime();
	} else
		ROS_ERROR("invalid goal pub_");
}

void Motor::addListener(IMotor *l)
{
	listeners_.push_back(l);
}

void Motor::removeListener(IMotor *l)
{
	vector<IMotor*>::iterator itor;

	for (itor = listeners_.begin(); itor != listeners_.end(); itor++) {
		if (*itor == l) {
			listeners_.erase(itor);
			break;
		}
	}
}

void Motor::_notify(int pos)
{
	vector<IMotor*>::iterator itor;
	if (listeners_.size() > 0) {
		for (itor = listeners_.begin(); itor != listeners_.end(); itor++) {
			((asulada::IMotor*)*itor)->onCurrentMotorStatus(pos);
		}
	}
}

} // namespace asulada
