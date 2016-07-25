#include "Motor.h"

static Motor* Motor::inst_ = nullptr;
static Motor* Motor::getInstance()
{
	if (!inst_)
		inst_ = new Motor;

	return inst_;
}

Motor::Motor()
: max_(10000)
, nh_(new NodeHandle)
{
}

Motor::~Motor()
{
}

static void Motor::onPosSubscribed(const std_msgs::Int32::ConstPtr& position)
{
	ROS_INFO("pos updated:%s", position.c_str());
}

static void Motor::onMotorStatusSubscribed(const MotorStatus::ConstPtr& status)
{
	MotorInfo* info = _getMotorInfo(status.id);
	if (info == null) {
		info = new MotorInfo;
		*info = status;
		info_.push_back(info);
		return;
	}
	info = status;
}

int Motor::start()
{
	pos_sub_ = nh_->subscribe<std_msgs::Int32>("/asulada_current_position", onPosSubscribed, this);
	status_pub_ nh_->subscribe<MotorStatus>("/asulada_motor_status", onMotorStatusSubscribed, this);
	goal_pub_ = nh->advertise<std_msgs::Int32>("/asulada_goal_position", 1);
}

void Motor::stop()
{
	pos_sub_.shutdown();
	goal_pub_.shutdown();
}

void Motor::addListener(IMotor* l)
{
	listeners_.push_back(l);
}

void Motor::removeListener(IMotor* l)
{
	for (auto itor : listeners_) {
		if (*itor == l) {
			listeners_.erase(itor);
			break;
		}
	}
}

int Motor::setGoal(int id, int goal)
{
	std_msgs::Int32 goal_pos;
	goal_pos.data = goal;
	goal_pub_.publish(goal_pos);
}

int Motor::setSpeed(int id, int speed)
{
	//TODO
}

int Motor::getGoal(int id)
{
	MotorInfo* info = _getMotorInfo(id);
	return info ? info->goal : 0;
}

bool Motor::isMoving(int id)
{
	MotorInfo* info = _getMotorInfo(id);
	return info ? info->moving : false;
}

MotorInfo* Motor::_getMotorInfo(int id)
{
	for (auto itor : listeners_) {
		if (*itor->id == id) {
			return *itor;
		}
	}
	return nullptr;
}

void _notify(const MotorInfo&& info)
{
	for (auto itor : listeners_) {
		*itor->onMotorUpdated(info);
	}
}

};

#endif /* __MOTOR_H__ */
