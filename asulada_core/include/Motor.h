#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <vector>
#include <ros/ros.h>

namespace asulada {

class IMotor;

class Motor {
public:
	Motor();
	~Motor();
	int start();
	void stop();
	void addListener(IMotor *l);
	void removeListener(IMotor *l);
	
	static Motor *getInstance(ros::NodeHandle *nh);

private:
	void _notify();

private:
	ros::NodeHandle *pnh_;
	static Motor *inst_;
	std::vector<IMotor *> listeners_;
};

}

#endif /* __MOTOR_H__ */
