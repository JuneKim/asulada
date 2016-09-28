#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

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
	void setGoal(int goal);
	
	static Motor *getInstance(ros::NodeHandle *nh);
	static void positionCallback(const std_msgs::Int32::ConstPtr& position);

private:
	void _notify(int pos);
	int _updateTime();
	int _isValidTime();


private:
	ros::NodeHandle *pnh_;
	int currentPos_;
	double curTime_;
	
	ros::Publisher goal_pub_;
	ros::Subscriber pos_sub_;
	std::vector<IMotor *> listeners_;

	static Motor *inst_;
};

}

#endif /* __MOTOR_H__ */
