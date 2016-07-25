#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <ros/ros.h>
#include <std_msgs/Int32.h>

class MotorInfo {
public:
	float timestamp;
	int id;
	int goal;
	int position;
	int error;
	int speed;
	float load;
	float voltage;
	int temperature;
	bool moving;
};

class Motor {
public:
	static Motor* getInstance();
	static void onPosSubscribed(const std_msgs::Int32::ConstPtr& position);
	static void onMotorStatusSubscribed(const MotorStatus::ConstPtr& status);
	
	Motor();
	~Motor();
	int start();
	void stop();
	void addListener(IMotorWPtr l);
	void removeListener(IMotorWPtr l);
	int setGoal(int id, int goal);
	int setSpeed(int id, int speed);
	
	int getGoal(int id);
	bool isMoving(int id);
private:
	void _notify(const MotorInfo&& info);
	MotorInfo* _getMotorInfo(int id);
	

	static Motor* inst_;
	NodoHandle* nh_;
	std::vector<MotorInfo*> info_;
	ros::Subscriber pos_sub_;
	ros::Subscriber status_pub_;
	ros::Publisher goal_pub_;

	static const double max_;
};

#endif /* __MOTOR_H__ */
