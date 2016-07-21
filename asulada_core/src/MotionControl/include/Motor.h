#ifndef __MOTOR_H__
#define __MOTOR_H__

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
	void _notify();
	std::vector<MotorInfo*> info_;

	static const double max_;

};

#endif /* __MOTOR_H__ */
