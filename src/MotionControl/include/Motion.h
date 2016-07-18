#ifndef __MOTOR_H__
#define __MOTOR_H__

class Motor {
public:
	Motor();
	~Motor();
	int start();
	void stop();
	void addListener(IMotorWPtr l);
	void removeListener(IMotorWPtr l);


	void move(double x);
private:
	void _notify();
	

	static const double max_;

};

#endif /* __MOTOR_H__ */
