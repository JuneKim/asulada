#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <vector>

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
	
	static Motor *getInstance();

private:
	void _notify();

private:
	static Motor *inst_;
	std::vector<IMotor *> listeners_;
};

}

#endif /* __MOTOR_H__ */
