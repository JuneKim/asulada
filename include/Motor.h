#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <memory>

namespace Asulada {

class Motor;
typedef std::shared_ptr<Motor> MotorPtr;
typedef std::weak_ptr<Motor> MotorWPtr;

class Motor {
public:
	static MotorWPtr getInstance();
	Motor();
	~Motor();
	int enable();
	int disable();
	int setArc(int x, int y);

private:
	static MotorPtr* inst_;
}; // class Motor

} // namespace Asulada

#endif /* __MOTOR_H */
